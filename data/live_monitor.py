import re
import os
import csv
import json
import argparse
import time
import datetime
from collections import deque

import serial
import numpy as np
from scipy.signal import butter, filtfilt, welch, detrend

DATA_DIR = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_LOG_PATH = os.path.join(DATA_DIR, "calibration_log.csv")
CALIBRATION_CONFIG_PATH = os.path.join(DATA_DIR, "calibration.json")

parser = argparse.ArgumentParser(description="Live CSI/RSSI presence+breathing monitor")
parser.add_argument("port", help="Serial port of the AP board, e.g. /dev/cu.usbserial-0001")
parser.add_argument("baud", nargs="?", type=int, default=115200)
parser.add_argument("--label", default="default",
                     help="Space/session label (e.g. bed, floor) -- tags rows in calibration_log.csv "
                          "so per-space thresholds can be calibrated later")
args = parser.parse_args()

PORT = args.port
BAUD = args.baud
SPACE_LABEL = args.label

WINDOW_SEC = 20.0
STEP_SEC = 10.0
FS = 10.0
BREATH_LO, BREATH_HI = 0.2, 0.5  # 12-30 breaths/min; keeps clear of low-frequency drift artifacts
RSSI_MOVE_THRESHOLD = 1.5
BREATH_SNR_THRESHOLD = 2.0

# A single window's "breathing peak" is often just noise -- real breathing
# holds roughly the same rate for a while, so require the peak frequency to
# stay consistent across consecutive windows before trusting it.
BREATH_FREQ_TOLERANCE_HZ = 0.12
BREATH_HISTORY_LOOKBACK = 3  # windows to search for a matching peak frequency

# Only escalate to a physical alert during these local hours (overnight range).
SLEEP_HOUR_START = 22
SLEEP_HOUR_END = 7

# Require this many consecutive NO_RESPONSE windows (~ NO_RESPONSE_STREAK * STEP_SEC seconds)
# before triggering the buzzer/LED, to avoid single-window false alarms.
NO_RESPONSE_STREAK_REQUIRED = 6  # 6 * 10s = 60s sustained

# --- Adaptive calibration -------------------------------------------------
# calibrate.py analyzes calibration_log.csv (built up across sessions/spaces)
# and writes suggested per-label threshold overrides here. If present, they
# replace the hardcoded defaults above for whichever label this run uses.
if os.path.exists(CALIBRATION_CONFIG_PATH):
    with open(CALIBRATION_CONFIG_PATH) as _f:
        _calib = json.load(_f)
    _overrides = _calib.get(SPACE_LABEL) or _calib.get("default") or {}
    if _overrides:
        RSSI_MOVE_THRESHOLD = _overrides.get("rssi_move_threshold", RSSI_MOVE_THRESHOLD)
        BREATH_SNR_THRESHOLD = _overrides.get("breath_snr_threshold", BREATH_SNR_THRESHOLD)
        print(f"Loaded calibration for label '{SPACE_LABEL}': {_overrides}")

CSI_ROW_RE = re.compile(r"\[(.*)\]")


def in_sleep_hours(now=None):
    now = now or datetime.datetime.now()
    h = now.hour
    if SLEEP_HOUR_START > SLEEP_HOUR_END:
        return h >= SLEEP_HOUR_START or h < SLEEP_HOUR_END
    return SLEEP_HOUR_START <= h < SLEEP_HOUR_END


def classify(buf_t, buf_rssi, buf_amp):
    if len(buf_t) < 5:
        return "NO_DATA", {}

    t = np.array(buf_t)
    rssi = np.array(buf_rssi)
    amp = np.array(buf_amp)

    rssi_std = float(np.std(rssi))
    if rssi_std > RSSI_MOVE_THRESHOLD:
        return "ACTIVE", {"rssi_std": rssi_std}

    t0 = t - t[0]
    t_uniform = np.arange(0, t0[-1], 1.0 / FS) if t0[-1] > 1 else np.array([0])
    if len(t_uniform) < int(FS * WINDOW_SEC * 0.5):
        return "NO_DATA", {"rssi_std": rssi_std}

    amp_uniform = np.interp(t_uniform, t0, amp)
    # Mean-subtraction alone leaves slow baseline drift in the signal, which
    # shows up as spurious low-frequency power right at the bottom edge of the
    # breathing band. A linear detrend removes that drift.
    seg = detrend(amp_uniform, type="linear")

    freqs, psd = welch(seg, fs=FS, nperseg=min(len(seg), 256))
    band_mask = (freqs >= 0.05) & (freqs <= 1.0)
    breath_mask = (freqs >= BREATH_LO) & (freqs <= BREATH_HI)
    if not breath_mask.any() or psd[band_mask].size == 0:
        return "NO_RESPONSE", {"rssi_std": rssi_std, "snr": 0.0}

    peak_power = float(np.max(psd[breath_mask]))
    peak_freq = float(freqs[breath_mask][np.argmax(psd[breath_mask])])
    noise_floor = float(np.median(psd[band_mask])) or 1e-9
    snr = peak_power / noise_floor

    info = {"rssi_std": rssi_std, "snr": snr, "peak_freq": peak_freq}
    if snr > BREATH_SNR_THRESHOLD:
        return "SLEEP_NORMAL", info
    return "NO_RESPONSE", info


CALIBRATION_LOG_FIELDS = ["timestamp", "label", "state", "rssi_std", "snr", "peak_freq", "confirmed"]


def log_calibration_row(state, info):
    is_new = not os.path.exists(CALIBRATION_LOG_PATH)
    with open(CALIBRATION_LOG_PATH, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CALIBRATION_LOG_FIELDS)
        if is_new:
            writer.writeheader()
        writer.writerow({
            "timestamp": datetime.datetime.now().isoformat(),
            "label": SPACE_LABEL,
            "state": state,
            "rssi_std": info.get("rssi_std", ""),
            "snr": info.get("snr", ""),
            "peak_freq": info.get("peak_freq", ""),
            "confirmed": info.get("confirmed", ""),
        })


def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    ser.dtr = False
    ser.rts = False

    print(f"Live monitoring {PORT} @ {BAUD} [label={SPACE_LABEL}]. Ctrl+C to stop.")
    print(f"Logging calibration data to {CALIBRATION_LOG_PATH}")

    buf_t, buf_rssi, buf_amp = deque(), deque(), deque()
    no_response_streak = 0
    alert_active = False
    last_window_time = time.time()
    recent_breath_freqs = deque(maxlen=BREATH_HISTORY_LOOKBACK)

    try:
        while True:
            raw = ser.readline()
            if raw:
                try:
                    line = raw.decode("utf-8", errors="ignore").strip()
                except Exception:
                    line = ""
                if line.startswith("CSI_DATA,"):
                    m = CSI_ROW_RE.search(line)
                    if m:
                        fields = line.split(",")
                        try:
                            rssi = float(fields[3])
                            vals = [int(x) for x in m.group(1).split(" ") if x != ""]
                            if len(vals) >= 4:
                                imag = np.array(vals[0::2], dtype=float)
                                real = np.array(vals[1::2], dtype=float)
                                amplitude = np.sqrt(imag**2 + real**2)
                                nonzero = amplitude[amplitude > 0]
                                if len(nonzero) > 0:
                                    now_t = time.time()
                                    buf_t.append(now_t)
                                    buf_rssi.append(rssi)
                                    buf_amp.append(float(np.mean(nonzero)))
                                    cutoff = now_t - WINDOW_SEC
                                    while buf_t and buf_t[0] < cutoff:
                                        buf_t.popleft()
                                        buf_rssi.popleft()
                                        buf_amp.popleft()
                        except (ValueError, IndexError):
                            pass

            now = time.time()

            # Prune stale samples on a wall-clock basis, independent of whether
            # new data has arrived -- otherwise a full STA disconnect leaves the
            # buffer frozen with the last-known window forever.
            cutoff = now - WINDOW_SEC
            while buf_t and buf_t[0] < cutoff:
                buf_t.popleft()
                buf_rssi.popleft()
                buf_amp.popleft()

            if now - last_window_time >= STEP_SEC:
                last_window_time = now
                state, info = classify(list(buf_t), list(buf_rssi), list(buf_amp))

                # A single window's breathing-band peak is not trustworthy on its
                # own -- confirm it only if it lines up with another recent
                # window's peak (real breathing holds a roughly steady rate over
                # a minute; noise peaks land at random frequencies each time and
                # rarely repeat). Looking back across the last few windows (not
                # just the immediately preceding one) tolerates the occasional
                # noisy/missed window without needing back-to-back detections.
                if state == "SLEEP_NORMAL":
                    this_freq = info.get("peak_freq")
                    matched = any(
                        abs(this_freq - f) <= BREATH_FREQ_TOLERANCE_HZ
                        for f in recent_breath_freqs
                    )
                    if this_freq is not None:
                        recent_breath_freqs.append(this_freq)
                    if not matched:
                        state = "NO_RESPONSE"
                    info["confirmed"] = 1.0 if matched else 0.0

                log_calibration_row(state, info)

                extra = ", ".join(f"{k}={v:.2f}" for k, v in info.items())
                ts = datetime.datetime.now().strftime("%H:%M:%S")

                if state in ("ACTIVE", "SLEEP_NORMAL"):
                    no_response_streak = 0
                elif state in ("NO_RESPONSE", "NO_DATA"):
                    no_response_streak += 1

                sleep_hours = in_sleep_hours()
                should_alert = (
                    no_response_streak >= NO_RESPONSE_STREAK_REQUIRED
                    and sleep_hours
                )

                print(f"[{ts}] {state:14s} streak={no_response_streak} sleep_hours={sleep_hours} {extra}")

                if should_alert and not alert_active:
                    alert_active = True
                    ser.write(b"ALERT_ON\n")
                    print(f"[{ts}] *** ALERT TRIGGERED (sustained no-response during sleep hours) ***")
                elif not should_alert and alert_active:
                    alert_active = False
                    ser.write(b"ALERT_OFF\n")
                    print(f"[{ts}] alert cleared")

    except KeyboardInterrupt:
        print("\nStopped.")
        if alert_active:
            ser.write(b"ALERT_OFF\n")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
