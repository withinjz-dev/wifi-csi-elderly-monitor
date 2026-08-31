import re
import sys
from collections import deque
sys.path.insert(0, "/Users/parkjuan/Desktop/ESP_project_2026/data")
from live_monitor import classify, WINDOW_SEC, STEP_SEC, BREATH_FREQ_TOLERANCE_HZ, BREATH_HISTORY_LOOKBACK

FILE = sys.argv[1] if len(sys.argv) > 1 else "breathing_test_03.csv"

times, rssis, amps = [], [], []
with open(FILE) as f:
    for line in f:
        line = line.strip()
        if not line.startswith("CSI_DATA,"):
            continue
        m = re.search(r"\[(.*)\],([0-9.]+)$", line)
        if not m:
            continue
        csi_str, ts_str = m.group(1), m.group(2)
        fields = line.split(",")
        rssi = float(fields[3])
        vals = [int(x) for x in csi_str.split(" ") if x != ""]
        if len(vals) < 4:
            continue
        import numpy as np
        imag = np.array(vals[0::2], dtype=float)
        real = np.array(vals[1::2], dtype=float)
        amplitude = np.sqrt(imag**2 + real**2)
        nonzero = amplitude[amplitude > 0]
        if len(nonzero) == 0:
            continue
        amps.append(float(np.mean(nonzero)))
        rssis.append(rssi)
        times.append(float(ts_str))

t0 = times[0]
times = [t - t0 for t in times]

window_starts = []
t = 0
while t + WINDOW_SEC <= times[-1]:
    window_starts.append(t)
    t += STEP_SEC

recent_breath_freqs = deque(maxlen=BREATH_HISTORY_LOOKBACK)
confirmed_count = 0
sleep_normal_raw_count = 0
total = 0
max_streak = 0
streak = 0

for ws in window_starts:
    we = ws + WINDOW_SEC
    idx = [i for i, tt in enumerate(times) if ws <= tt < we]
    if len(idx) < 5:
        continue
    bt = [times[i] for i in idx]
    br = [rssis[i] for i in idx]
    ba = [amps[i] for i in idx]

    state, info = classify(bt, br, ba)
    total += 1
    if state == "SLEEP_NORMAL":
        sleep_normal_raw_count += 1

    confirmed = False
    if state == "SLEEP_NORMAL":
        this_freq = info.get("peak_freq")
        confirmed = any(abs(this_freq - f) <= BREATH_FREQ_TOLERANCE_HZ for f in recent_breath_freqs)
        if this_freq is not None:
            recent_breath_freqs.append(this_freq)
        if confirmed:
            confirmed_count += 1

    final_state = state if (state != "SLEEP_NORMAL" or confirmed) else "NO_RESPONSE"
    if final_state in ("ACTIVE", "SLEEP_NORMAL"):
        streak = 0
    else:
        streak += 1
        max_streak = max(max_streak, streak)

    print(f"[{ws:6.1f}s] raw={state:14s} confirmed={confirmed} streak={streak} " +
          ", ".join(f"{k}={v:.3f}" for k, v in info.items()))

print(f"\ntotal windows: {total}")
print(f"raw SLEEP_NORMAL (before consistency check): {sleep_normal_raw_count} ({100*sleep_normal_raw_count/total:.0f}%)")
print(f"confirmed SLEEP_NORMAL (after consistency check): {confirmed_count} ({100*confirmed_count/total:.0f}%)")
print(f"max consecutive non-confirmed streak: {max_streak} windows ({max_streak*10}s) -- alert fires at streak>=6 (60s)")
