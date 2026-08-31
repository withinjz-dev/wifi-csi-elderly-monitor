"""One-off: replay a saved CSI CSV through live_monitor's classify() logic and
append the results to calibration_log.csv under a given label, so calibrate.py
has real data to work with before any live --label session has been run.

Usage: python3 backfill_calibration_log.py <csv_path> <label>
"""
import re
import sys
import argparse
from collections import deque

import numpy as np

real_argv = sys.argv[1:]
sys.path.insert(0, "/Users/parkjuan/Desktop/ESP_project_2026/data")
sys.argv = ["live_monitor.py", "/dev/null"]  # dummy args so live_monitor's own argparse doesn't choke on import
import live_monitor as lm
sys.argv = ["backfill_calibration_log.py"] + real_argv


def run(csv_path, label):
    times, rssis, amps = [], [], []
    with open(csv_path) as f:
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
    while t + lm.WINDOW_SEC <= times[-1]:
        window_starts.append(t)
        t += lm.STEP_SEC

    lm.SPACE_LABEL = label
    recent_breath_freqs = deque(maxlen=lm.BREATH_HISTORY_LOOKBACK)
    count = 0

    for ws in window_starts:
        we = ws + lm.WINDOW_SEC
        idx = [i for i, tt in enumerate(times) if ws <= tt < we]
        if len(idx) < 5:
            continue
        bt = [times[i] for i in idx]
        br = [rssis[i] for i in idx]
        ba = [amps[i] for i in idx]

        state, info = lm.classify(bt, br, ba)
        if state == "SLEEP_NORMAL":
            this_freq = info.get("peak_freq")
            matched = any(abs(this_freq - f) <= lm.BREATH_FREQ_TOLERANCE_HZ for f in recent_breath_freqs)
            if this_freq is not None:
                recent_breath_freqs.append(this_freq)
            if not matched:
                state = "NO_RESPONSE"
            info["confirmed"] = 1.0 if matched else 0.0

        lm.log_calibration_row(state, info)
        count += 1

    print(f"Backfilled {count} rows from {csv_path} under label '{label}'")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("csv_path")
    p.add_argument("label")
    a = p.parse_args()
    run(a.csv_path, a.label)
