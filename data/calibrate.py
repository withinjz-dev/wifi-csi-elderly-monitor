"""
Analyzes calibration_log.csv (built up by live_monitor.py --label <space> across
sessions/spaces) and suggests per-space threshold overrides.

For each label, finds a natural split point in the rssi_std and snr
distributions using 1D Otsu thresholding (the same technique used for image
binarization) -- an unsupervised method that needs no ground-truth labels,
just "this metric is probably bimodal (quiet vs. active/breathing)".

Usage:
    python3 calibrate.py                # print suggestions only
    python3 calibrate.py --apply        # write calibration.json
"""
import os
import csv
import json
import argparse
from collections import defaultdict

import numpy as np

DATA_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_PATH = os.path.join(DATA_DIR, "calibration_log.csv")
CONFIG_PATH = os.path.join(DATA_DIR, "calibration.json")


def otsu_threshold(values):
    """1D Otsu's method: find the split point maximizing between-class variance."""
    values = np.asarray(sorted(values))
    if len(values) < 4:
        return None
    best_t, best_score = None, -1
    for i in range(1, len(values)):
        lo, hi = values[:i], values[i:]
        if len(lo) == 0 or len(hi) == 0:
            continue
        w_lo, w_hi = len(lo) / len(values), len(hi) / len(values)
        score = w_lo * w_hi * (lo.mean() - hi.mean()) ** 2
        if score > best_score:
            best_score = score
            best_t = (lo[-1] + hi[0]) / 2
    return best_t


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--apply", action="store_true", help="write calibration.json instead of just printing")
    args = parser.parse_args()

    if not os.path.exists(LOG_PATH):
        print(f"No log found at {LOG_PATH} -- run live_monitor.py first to build up history.")
        return

    rows_by_label = defaultdict(list)
    with open(LOG_PATH) as f:
        for row in csv.DictReader(f):
            rows_by_label[row["label"]].append(row)

    suggestions = {}
    for label, rows in rows_by_label.items():
        rssi_stds = [float(r["rssi_std"]) for r in rows if r["rssi_std"]]
        snrs = [float(r["snr"]) for r in rows if r["snr"]]

        rssi_t = otsu_threshold(rssi_stds)
        snr_t = otsu_threshold(snrs)

        print(f"\n=== label: {label} ({len(rows)} rows) ===")
        if rssi_t is not None:
            print(f"  rssi_std: min={min(rssi_stds):.2f} max={max(rssi_stds):.2f} "
                  f"-> suggested RSSI_MOVE_THRESHOLD = {rssi_t:.2f} (current default: 1.5)")
        else:
            print("  rssi_std: not enough data yet")

        if snr_t is not None:
            print(f"  snr: min={min(snrs):.2f} max={max(snrs):.2f} "
                  f"-> suggested BREATH_SNR_THRESHOLD = {snr_t:.2f} (current default: 2.0)")
        else:
            print("  snr: not enough data yet")

        entry = {}
        if rssi_t is not None:
            entry["rssi_move_threshold"] = round(float(rssi_t), 3)
        if snr_t is not None:
            entry["breath_snr_threshold"] = round(float(snr_t), 3)
        if entry:
            suggestions[label] = entry

    if not suggestions:
        print("\nNothing to suggest yet -- collect more data with live_monitor.py --label <space>.")
        return

    if args.apply:
        with open(CONFIG_PATH, "w") as f:
            json.dump(suggestions, f, indent=2, ensure_ascii=False)
        print(f"\nWrote {CONFIG_PATH} -- live_monitor.py will pick these up automatically per --label.")
    else:
        print(f"\n(dry run -- re-run with --apply to write {CONFIG_PATH})")


if __name__ == "__main__":
    main()
