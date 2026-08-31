import re
import sys
import numpy as np
from scipy.signal import butter, filtfilt, welch
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
matplotlib.rcParams["font.family"] = "AppleGothic"
matplotlib.rcParams["axes.unicode_minus"] = False

FILE = sys.argv[1] if len(sys.argv) > 1 else "breathing_test_03.csv"
OUT_PNG = sys.argv[2] if len(sys.argv) > 2 else "day3_classification.png"

BLUE = "#2a78d6"
GOOD = "#0ca30c"
CRITICAL = "#d03b3b"
GRID = "#d9d8d3"
TEXT = "#52514e"

WINDOW_SEC = 20.0
STEP_SEC = 10.0
FS = 10.0  # resample rate for CSI amplitude
BREATH_LO, BREATH_HI = 0.15, 0.5
RSSI_MOVE_THRESHOLD = 1.5   # std(RSSI) above this within a window => movement
BREATH_SNR_THRESHOLD = 2.0  # peak/noise-floor ratio above this => breathing present

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
        imag = np.array(vals[0::2], dtype=float)
        real = np.array(vals[1::2], dtype=float)
        amplitude = np.sqrt(imag**2 + real**2)
        nonzero = amplitude[amplitude > 0]
        if len(nonzero) == 0:
            continue
        amps.append(np.mean(nonzero))
        rssis.append(rssi)
        times.append(float(ts_str))

times = np.array(times) - times[0]
rssis = np.array(rssis)
amps = np.array(amps)

t_uniform_full = np.arange(0, times[-1], 1.0 / FS)
amp_uniform_full = np.interp(t_uniform_full, times, amps)

def classify_window(t0, t1):
    mask = (times >= t0) & (times < t1)
    if mask.sum() < 5:
        return "NO_DATA", {}

    rssi_std = float(np.std(rssis[mask]))

    if rssi_std > RSSI_MOVE_THRESHOLD:
        return "ACTIVE", {"rssi_std": rssi_std}

    umask = (t_uniform_full >= t0) & (t_uniform_full < t1)
    seg = amp_uniform_full[umask]
    if len(seg) < int(FS * WINDOW_SEC * 0.5):
        return "NO_DATA", {"rssi_std": rssi_std}

    seg = seg - np.mean(seg)
    b, a = butter(4, [BREATH_LO / (FS / 2), BREATH_HI / (FS / 2)], btype="band")
    try:
        filtfilt(b, a, seg)
    except Exception:
        pass

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
    else:
        return "NO_RESPONSE", info

window_starts = np.arange(0, times[-1] - WINDOW_SEC, STEP_SEC)
states = []
for t0 in window_starts:
    state, info = classify_window(t0, t0 + WINDOW_SEC)
    states.append((t0, t0 + WINDOW_SEC, state, info))
    extra = ", ".join(f"{k}={v:.2f}" for k, v in info.items())
    print(f"[{t0:6.1f}s - {t0+WINDOW_SEC:6.1f}s] {state:14s} {extra}")

color_map = {"ACTIVE": BLUE, "SLEEP_NORMAL": GOOD, "NO_RESPONSE": CRITICAL, "NO_DATA": "#c3c2b7"}

fig, axes = plt.subplots(2, 1, figsize=(12, 6), constrained_layout=True, gridspec_kw={"height_ratios": [3, 1]})

ax = axes[0]
ax.plot(times, rssis, color=BLUE, linewidth=1.0)
ax.set_ylabel("RSSI (dBm)", color=TEXT)
ax.set_title("RSSI over time", color=TEXT, fontsize=12)
ax.grid(True, color=GRID, linewidth=0.6)
for spine in ["top", "right"]:
    ax.spines[spine].set_visible(False)

ax2 = axes[1]
for t0, t1, state, info in states:
    ax2.barh(0, t1 - t0, left=t0, height=0.8, color=color_map[state], edgecolor="white", linewidth=0.5)
ax2.set_yticks([])
ax2.set_xlabel("Time (s)", color=TEXT)
ax2.set_title("Classified state (20s windows)", color=TEXT, fontsize=12)
for spine in ["top", "right", "left"]:
    ax2.spines[spine].set_visible(False)

handles = [plt.Rectangle((0, 0), 1, 1, color=color_map[s]) for s in ["ACTIVE", "SLEEP_NORMAL", "NO_RESPONSE"]]
labels = ["활동중 (ACTIVE)", "정상수면 (SLEEP_NORMAL)", "무응답 (NO_RESPONSE)"]
ax2.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, -0.6), ncol=3, frameon=False)

plt.savefig(OUT_PNG, dpi=150, facecolor="white")
print(f"saved plot to {OUT_PNG}")
