import re
import sys
import numpy as np
from scipy.signal import butter, filtfilt, welch
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FILE = sys.argv[1] if len(sys.argv) > 1 else "breathing_test_03.csv"
OUT_PNG = sys.argv[2] if len(sys.argv) > 2 else "breathing_analysis.png"

BLUE = "#2a78d6"
ORANGE = "#eb6834"
GRID = "#d9d8d3"
TEXT = "#52514e"

times = []
amps = []

with open(FILE) as f:
    for line in f:
        line = line.strip()
        if not line.startswith("CSI_DATA,"):
            continue
        m = re.search(r"\[(.*)\],([0-9.]+)$", line)
        if not m:
            continue
        csi_str, ts_str = m.group(1), m.group(2)
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
        times.append(float(ts_str))

times = np.array(times)
amps = np.array(amps)
times = times - times[0]

print(f"rows parsed: {len(times)}")
print(f"duration: {times[-1]:.1f}s")
print(f"avg rate: {len(times)/times[-1]:.2f} Hz")

# Resample onto uniform grid
fs = 10.0  # target uniform sample rate (Hz), well above breathing band
t_uniform = np.arange(0, times[-1], 1.0 / fs)
amp_uniform = np.interp(t_uniform, times, amps)

# Detrend (remove slow drift) before filtering
amp_detrended = amp_uniform - np.convolve(amp_uniform, np.ones(int(fs * 10)) / (fs * 10), mode="same")

# Bandpass filter for breathing band (0.15 - 0.5 Hz)
lo, hi = 0.15, 0.5
b, a = butter(4, [lo / (fs / 2), hi / (fs / 2)], btype="band")
filtered = filtfilt(b, a, amp_detrended)

# Power spectrum of the detrended signal
freqs, psd = welch(amp_detrended, fs=fs, nperseg=min(1024, len(amp_detrended)))
band_mask = (freqs >= 0.05) & (freqs <= 1.0)
breathing_mask = (freqs >= lo) & (freqs <= hi)

peak_idx = np.argmax(psd[breathing_mask])
peak_freq = freqs[breathing_mask][peak_idx]
peak_power = psd[breathing_mask][peak_idx]
noise_floor = np.median(psd[band_mask])
snr = peak_power / noise_floor if noise_floor > 0 else float("nan")

print(f"peak frequency in breathing band: {peak_freq:.3f} Hz ({peak_freq*60:.1f} breaths/min)")
print(f"peak power: {peak_power:.4f}, noise floor (median): {noise_floor:.4f}, SNR: {snr:.2f}x")

def zscore(x):
    return (x - np.mean(x)) / (np.std(x) + 1e-9)

fig, axes = plt.subplots(2, 1, figsize=(11, 7), constrained_layout=True)

ax = axes[0]
ax.plot(t_uniform, zscore(amp_detrended), color=BLUE, linewidth=1.2, label="CSI amplitude (detrended, normalized)")
ax.plot(t_uniform, zscore(filtered), color=ORANGE, linewidth=1.6, label="Bandpass filtered (0.15-0.5 Hz)")
ax.set_xlabel("Time (s)", color=TEXT)
ax.set_ylabel("Normalized amplitude", color=TEXT)
ax.set_title("CSI Amplitude Over Time — Raw vs. Breathing-band Filtered", color=TEXT, fontsize=12)
ax.grid(True, color=GRID, linewidth=0.6)
ax.legend(frameon=False)
for spine in ["top", "right"]:
    ax.spines[spine].set_visible(False)

ax2 = axes[1]
ax2.plot(freqs[band_mask], psd[band_mask], color=BLUE, linewidth=1.6)
ax2.axvspan(lo, hi, color=ORANGE, alpha=0.15, label="Breathing band (0.15-0.5 Hz)")
ax2.axvline(peak_freq, color=ORANGE, linewidth=1.6, linestyle="--", label=f"Peak: {peak_freq:.3f} Hz ({peak_freq*60:.1f} /min)")
ax2.set_xlabel("Frequency (Hz)", color=TEXT)
ax2.set_ylabel("Power", color=TEXT)
ax2.set_title(f"Power Spectrum — SNR in breathing band: {snr:.2f}x", color=TEXT, fontsize=12)
ax2.grid(True, color=GRID, linewidth=0.6)
ax2.legend(frameon=False)
for spine in ["top", "right"]:
    ax2.spines[spine].set_visible(False)

plt.savefig(OUT_PNG, dpi=150, facecolor="white")
print(f"saved plot to {OUT_PNG}")
