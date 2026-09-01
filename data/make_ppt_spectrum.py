"""
PPT용 단일 패널 호흡 스펙트럼 차트.

analyze_breathing.py의 데이터 처리를 그대로 따르되 두 가지만 바꾼다.
  1. 대역을 0.2~0.5Hz로 맞춘다. 최종 펌웨어가 쓰는 대역이며, 0.15Hz 하한은
     저주파 드리프트가 대역 하단으로 누설되는 문제 때문에 폐기했다
     (개발완료보고서 3.3절 (4)a).
  2. 슬라이드 한 장에 들어가도록 스펙트럼 패널만, 한글 라벨로 그린다.
"""
import re
import sys
import numpy as np
from scipy.signal import welch
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

FILE = sys.argv[1] if len(sys.argv) > 1 else "breathing_test_03.csv"
OUT = sys.argv[2] if len(sys.argv) > 2 else "../figures/호흡스펙트럼.png"

# 한글 폰트 (macOS 기본)
for cand in ["AppleSDGothicNeo-Regular", "Apple SD Gothic Neo", "AppleGothic", "NanumGothic"]:
    try:
        font_manager.findfont(cand, fallback_to_default=False)
        plt.rcParams["font.family"] = cand
        break
    except Exception:
        continue
plt.rcParams["axes.unicode_minus"] = False

BLUE, ORANGE, TEXT, GRID = "#2a78d6", "#eb6834", "#37474f", "#d9d8d3"

# ---- 데이터 로드 (analyze_breathing.py와 동일) ----
times, amps = [], []
with open(FILE) as f:
    for line in f:
        line = line.strip()
        if not line.startswith("CSI_DATA,"):
            continue
        m = re.search(r"\[(.*)\],([0-9.]+)$", line)
        if not m:
            continue
        vals = [int(x) for x in m.group(1).split(" ") if x != ""]
        if len(vals) < 4:
            continue
        imag = np.array(vals[0::2], dtype=float)
        real = np.array(vals[1::2], dtype=float)
        amplitude = np.sqrt(imag**2 + real**2)
        nz = amplitude[amplitude > 0]
        if len(nz) == 0:
            continue
        amps.append(np.mean(nz))
        times.append(float(m.group(2)))

times = np.array(times) - times[0]
amps = np.array(amps)

fs = 10.0
t_u = np.arange(0, times[-1], 1.0 / fs)
amp_u = np.interp(t_u, times, amps)
amp_d = amp_u - np.convolve(amp_u, np.ones(int(fs * 10)) / (fs * 10), mode="same")

freqs, psd = welch(amp_d, fs=fs, nperseg=min(1024, len(amp_d)))

LO, HI = 0.2, 0.5                      # 최종 펌웨어 대역
view = (freqs >= 0.05) & (freqs <= 1.0)
band = (freqs >= LO) & (freqs <= HI)

peak_i = np.argmax(psd[band])
peak_f = freqs[band][peak_i]
peak_p = psd[band][peak_i]
noise = np.median(psd[view])
snr = peak_p / noise

print(f"프레임 {len(times)}개 / {times[-1]:.0f}초 / 평균 {len(times)/times[-1]:.1f} Hz")
print(f"피크 {peak_f:.3f} Hz ({peak_f*60:.1f} 회/분) / 대역 SNR {snr:.2f}배")

# ---- 그리기 ----
fig, ax = plt.subplots(figsize=(12.8, 6.4), dpi=200)

ax.axvspan(LO, HI, color=ORANGE, alpha=0.13,
           label=f"호흡 대역 ({LO}~{HI} Hz)")
ax.plot(freqs[view], psd[view], color=BLUE, linewidth=2.0)
ax.axvline(peak_f, color=ORANGE, linestyle="--", linewidth=2.2,
           label=f"검출 피크  {peak_f:.3f} Hz  ({peak_f*60:.1f} 회/분)")

ax.annotate(f"{peak_f:.3f} Hz\n{peak_f*60:.1f} 회/분",
            xy=(peak_f, peak_p), xytext=(peak_f + 0.11, peak_p * 0.94),
            fontsize=15, fontweight="bold", color=ORANGE,
            arrowprops=dict(arrowstyle="->", color=ORANGE, linewidth=1.8))

ax.set_xlabel("주파수 (Hz)", fontsize=15, color=TEXT)
ax.set_ylabel("전력", fontsize=15, color=TEXT)
ax.set_title("WiFi CSI에서 검출된 호흡 주기성", fontsize=20, fontweight="bold",
             color="#1c2833", pad=14)
ax.set_xlim(0.05, 1.0)
ax.set_ylim(bottom=0)
ax.grid(color=GRID, linewidth=0.8)
ax.tick_params(labelsize=13, colors=TEXT)
for s in ("top", "right"):
    ax.spines[s].set_visible(False)
ax.legend(fontsize=13, loc="upper right", framealpha=0.95)

ax.text(0.012, -0.155,
        f"실측 데이터 · CSI {len(times):,}프레임 / {times[-1]:.0f}초 · "
        f"대역 내 SNR {snr:.1f}배 · 성인 정상 호흡수 12~20회/분",
        transform=ax.transAxes, fontsize=12, color="#607d8b")

fig.tight_layout()
fig.savefig(OUT, bbox_inches="tight", facecolor="white")
print(f"저장: {OUT}")
