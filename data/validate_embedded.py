"""
호스트 레퍼런스 구현 vs 임베디드 알고리즘 교차검증.

목적: ESP32에 올리기 위해 Welch/scipy 기반 신호처리를 Goertzel/biquad 기반으로
재설계했다. 연산을 가볍게 만든 것이 판정 성능을 훼손하지 않았음을 같은 녹화
데이터로 확인한다.

여기 구현된 EmbeddedPipeline은 _components/edge_dsp.h, edge_csi.h,
edge_monitor.h의 C 코드를 1:1로 옮긴 것이다 (float64로 도는 점만 다르다).
따라서 이 스크립트가 내는 판정은 펌웨어가 같은 입력에서 낼 판정과 같아야 한다.

  레퍼런스 (PC)                      임베디드 (ESP32)
  ─────────────────                  ─────────────────
  sqrt(I²+Q²) 전체 평균         →     I²+Q², 선별된 8개 서브캐리어만
  np.interp 타임스탬프 리샘플   →     100ms 시간 빈 + zero-order hold
  scipy.signal.detrend          →     최소자승 직선 제거
  Welch PSD (전 대역)           →     Goertzel 7개 빈 (0.2~0.5Hz)
  피크/잡음중앙값 SNR           →     스펙트럼 집중도 x 빈 개수
"""
import os
import re
import sys
from collections import deque

import numpy as np
from scipy.signal import welch, detrend as sp_detrend

DATA_DIR = os.path.dirname(os.path.abspath(__file__))

# --- 펌웨어 상수와 동일하게 유지 (edge_dsp.h) ---
FS = 10.0
BIN_US = 100_000
WINDOW_N = 256
STEP_SEC = 10.0
BREATH_LO, BREATH_HI = 0.2, 0.5
N_BINS = 7
SELECTED_MAX = 8
SC_LOW_BASE, SC_HIGH_BASE = 6, 33
FREQ_TOL = 0.12
FREQ_HIST = 3


def sc_slot(idx):
    return idx + SC_LOW_BASE if idx < 26 else idx - 26 + SC_HIGH_BASE


# ─────────────────────────── 임베디드 알고리즘 ───────────────────────────

class Biquad:
    """edge_biquad_* 의 이식. RBJ cookbook band-pass."""

    def __init__(self, f_lo, f_hi, fs):
        f0 = np.sqrt(f_lo * f_hi)
        q = f0 / (f_hi - f_lo)
        w0 = 2 * np.pi * f0 / fs
        alpha = np.sin(w0) / (2 * q)
        a0 = 1 + alpha
        self.b0, self.b1, self.b2 = alpha / a0, 0.0, -alpha / a0
        self.a1, self.a2 = (-2 * np.cos(w0)) / a0, (1 - alpha) / a0
        self.reset()

    def reset(self):
        self.x1 = self.x2 = self.y1 = self.y2 = 0.0

    def step(self, x):
        y = (self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
             - self.a1 * self.y1 - self.a2 * self.y2)
        self.x2, self.x1 = self.x1, x
        self.y2, self.y1 = self.y1, y
        return y

    def run(self, xs):
        self.reset()
        return np.array([self.step(float(x)) for x in xs])


def goertzel_power(x, coeff):
    """edge_goertzel_power 의 이식."""
    s1 = s2 = 0.0
    for v in x:
        s = v + coeff * s1 - s2
        s2, s1 = s1, s
    return s1 * s1 + s2 * s2 - coeff * s1 * s2


class GoertzelBank:
    def __init__(self, fs):
        step = (BREATH_HI - BREATH_LO) / (N_BINS - 1)
        self.freq = np.array([BREATH_LO + step * i for i in range(N_BINS)])
        self.coeff = 2.0 * np.cos(2 * np.pi * self.freq / fs)

    def analyze(self, x):
        p = np.array([max(0.0, goertzel_power(x, c)) for c in self.coeff])
        total = p.sum()
        best = int(np.argmax(p))
        conc = p[best] / total if total > 1e-9 else 0.0
        return self.freq[best], conc * N_BINS   # score: 1.0 == 평탄한 잡음


def embedded_detrend(x):
    """edge_monitor.h 의 최소자승 직선 제거."""
    n = len(x)
    i = np.arange(n, dtype=float)
    denom = n * (i * i).sum() - i.sum() ** 2
    slope = (n * (i * x).sum() - i.sum() * x.sum()) / denom if abs(denom) > 1e-6 else 0.0
    icpt = (x.sum() - slope * i.sum()) / n
    return x - (slope * i + icpt)


def select_subcarriers(frames):
    """edge_score_subcarriers / edge_finish_selection 의 이식."""
    n_sc = 52
    raw_var = np.zeros(n_sc)
    band_var = np.zeros(n_sc)
    for i in range(n_sc):
        slot = sc_slot(i)
        pw = frames[:, slot * 2].astype(float) ** 2 + frames[:, slot * 2 + 1].astype(float) ** 2
        raw_var[i] = pw.var(ddof=1)
        band_var[i] = Biquad(BREATH_LO, BREATH_HI, FS).run(pw).var(ddof=1)

    rv_max = raw_var.max()
    score = np.where((raw_var < 1e-3) | (raw_var < 0.02 * rv_max), -1.0,
                     band_var / np.maximum(raw_var, 1e-12))
    order = [i for i in np.argsort(-score) if score[i] >= 0][:SELECTED_MAX]
    return order, score


def time_bin(times_us, feats):
    """edge_ingest_frame 의 100ms 시간 빈 + zero-order hold."""
    out = []
    bin_start = times_us[0]
    acc, cnt = 0.0, 0
    last, have_last = 0.0, False
    for t, f in zip(times_us, feats):
        while t - bin_start >= BIN_US:
            if cnt > 0:
                v = acc / cnt
                last, have_last = v, True
            elif have_last:
                v = last
            else:
                v = f
            out.append(v)
            acc, cnt = 0.0, 0
            bin_start += BIN_US
        acc += f
        cnt += 1
    return np.array(out)


# ─────────────────────────── 호스트 레퍼런스 ───────────────────────────

def reference_decide(times, rssi, amps):
    """live_monitor.py 의 classify() 와 동일한 판정 경로."""
    rssi_std = float(np.std(rssi))
    t0 = times - times[0]
    tu = np.arange(0, t0[-1], 1.0 / FS)
    au = np.interp(tu, t0, amps)
    seg = sp_detrend(au, type="linear")
    freqs, psd = welch(seg, fs=FS, nperseg=min(len(seg), 256))
    band = (freqs >= 0.05) & (freqs <= 1.0)
    br = (freqs >= BREATH_LO) & (freqs <= BREATH_HI)
    if not br.any() or psd[band].size == 0:
        return rssi_std, 0.0, 0.0
    peak = float(psd[br].max())
    pf = float(freqs[br][np.argmax(psd[br])])
    floor = float(np.median(psd[band])) or 1e-9
    return rssi_std, peak / floor, pf


# ─────────────────────────── 실행 ───────────────────────────

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(DATA_DIR, "breathing_test_03.csv")

    times, rssis, raw = [], [], []
    with open(path) as f:
        for line in f:
            if not line.startswith("CSI_DATA,"):
                continue
            m = re.search(r"\[(.*)\],([0-9.]+)$", line.strip())
            if not m:
                continue
            v = [int(x) for x in m.group(1).split(" ") if x]
            if len(v) != 128:
                continue
            raw.append(v)
            rssis.append(float(line.split(",")[3]))
            times.append(float(m.group(2)))

    raw = np.array(raw, dtype=np.int16)
    times = np.array(times)
    rssis = np.array(rssis)
    t_us = ((times - times[0]) * 1e6).astype(np.int64)
    print(f"입력: {len(raw)} 프레임, {times[-1] - times[0]:.1f}초, "
          f"평균 {len(raw) / (times[-1] - times[0]):.2f} Hz\n")

    # --- 서브캐리어 선택 (임베디드에만 있는 단계) ---
    sel, score = select_subcarriers(raw)
    print("서브캐리어 자동 선택 (대역/전체 분산비 상위 8개):")
    for i in sel:
        print(f"   idx {i:2d} (slot {sc_slot(i):2d})  비율 {score[i]:.4f}")
    all_sc = [i for i in range(52) if score[i] >= 0]
    print(f"\n   선택된 8개 평균 비율 {np.mean([score[i] for i in sel]):.4f}"
          f"  vs 전체 {len(all_sc)}개 평균 {np.mean([score[i] for i in all_sc]):.4f}"
          f"   ({np.mean([score[i] for i in sel]) / np.mean([score[i] for i in all_sc]):.2f}배)\n")

    # --- 프레임별 스칼라 특징 ---
    emb_feat = np.mean([raw[:, sc_slot(i) * 2].astype(float) ** 2
                        + raw[:, sc_slot(i) * 2 + 1].astype(float) ** 2 for i in sel], axis=0)
    ref_amp = np.array([np.mean(np.sqrt(r[0::2].astype(float) ** 2 + r[1::2].astype(float) ** 2)[
                            np.sqrt(r[0::2].astype(float) ** 2 + r[1::2].astype(float) ** 2) > 0])
                        for r in raw])

    binned = time_bin(t_us, emb_feat)
    bank = GoertzelBank(FS)

    # --- 윈도우별 비교 ---
    rel_t = times - times[0]
    starts = np.arange(0, rel_t[-1] - 25.6, STEP_SEC)

    emb_hist, ref_hist = deque(maxlen=FREQ_HIST), deque(maxlen=FREQ_HIST)
    rows, agree_state, agree_breath = [], 0, 0

    for ws in starts:
        we = ws + 25.6
        idx = np.where((rel_t >= ws) & (rel_t < we))[0]
        if len(idx) < 20:
            continue

        # 임베디드 경로
        b_end = int(we * FS)
        b_start = b_end - WINDOW_N
        if b_start < 0 or b_end > len(binned):
            continue
        w = binned[b_start:b_end].astype(float)
        w = embedded_detrend(w)
        w = Biquad(BREATH_LO, BREATH_HI, FS).run(w)
        e_pf, e_score = bank.analyze(w)
        e_motion = float(np.std(rssis[idx]))

        # 레퍼런스 경로
        r_motion, r_snr, r_pf = reference_decide(times[idx], rssis[idx], ref_amp[idx])

        # 두 경로 모두 동일한 상태기계·일관성 규칙 적용
        def decide(motion, score_, pf, hist, m_thr, b_thr):
            if motion > m_thr:
                hist.clear()
                return "ACTIVE", False
            if score_ > b_thr:
                ok = any(abs(pf - h) <= FREQ_TOL for h in hist)
                hist.append(pf)
                return ("SLEEP_NORMAL", True) if ok else ("NO_RESPONSE", False)
            return "NO_RESPONSE", False

        # 움직임 임계값은 양쪽 동일 (검증된 mu+3sigma 방식 결과값)
        M_THR = 1.33
        e_state, e_ok = decide(e_motion, e_score, e_pf, emb_hist, M_THR, 1.8)
        r_state, r_ok = decide(r_motion, r_snr, r_pf, ref_hist, M_THR, 2.0)

        agree_state += (e_state == r_state)
        agree_breath += (e_ok == r_ok)
        rows.append((ws, r_state, e_state, r_snr, e_score, r_pf, e_pf))

    n = len(rows)
    print(f"{'t(s)':>6} {'레퍼런스':>14} {'임베디드':>14} {'ref SNR':>9} {'emb score':>10} "
          f"{'ref Hz':>7} {'emb Hz':>7}  일치")
    print("-" * 88)
    for ws, rs, es, rsnr, esc, rpf, epf in rows:
        mark = "O" if rs == es else "X"
        print(f"{ws:>6.0f} {rs:>14} {es:>14} {rsnr:>9.2f} {esc:>10.2f} "
              f"{rpf:>7.2f} {epf:>7.2f}   {mark}")

    print("-" * 88)
    print(f"\n상태 판정 일치율   {agree_state}/{n} = {100 * agree_state / n:.1f}%")
    print(f"호흡 확정 일치율   {agree_breath}/{n} = {100 * agree_breath / n:.1f}%")

    rp = np.array([r[5] for r in rows])
    ep = np.array([r[6] for r in rows])
    both = np.array([r[1] == "SLEEP_NORMAL" and r[2] == "SLEEP_NORMAL" for r in rows])
    if both.sum() > 0:
        print(f"\n양쪽 모두 SLEEP_NORMAL 인 {both.sum()}개 윈도우의 피크 주파수 차이:")
        print(f"   평균 절대오차 {np.mean(np.abs(rp[both] - ep[both])):.3f} Hz"
              f"  (Goertzel 빈 간격 0.05 Hz)")


if __name__ == "__main__":
    main()
