"""
환경 정규화 설계 검증 -- 세 가지 방식을 실제 데이터로 비교한다.

배경: Day 5에서 공간별 Otsu 임계값이 5.18배 차이난다는 결과를 얻었다. 이를
"공간이 다르면 임계값이 달라야 한다"는 근거로 삼았으나, 아래 A안 검증 과정에서
그 해석에 결함이 있음이 드러났다 (본문 주석 참조).

  A안: (feature - mu_room) / sigma_room 으로 정규화한 뒤 Otsu 임계값을 sigma 단위로 표현
       -> 기각. 편차가 오히려 악화된다.

  B안: 정지 상태 baseline(mu, sigma)만 측정하고, 임계값을 mu + k*sigma 로 두되
       k는 모든 공간이 공유하는 상수 -> 채택 후보.

  대조군: 공간별 Otsu 임계값 (현재 방식)

B안이 임베디드에 유리한 이유: 기기는 Otsu를 돌릴 필요가 없다. 정지 구간의
누적 평균/분산(EWMA로 갱신 가능)만 유지하면 되고, k는 펌웨어 상수다.
"""
import csv
import json
import os
from collections import defaultdict

import numpy as np

DATA_DIR = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(DATA_DIR, "calibration_log.csv")
CALIB = os.path.join(DATA_DIR, "calibration.json")

STILL_STATES = {"SLEEP_NORMAL", "NO_RESPONSE"}
ROOMS = ["desk", "bed", "floor_boards_bed_person", "floor"]
K_CANDIDATES = [2.0, 3.0, 4.0, 5.0, 6.0]


def spread(v):
    v = [x for x in v if x > 0]
    return max(v) / min(v) if v else float("nan")


rows = defaultdict(list)
with open(LOG) as f:
    for r in csv.DictReader(f):
        if r["rssi_std"]:
            rows[r["label"]].append(r)

calib = json.load(open(CALIB))

stats = {}
for room in ROOMS:
    still = [float(r["rssi_std"]) for r in rows[room] if r["state"] in STILL_STATES]
    motion = [float(r["rssi_std"]) for r in rows[room] if r["state"] == "ACTIVE"]
    stats[room] = {
        "mu": float(np.mean(still)),
        "sigma": float(np.std(still)),
        "still_max": max(still),
        "motion": motion,
        "otsu": calib[room]["rssi_move_threshold"],
    }

print("=" * 84)
print("A안 검증: Otsu 임계값을 정지 baseline sigma 단위로 환산")
print("=" * 84)
a_vals = []
for room in ROOMS:
    s = stats[room]
    z = (s["otsu"] - s["mu"]) / s["sigma"]
    a_vals.append(z)
    print(f"  {room:<26} {s['otsu']:>6.2f}  ->  {z:>7.2f} sigma")
print(f"\n  원본 편차 {spread([stats[r]['otsu'] for r in ROOMS]):.2f}배"
      f"  ->  정규화 후 편차 {spread(a_vals):.2f}배   [기각]")

print()
print("=" * 84)
print("A안이 실패한 이유 -- Day 5 해석의 결함")
print("=" * 84)
print("  Otsu는 '정지 군집'과 '움직임 군집' 사이를 가른다. 따라서 임계값은")
print("  공간의 RF 특성뿐 아니라 그 세션에서 사람이 '얼마나 크게 움직였는가'에")
print("  함께 좌우된다. 실제 데이터를 보면:")
print()
print(f"  {'공간':<26} {'정지 μ':>8} {'정지 max':>9} {'움직임 N':>9} {'움직임 max':>11}")
print("  " + "-" * 68)
for room in ROOMS:
    s = stats[room]
    mx = max(s["motion"]) if s["motion"] else float("nan")
    print(f"  {room:<26} {s['mu']:>8.3f} {s['still_max']:>9.2f} "
          f"{len(s['motion']):>9} {mx:>11.2f}")
print()
mus = [stats[r]["mu"] for r in ROOMS]
print(f"  통제된 조건(정지)만 비교하면 공간 간 편차는 {spread(mus):.2f}배에 불과하다.")
print(f"  즉 Day 5의 '5.18배'는 상당 부분 세션별 움직임 강도 차이의 산물이며,")
print(f"  순수한 공간 RF 특성 차이는 그보다 훨씬 작다. (보고서 수정 필요)")

print()
print("=" * 84)
print("B안 검증: 공유 상수 k에 대해 threshold = mu + k*sigma")
print("=" * 84)
print(f"  {'k':>4} {'desk':>8} {'bed':>8} {'floor_bp':>9} {'floor':>8} "
      f"{'편차':>7}  {'정지 오탐':>9} {'움직임 미탐':>11}")
print("  " + "-" * 74)

best = None
for k in K_CANDIDATES:
    thrs, fp, fn, n_still, n_motion = [], 0, 0, 0, 0
    for room in ROOMS:
        s = stats[room]
        t = s["mu"] + k * s["sigma"]
        thrs.append(t)
        still = [float(r["rssi_std"]) for r in rows[room] if r["state"] in STILL_STATES]
        fp += sum(1 for x in still if x > t)      # 정지인데 움직임으로 오판
        n_still += len(still)
        fn += sum(1 for x in s["motion"] if x <= t)  # 움직임인데 놓침
        n_motion += len(s["motion"])
    sp = spread(thrs)
    print(f"  {k:>4.1f} {thrs[0]:>8.2f} {thrs[1]:>8.2f} {thrs[2]:>9.2f} {thrs[3]:>8.2f} "
          f"{sp:>6.2f}배 {fp:>4}/{n_still:<4} {fn:>6}/{n_motion:<4}")
    score = (fp / n_still) + (fn / n_motion)
    if best is None or score < best[1]:
        best = (k, score, thrs, sp, fp, n_still, fn, n_motion)

k, _, thrs, sp, fp, n_still, fn, n_motion = best
print()
print(f"  최적 k = {k:.1f}")
print(f"    임계값     {min(thrs):.2f} ~ {max(thrs):.2f}   편차 {sp:.2f}배"
      f"  (Otsu 방식 {spread([stats[r]['otsu'] for r in ROOMS]):.2f}배)")
print(f"    정지 오탐   {fp}/{n_still}")
print(f"    움직임 미탐 {fn}/{n_motion}")

print()
print("=" * 84)
print("결론")
print("=" * 84)
print("  · A안(사용자 제안 형태)은 기각 -- 정지 sigma는 움직임 규모를 담지 않는다.")
print("  · B안 채택 -- 정지 baseline + 공유 상수 k 하나로 모든 공간을 커버.")
print("  · 임베디드 이점: 기기가 Otsu를 돌릴 필요 없이 정지 구간의 mu/sigma만")
print("    EWMA로 갱신하면 되고, k는 펌웨어 상수다. NVS에 저장할 값은 실수 2개뿐.")
