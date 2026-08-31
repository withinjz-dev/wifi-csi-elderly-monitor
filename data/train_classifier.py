"""
Trains a lightweight breathing-presence classifier from calibration_log.csv,
using session-protocol ground truth: rows logged under label "empty" are
known-negative (no one in the room, so no breathing signal); rows from any
other session (desk/bed/floor/...) are known-positive whenever the person
wasn't moving (ACTIVE rows are excluded -- that's a separate, already-solved
decision handled by the RSSI threshold, not what this model is for).

Compares the trained model's cross-validated accuracy against the existing
hand-tuned SNR-threshold rule on the same data, and saves a comparison chart.
"""
import os
import csv
import pickle
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = "AppleGothic"
matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt

from sklearn.linear_model import LogisticRegression
from sklearn.model_selection import StratifiedKFold, cross_val_predict
from sklearn.metrics import accuracy_score, balanced_accuracy_score, confusion_matrix, classification_report

DATA_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_PATH = os.path.join(DATA_DIR, "calibration_log.csv")
MODEL_PATH = os.path.join(DATA_DIR, "breathing_classifier.pkl")

RULE_SNR_THRESHOLD = 2.0  # the existing hand-tuned rule, for comparison

rows = []
with open(LOG_PATH) as f:
    for row in csv.DictReader(f):
        if row["snr"] == "" or row["state"] not in ("SLEEP_NORMAL", "NO_RESPONSE"):
            continue
        rows.append(row)

X, y, rule_pred = [], [], []
for row in rows:
    rssi_std = float(row["rssi_std"])
    snr = float(row["snr"])
    peak_freq = float(row["peak_freq"])
    X.append([rssi_std, snr, peak_freq])
    y.append(0 if row["label"] == "empty" else 1)
    rule_pred.append(1 if snr > RULE_SNR_THRESHOLD else 0)

X = np.array(X)
y = np.array(y)
rule_pred = np.array(rule_pred)

print(f"total samples: {len(y)} (positive/breathing={y.sum()}, negative/empty={len(y) - y.sum()})")
print("NOTE: classes are imbalanced (69 vs 14) -- raw accuracy is misleading here "
      "(always predicting the majority class alone scores ~83%). Balanced accuracy "
      "(average of per-class recall) is the metric that actually matters.")

rule_acc = accuracy_score(y, rule_pred)
rule_bal_acc = balanced_accuracy_score(y, rule_pred)
print(f"\n--- existing rule (SNR > {RULE_SNR_THRESHOLD}) ---")
print(f"accuracy: {rule_acc:.3f}  balanced_accuracy: {rule_bal_acc:.3f}")
print(confusion_matrix(y, rule_pred))

cv = StratifiedKFold(n_splits=5, shuffle=True, random_state=0)
# class_weight="balanced" reweights the loss inversely to class frequency so the
# model can't just collapse to predicting the majority class -- without it, a
# plain LogisticRegression here scores 83% accuracy by predicting "breathing"
# for every single row, including all 14 true negatives (0% recall on "empty").
model = LogisticRegression(class_weight="balanced")
cv_pred = cross_val_predict(model, X, y, cv=cv)
model_acc = accuracy_score(y, cv_pred)
model_bal_acc = balanced_accuracy_score(y, cv_pred)
print(f"\n--- trained logistic regression, class_weight=balanced (5-fold cross-validated) ---")
print(f"accuracy: {model_acc:.3f}  balanced_accuracy: {model_bal_acc:.3f}")
print(confusion_matrix(y, cv_pred))
print(classification_report(y, cv_pred, target_names=["no breathing", "breathing"]))

model.fit(X, y)
print("\nlearned coefficients [rssi_std, snr, peak_freq]:", model.coef_[0])
print("intercept:", model.intercept_[0])

with open(MODEL_PATH, "wb") as f:
    pickle.dump(model, f)
print(f"\nsaved trained model to {MODEL_PATH}")

# --- comparison chart ---
BLUE = "#2a78d6"
ORANGE = "#eb6834"
TEXT = "#52514e"
GRID = "#d9d8d3"

fig, axes = plt.subplots(1, 2, figsize=(10, 4.5), constrained_layout=True)

ax = axes[0]
labels = ["기존 규칙\n(SNR>2.0)", "학습된 분류기\n(로지스틱회귀)"]
x = np.arange(2)
width = 0.32
bars1 = ax.bar(x - width / 2, [rule_acc * 100, model_acc * 100], width, color=BLUE, label="정확도")
bars2 = ax.bar(x + width / 2, [rule_bal_acc * 100, model_bal_acc * 100], width, color=ORANGE, label="균형정확도")
for bars in (bars1, bars2):
    for bar in bars:
        h = bar.get_height()
        ax.text(bar.get_x() + bar.get_width() / 2, h + 1.5, f"{h:.0f}%", ha="center", color=TEXT, fontsize=10)
ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.set_ylim(0, 110)
ax.set_ylabel("%", color=TEXT)
ax.set_title("규칙 기반 vs 학습된 분류기\n(정확도는 불균형 데이터에서 오해 소지 있음)", color=TEXT, fontsize=11)
ax.legend(frameon=False, fontsize=9)
ax.grid(True, axis="y", color=GRID, linewidth=0.6)
for spine in ["top", "right"]:
    ax.spines[spine].set_visible(False)

ax2 = axes[1]
cm = confusion_matrix(y, cv_pred)
im = ax2.imshow(cm, cmap="Blues")
ax2.set_xticks([0, 1])
ax2.set_yticks([0, 1])
ax2.set_xticklabels(["호흡없음", "호흡있음"])
ax2.set_yticklabels(["호흡없음", "호흡있음"])
ax2.set_xlabel("예측", color=TEXT)
ax2.set_ylabel("실제", color=TEXT)
ax2.set_title("분류기 혼동행렬 (5-fold CV)", color=TEXT, fontsize=12)
for i in range(2):
    for j in range(2):
        ax2.text(j, i, str(cm[i, j]), ha="center", va="center",
                  color="white" if cm[i, j] > cm.max() / 2 else TEXT, fontsize=14)

plt.savefig(os.path.join(DATA_DIR, "classifier_comparison.png"), dpi=150, facecolor="white")
print("saved classifier_comparison.png")
