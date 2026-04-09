#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
FOC Sampling Point Prediction Accuracy - Discrete Point Visualization

Show each FOC sampling instant as a discrete marker:
  - Black dots: ground truth (what FOC actually needs)
  - Red dots: no prediction (what FOC currently gets, delayed)
  - Blue dots: linear prediction
  - Arrows connecting predicted to truth for each point
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ── Actual system parameters ──
F_IMP = 253200.0          # Impedance control frequency
F_FOC_CLK = 36864000.0    # FOC clock
SVPWM_DIV = 2048          # SVPWM divider
F_FOC = F_FOC_CLK / SVPWM_DIV  # = 18000 Hz

DT_IMP = 1.0 / F_IMP     # 3.95 us
DT_FOC = 1.0 / F_FOC     # 55.56 us
RATIO = int(round(F_IMP / F_FOC))  # 14

KS, DS, KT_INV = 500.0, 30.0, 20.0


def iq_signal(t, freq, amp=0.005):
    omega = 2 * np.pi * freq
    xd = amp * np.sin(omega * t)
    vd = amp * omega * np.cos(omega * t)
    x = amp * 0.95 * np.sin(omega * t - 0.05)
    v = amp * 0.95 * omega * np.cos(omega * t - 0.05)
    return (KS * (xd - x) + DS * (vd - v)) * KT_INV


plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'DejaVu Serif'],
    'font.size': 9,
    'axes.linewidth': 0.8,
})

fig, axes = plt.subplots(4, 1, figsize=(16, 18), gridspec_kw={'height_ratios': [3, 3, 3, 2.5]})

# ── Show 3 frequencies ──
freqs = [20, 100, 500]
titles = [
    '(a) 20 Hz — Typical slow motion (robot arm normal operation)',
    '(b) 100 Hz — Typical fast motion',
    '(c) 500 Hz — Extreme high-speed (stress test)',
]

# Show ~2 full cycles for each frequency
all_results = []

for idx, (freq, title) in enumerate(zip(freqs, titles)):
    ax = axes[idx]

    n_cycles = 2.5
    T_show = n_cycles / freq
    N_foc_show = int(T_show / DT_FOC)
    N_imp_show = int(T_show / DT_IMP)

    t_foc = np.arange(N_foc_show) * DT_FOC
    t_imp = np.arange(N_imp_show) * DT_IMP

    # Continuous ground truth (high resolution for smooth curve)
    t_hires = np.linspace(0, T_show, 5000)
    iq_hires = np.array([iq_signal(t, freq) for t in t_hires])

    # Impedance control samples
    iq_imp = np.array([iq_signal(t, freq) for t in t_imp])

    # At each FOC sample point:
    iq_truth = []     # what's needed when torque is applied (t + DT_FOC)
    iq_no_pred = []   # what FOC reads now (t)
    iq_predicted = [] # linear extrapolation to t + DT_FOC

    for i in range(N_foc_show):
        t = t_foc[i]
        # Truth: value needed at apply time
        iq_truth.append(iq_signal(t + DT_FOC, freq))
        # No prediction: current value
        iq_no_pred.append(iq_signal(t, freq))
        # Linear prediction
        imp_idx = min(int(t / DT_IMP), N_imp_show - 1)
        if imp_idx >= 1:
            slope = (iq_imp[imp_idx] - iq_imp[imp_idx - 1]) / DT_IMP
            iq_predicted.append(iq_imp[imp_idx] + slope * DT_FOC)
        else:
            iq_predicted.append(iq_imp[imp_idx])

    iq_truth = np.array(iq_truth)
    iq_no_pred = np.array(iq_no_pred)
    iq_predicted = np.array(iq_predicted)

    t_foc_ms = t_foc * 1000

    # Compute errors
    err_no = np.abs(iq_no_pred - iq_truth)
    err_pred = np.abs(iq_predicted - iq_truth)
    rms_no = np.sqrt(np.mean(err_no**2))
    rms_pred = np.sqrt(np.mean(err_pred**2))
    iq_range = np.max(np.abs(iq_truth))
    pct_no = rms_no / iq_range * 100 if iq_range > 0 else 0
    pct_pred = rms_pred / iq_range * 100 if iq_range > 0 else 0
    improve = rms_no / rms_pred if rms_pred > 0 else float('inf')

    all_results.append((freq, iq_range, rms_no, rms_pred, pct_no, pct_pred, improve))

    # ── Plot ──
    # Continuous true signal
    ax.plot(t_hires * 1000, iq_hires, 'k-', linewidth=0.8, alpha=0.3, label='_nolegend_')

    # FOC sampling points
    ax.scatter(t_foc_ms, iq_truth, s=60, c='black', marker='o', zorder=5,
               label=f'Ground truth (at apply time)', edgecolors='black', linewidths=0.5)
    ax.scatter(t_foc_ms, iq_no_pred, s=40, c='red', marker='x', zorder=4,
               label=f'No prediction (err={pct_no:.2f}%)', linewidths=1.5)
    ax.scatter(t_foc_ms, iq_predicted, s=40, c='blue', marker='^', zorder=4,
               label=f'Linear prediction (err={pct_pred:.3f}%, {improve:.0f}x better)', linewidths=0.5)

    # Error arrows: connect no-prediction to truth
    for i in range(len(t_foc_ms)):
        # Red dashed line: no prediction error
        ax.plot([t_foc_ms[i], t_foc_ms[i]], [iq_no_pred[i], iq_truth[i]],
                'r-', linewidth=0.8, alpha=0.4)
        # Blue dashed line: prediction error (thinner, often invisible)
        ax.plot([t_foc_ms[i] + T_show * 0.3, t_foc_ms[i] + T_show * 0.3],
                [iq_predicted[i], iq_truth[i]],
                'b-', linewidth=0.8, alpha=0.4)

    ax.set_ylabel('$i_q$ ref (A)', fontsize=11)
    ax.set_title(title, fontsize=11, fontweight='bold')
    ax.legend(fontsize=8, loc='upper right')
    ax.grid(True, alpha=0.15)

    # Mark FOC period
    if idx == 0:
        y_pos = np.min(iq_truth) * 0.8
        ax.annotate('', xy=(t_foc_ms[1], y_pos), xytext=(t_foc_ms[0], y_pos),
                     arrowprops=dict(arrowstyle='<->', color='green', lw=1.5))
        ax.text((t_foc_ms[0] + t_foc_ms[1]) / 2, y_pos * 1.1,
                f'1 FOC period\n= {DT_FOC*1e6:.1f} μs',
                ha='center', va='bottom', fontsize=8, color='green')

axes[2].set_xlabel('Time (ms)', fontsize=11)

# ── Bottom: Summary bar chart ──
ax = axes[3]
x = np.arange(len(freqs))
width = 0.35

bars_no = [r[4] for r in all_results]    # pct no pred
bars_pred = [r[5] for r in all_results]   # pct predicted
improves = [r[6] for r in all_results]

b1 = ax.bar(x - width/2, bars_no, width, color='red', alpha=0.7, label='No prediction')
b2 = ax.bar(x + width/2, bars_pred, width, color='blue', alpha=0.7, label='Linear prediction')

ax.set_xticks(x)
ax.set_xticklabels([f'{f} Hz' for f in freqs], fontsize=11)
ax.set_ylabel('RMS Error (%)', fontsize=11)
ax.set_title('(d) Error Comparison Summary', fontsize=11, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.15, axis='y')

# Add improvement labels
for i, (b, imp) in enumerate(zip(b2, improves)):
    ax.annotate(f'{imp:.0f}x better',
                xy=(b.get_x() + b.get_width()/2, b.get_height()),
                xytext=(0, 8), textcoords='offset points',
                ha='center', fontsize=9, fontweight='bold', color='blue')

# Add percentage labels on bars
for b in b1:
    ax.text(b.get_x() + b.get_width()/2, b.get_height() + 0.1,
            f'{b.get_height():.2f}%', ha='center', va='bottom', fontsize=8, color='red')
for b in b2:
    ax.text(b.get_x() + b.get_width()/2, b.get_height() - 0.3,
            f'{b.get_height():.3f}%', ha='center', va='top', fontsize=8, color='white',
            fontweight='bold')

fig.suptitle(
    'FOC Delay Compensation: Predicted vs Ground Truth at Each FOC Sampling Point\n'
    f'System: foc_clk = {F_FOC_CLK/1e6:.3f} MHz, '
    f'SVPWM÷{SVPWM_DIV} = {F_FOC/1000:.0f} kHz, '
    f'Impedance = {F_IMP/1000:.0f} kHz, '
    f'FOC delay = {DT_FOC*1e6:.1f} μs',
    fontsize=12, fontweight='bold', y=1.0
)

plt.tight_layout()
out_path = 'D:/robot/impedance_control/figures/fig_prediction_discrete.png'
plt.savefig(out_path, dpi=200, bbox_inches='tight', facecolor='white')
plt.close()
print(f"Figure saved: {out_path}")

# Print table
print(f"\nSystem: FOC clock = {F_FOC_CLK/1e6:.3f} MHz / {SVPWM_DIV} = {F_FOC:.0f} Hz")
print(f"FOC period = {DT_FOC*1e6:.1f} μs, Oversampling ratio = {RATIO}\n")
print(f"{'Freq':>6} | {'Signal':>8} | {'NoPred':>10} | {'Predicted':>10} | {'Improve':>8}")
print(f"{'':>6} | {'Range(A)':>8} | {'RMS(%)':>10} | {'RMS(%)':>10} | {'':>8}")
print("-" * 58)
for freq, iq_range, rms_no, rms_pred, pct_no, pct_pred, improve in all_results:
    print(f"{freq:>5}Hz | {iq_range:>7.1f}A | {pct_no:>9.3f}% | {pct_pred:>9.4f}% | {improve:>6.0f}x")
