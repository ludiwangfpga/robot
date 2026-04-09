#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Prediction vs Ground Truth Comparison - Visual Accuracy Demo

直观展示线性预测的准确率：
  - 预测值 vs 真实值的波形对比
  - 误差放大图
  - 不同频率下的效果
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# System params
F_IMP = 253200.0
F_FOC = 18000.0
DT_IMP = 1.0 / F_IMP
DT_FOC = 1.0 / F_FOC
RATIO = int(round(F_IMP / F_FOC))

KS, DS, KT_INV = 500.0, 30.0, 20.0

T_SIM = 0.03  # 30ms
N_IMP = int(T_SIM / DT_IMP)
N_FOC = int(T_SIM / DT_FOC)
t_imp = np.arange(N_IMP) * DT_IMP
t_foc = np.arange(N_FOC) * DT_FOC


def iq_signal(t, freq, amp=0.005):
    omega = 2 * np.pi * freq
    xd = amp * np.sin(omega * t)
    vd = amp * omega * np.cos(omega * t)
    x = amp * 0.95 * np.sin(omega * t - 0.05)
    v = amp * 0.95 * omega * np.cos(omega * t - 0.05)
    return (KS * (xd - x) + DS * (vd - v)) * KT_INV


def run_prediction(freq):
    iq_imp = np.array([iq_signal(t, freq) for t in t_imp])

    # Ground truth: what FOC needs when it APPLIES torque (one FOC cycle later)
    iq_truth = np.array([iq_signal(t + DT_FOC, freq) for t in t_foc])

    # No prediction: FOC reads current value, applies one cycle later (stale)
    iq_no_pred = np.array([iq_signal(t, freq) for t in t_foc])

    # Linear prediction: extrapolate one FOC cycle ahead
    iq_pred = np.zeros(N_FOC)
    for i in range(N_FOC):
        idx = min(int(t_foc[i] / DT_IMP), N_IMP - 1)
        if idx >= 1:
            slope = (iq_imp[idx] - iq_imp[idx - 1]) / DT_IMP
            iq_pred[i] = iq_imp[idx] + slope * DT_FOC
        else:
            iq_pred[i] = iq_imp[idx]

    return iq_truth, iq_no_pred, iq_pred


# ════════════════════════════════════════
# Plot
# ════════════════════════════════════════
plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'DejaVu Serif'],
    'font.size': 9,
    'axes.linewidth': 0.8,
})

fig = plt.figure(figsize=(14, 16))
gs = GridSpec(6, 2, figure=fig, hspace=0.45, wspace=0.3,
              height_ratios=[3, 1.2, 3, 1.2, 3, 1.2])

test_cases = [
    (20,  '20 Hz (typical slow motion)'),
    (100, '100 Hz (typical fast motion)'),
    (500, '500 Hz (extreme high-speed)'),
]

for row, (freq, title) in enumerate(test_cases):
    truth, no_pred, pred = run_prediction(freq)
    t_ms = t_foc * 1000

    err_no = no_pred - truth
    err_pred = pred - truth

    rms_no = np.sqrt(np.mean(err_no**2))
    rms_pred = np.sqrt(np.mean(err_pred**2))
    iq_range = np.max(np.abs(truth))
    pct_no = rms_no / iq_range * 100
    pct_pred = rms_pred / iq_range * 100
    improve = rms_no / rms_pred if rms_pred > 0 else float('inf')

    # ── Waveform ──
    ax_wave = fig.add_subplot(gs[row * 2, :])

    ax_wave.plot(t_ms, truth, 'k-', linewidth=2.0, label='Ground truth (needed at apply time)', zorder=3)
    ax_wave.plot(t_ms, no_pred, 'r--', linewidth=1.2, alpha=0.8,
                 label=f'No prediction (RMS err={pct_no:.2f}%)')
    ax_wave.plot(t_ms, pred, 'b-', linewidth=1.2, alpha=0.8,
                 label=f'Linear prediction (RMS err={pct_pred:.3f}%, {improve:.0f}x better)')

    ax_wave.set_ylabel('$i_q$ ref (A)', fontsize=10)
    ax_wave.set_title(title, fontsize=12, fontweight='bold', pad=8)
    ax_wave.legend(fontsize=8, loc='upper right')
    ax_wave.grid(True, alpha=0.2)

    # Zoom inset for 100Hz and 500Hz
    if freq >= 100:
        # Find a peak region to zoom into
        peak_idx = np.argmax(truth[:len(truth)//2])
        zoom_start = max(0, peak_idx - 8)
        zoom_end = min(len(truth), peak_idx + 8)

        axins = ax_wave.inset_axes([0.02, 0.05, 0.3, 0.45])
        axins.plot(t_ms[zoom_start:zoom_end], truth[zoom_start:zoom_end], 'k-', linewidth=2)
        axins.plot(t_ms[zoom_start:zoom_end], no_pred[zoom_start:zoom_end], 'r--', linewidth=1.2)
        axins.plot(t_ms[zoom_start:zoom_end], pred[zoom_start:zoom_end], 'b-', linewidth=1.2)
        axins.set_title('Zoom', fontsize=7)
        axins.tick_params(labelsize=6)
        axins.grid(True, alpha=0.2)
        ax_wave.indicate_inset_zoom(axins, edgecolor='gray', alpha=0.5)

    # ── Error ──
    ax_err = fig.add_subplot(gs[row * 2 + 1, :])

    ax_err.fill_between(t_ms, err_no, 0, color='red', alpha=0.3, label=f'No pred error (RMS={rms_no:.2f} A)')
    ax_err.fill_between(t_ms, err_pred, 0, color='blue', alpha=0.3, label=f'Predicted error (RMS={rms_pred:.3f} A)')
    ax_err.axhline(0, color='k', linewidth=0.5)
    ax_err.set_ylabel('Error (A)', fontsize=9)
    ax_err.set_xlabel('Time (ms)', fontsize=9)
    ax_err.legend(fontsize=7, loc='upper right')
    ax_err.grid(True, alpha=0.2)

    # Set symmetric y-axis for error
    max_err = max(np.max(np.abs(err_no)), np.max(np.abs(err_pred))) * 1.2
    ax_err.set_ylim(-max_err, max_err)

fig.suptitle('Linear Extrapolation Prediction: Predicted vs Ground Truth\n'
             f'FOC delay = {DT_FOC*1e6:.1f} μs | '
             f'Impedance: {F_IMP/1000:.0f} kHz | FOC: {F_FOC/1000:.0f} kHz | '
             f'Oversampling: {RATIO}x',
             fontsize=13, fontweight='bold', y=0.995)

out_path = 'D:/robot/impedance_control/figures/fig_prediction_accuracy.png'
plt.savefig(out_path, dpi=200, bbox_inches='tight', facecolor='white')
plt.close()
print(f"Figure saved: {out_path}")

# Print summary table
print("\n" + "=" * 65)
print(f"{'Freq':>6} | {'Signal Range':>12} | {'No Pred RMS':>12} | {'Predict RMS':>12} | {'Improve':>8}")
print("-" * 65)
for freq, title in test_cases:
    truth, no_pred, pred = run_prediction(freq)
    rms_no = np.sqrt(np.mean((no_pred - truth)**2))
    rms_pred = np.sqrt(np.mean((pred - truth)**2))
    iq_range = np.max(np.abs(truth))
    improve = rms_no / rms_pred if rms_pred > 0 else 0
    print(f"{freq:>5}Hz | {iq_range:>10.2f} A | {rms_no:>10.4f} A | {rms_pred:>10.4f} A | {improve:>6.1f}x")
print("=" * 65)
