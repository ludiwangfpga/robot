#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Linear Extrapolation Prediction Test v2 - Corrected Model

The REAL delay problem:
  - iq_ref is always fresh (updated every 3.95 us)
  - BUT: FOC reads iq_ref at t=0, applies torque at t=55.6 us (one FOC cycle later)
  - So torque applied at t+55.6us is based on iq_ref computed at t
  - Prediction goal: at time t, predict iq_ref(t + 55.6us) and feed that to FOC
    so the applied torque matches what's actually needed at t+55.6us

Three strategies:
  1. No prediction: FOC uses iq_ref(t), applies at t+55.6us -> stale by 55.6us
  2. Linear prediction: predict iq_ref(t+55.6us) using velocity of iq_ref
  3. Oversampled prediction: same but use averaged velocity from 14 samples
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ── System parameters ──
F_IMPEDANCE = 253200.0
F_FOC = 18000.0
DT_IMP = 1.0 / F_IMPEDANCE
DT_FOC = 1.0 / F_FOC
OVERSAMPLE_RATIO = int(round(F_IMPEDANCE / F_FOC))  # 14

KS = 500.0
DS = 30.0
KT_INV = 20.0

T_SIM = 0.05
N_IMP = int(T_SIM / DT_IMP)
N_FOC = int(T_SIM / DT_FOC)

t_imp = np.arange(N_IMP) * DT_IMP
t_foc = np.arange(N_FOC) * DT_FOC


def compute_iq_from_trajectory(t, freq, amp=0.005):
    """Compute iq_ref for a sinusoidal trajectory at time t"""
    omega = 2 * np.pi * freq
    xd = amp * np.sin(omega * t)
    vd = amp * omega * np.cos(omega * t)
    x = amp * 0.95 * np.sin(omega * t - 0.05)
    v = amp * 0.95 * omega * np.cos(omega * t - 0.05)
    F = KS * (xd - x) + DS * (vd - v)
    return F * KT_INV


def test_sinusoidal(freq_hz):
    """Test with sinusoidal desired trajectory"""
    # Compute iq_ref at impedance rate (this is the "true" signal)
    iq_imp = np.array([compute_iq_from_trajectory(t, freq_hz) for t in t_imp])

    # What FOC actually needs: iq_ref at the moment torque is APPLIED
    # Torque is applied one FOC cycle AFTER reading
    # So at FOC sample k, torque applies at t_foc[k] + DT_FOC
    # Ideal: iq_ref should match what's needed at t_foc[k] + DT_FOC
    iq_ideal = np.array([compute_iq_from_trajectory(t + DT_FOC, freq_hz)
                         for t in t_foc])

    # Strategy 1: No prediction - FOC reads latest iq_ref(t)
    # Applied at t+DT_FOC, but computed for time t -> one cycle stale
    iq_no_pred = np.array([compute_iq_from_trajectory(t, freq_hz)
                           for t in t_foc])

    # Strategy 2: Linear prediction using 2 recent impedance samples
    iq_lin2 = np.zeros(N_FOC)
    for i in range(N_FOC):
        imp_idx = min(int(t_foc[i] / DT_IMP), N_IMP - 1)
        if imp_idx >= 1:
            diq = (iq_imp[imp_idx] - iq_imp[imp_idx - 1]) / DT_IMP
            iq_lin2[i] = iq_imp[imp_idx] + diq * DT_FOC  # predict one FOC cycle ahead
        else:
            iq_lin2[i] = iq_imp[imp_idx]

    # Strategy 3: Oversampled linear prediction (average slope from N samples)
    iq_lin_os = np.zeros(N_FOC)
    for i in range(N_FOC):
        imp_idx = min(int(t_foc[i] / DT_IMP), N_IMP - 1)
        n_use = min(OVERSAMPLE_RATIO, imp_idx)
        if n_use >= 2:
            # Least-squares slope over last n_use samples
            iq_window = iq_imp[imp_idx - n_use + 1: imp_idx + 1]
            t_window = np.arange(n_use) * DT_IMP
            # slope = (n*sum(t*iq) - sum(t)*sum(iq)) / (n*sum(t^2) - sum(t)^2)
            n = len(t_window)
            st = np.sum(t_window)
            st2 = np.sum(t_window**2)
            siq = np.sum(iq_window)
            stiq = np.sum(t_window * iq_window)
            slope = (n * stiq - st * siq) / (n * st2 - st**2)
            intercept = (siq - slope * st) / n
            # Predict at DT_FOC ahead of the last sample
            t_predict = t_window[-1] + DT_FOC
            iq_lin_os[i] = intercept + slope * t_predict
        else:
            iq_lin_os[i] = iq_imp[imp_idx]

    # Errors
    err_no = np.abs(iq_no_pred - iq_ideal)
    err_lin2 = np.abs(iq_lin2 - iq_ideal)
    err_os = np.abs(iq_lin_os - iq_ideal)
    iq_range = np.max(np.abs(iq_ideal))

    return {
        'freq': freq_hz,
        'iq_ideal': iq_ideal,
        'iq_no_pred': iq_no_pred,
        'iq_lin2': iq_lin2,
        'iq_lin_os': iq_lin_os,
        'iq_range': iq_range if iq_range > 0 else 1.0,
        'err_no_max': np.max(err_no), 'err_no_rms': np.sqrt(np.mean(err_no**2)),
        'err_lin2_max': np.max(err_lin2), 'err_lin2_rms': np.sqrt(np.mean(err_lin2**2)),
        'err_os_max': np.max(err_os), 'err_os_rms': np.sqrt(np.mean(err_os**2)),
    }


def test_step():
    """Step response test"""
    step_time = T_SIM * 0.25
    tau = 0.002  # 2ms time constant

    def iq_step(t):
        if t < step_time:
            return 0.0
        return 100.0 * (1.0 - np.exp(-(t - step_time) / tau))

    iq_imp = np.array([iq_step(t) for t in t_imp])
    iq_ideal = np.array([iq_step(t + DT_FOC) for t in t_foc])
    iq_no_pred = np.array([iq_step(t) for t in t_foc])

    iq_lin = np.zeros(N_FOC)
    for i in range(N_FOC):
        imp_idx = min(int(t_foc[i] / DT_IMP), N_IMP - 1)
        if imp_idx >= 1:
            diq = (iq_imp[imp_idx] - iq_imp[imp_idx - 1]) / DT_IMP
            iq_lin[i] = iq_imp[imp_idx] + diq * DT_FOC
        else:
            iq_lin[i] = iq_imp[imp_idx]

    err_no = np.abs(iq_no_pred - iq_ideal)
    err_lin = np.abs(iq_lin - iq_ideal)

    return {
        'iq_ideal': iq_ideal, 'iq_no_pred': iq_no_pred, 'iq_lin': iq_lin,
        'err_no_max': np.max(err_no), 'err_no_rms': np.sqrt(np.mean(err_no**2)),
        'err_lin_max': np.max(err_lin), 'err_lin_rms': np.sqrt(np.mean(err_lin**2)),
    }


def test_collision():
    """Collision test - sudden force spike"""
    coll_time = T_SIM * 0.33

    def iq_coll(t):
        base = 50.0 * np.sin(2 * np.pi * 20 * t)
        if t < coll_time:
            return base
        return base + 200.0 * np.exp(-(t - coll_time) / 0.001)

    iq_imp = np.array([iq_coll(t) for t in t_imp])
    iq_ideal = np.array([iq_coll(t + DT_FOC) for t in t_foc])
    iq_no_pred = np.array([iq_coll(t) for t in t_foc])

    iq_lin = np.zeros(N_FOC)
    for i in range(N_FOC):
        imp_idx = min(int(t_foc[i] / DT_IMP), N_IMP - 1)
        if imp_idx >= 1:
            diq = (iq_imp[imp_idx] - iq_imp[imp_idx - 1]) / DT_IMP
            iq_lin[i] = iq_imp[imp_idx] + diq * DT_FOC
        else:
            iq_lin[i] = iq_imp[imp_idx]

    err_no = np.abs(iq_no_pred - iq_ideal)
    err_lin = np.abs(iq_lin - iq_ideal)

    return {
        'iq_ideal': iq_ideal, 'iq_no_pred': iq_no_pred, 'iq_lin': iq_lin,
        'err_no_max': np.max(err_no), 'err_no_rms': np.sqrt(np.mean(err_no**2)),
        'err_lin_max': np.max(err_lin), 'err_lin_rms': np.sqrt(np.mean(err_lin**2)),
        'coll_time': coll_time,
    }


# ════════════════════════════════════════
# Run tests
# ════════════════════════════════════════
print("=" * 80)
print("Linear Extrapolation Prediction Test v2 (Corrected Delay Model)")
print(f"Problem: FOC reads iq_ref at t, applies torque at t + {DT_FOC*1e6:.1f} us")
print(f"Goal: predict iq_ref(t + {DT_FOC*1e6:.1f} us) to compensate FOC processing delay")
print(f"Impedance: {F_IMPEDANCE/1000:.1f} kHz | FOC: {F_FOC/1000:.1f} kHz | Oversample: {OVERSAMPLE_RATIO}x")
print("=" * 80)

# Sinusoidal tests
print("\n--- Sinusoidal Motion: RMS Error (% of signal range) ---")
print(f"{'Freq(Hz)':>8} | {'No Pred':>10} | {'Lin 2pt':>10} | {'Lin OvSmp':>10} | {'Improve(2pt)':>12} | {'Improve(OS)':>12}")
print("-" * 80)

freqs = [1, 5, 10, 20, 50, 100, 200, 300, 500]
sin_results = []
for f in freqs:
    r = test_sinusoidal(f)
    sin_results.append(r)
    pct_no = r['err_no_rms'] / r['iq_range'] * 100
    pct_lin = r['err_lin2_rms'] / r['iq_range'] * 100
    pct_os = r['err_os_rms'] / r['iq_range'] * 100
    imp_lin = pct_no / pct_lin if pct_lin > 0 else float('inf')
    imp_os = pct_no / pct_os if pct_os > 0 else float('inf')
    print(f"{f:>8} | {pct_no:>9.4f}% | {pct_lin:>9.4f}% | {pct_os:>9.4f}% | "
          f"{imp_lin:>10.1f}x | {imp_os:>10.1f}x")

# Step test
print("\n--- Step Response ---")
step_r = test_step()
print(f"  No prediction:  max={step_r['err_no_max']:.4f}, rms={step_r['err_no_rms']:.4f}")
print(f"  Linear predict: max={step_r['err_lin_max']:.4f}, rms={step_r['err_lin_rms']:.4f}")
imp_step = step_r['err_no_rms'] / step_r['err_lin_rms'] if step_r['err_lin_rms'] > 0 else float('inf')
print(f"  Improvement: {imp_step:.1f}x")

# Collision test
print("\n--- Collision Event ---")
coll_r = test_collision()
print(f"  No prediction:  max={coll_r['err_no_max']:.2f}, rms={coll_r['err_no_rms']:.4f}")
print(f"  Linear predict: max={coll_r['err_lin_max']:.2f}, rms={coll_r['err_lin_rms']:.4f}")
imp_coll = coll_r['err_no_rms'] / coll_r['err_lin_rms'] if coll_r['err_lin_rms'] > 0 else float('inf')
print(f"  RMS Improvement: {imp_coll:.1f}x")
if coll_r['err_lin_max'] > coll_r['err_no_max']:
    print(f"  Peak overshoot: {coll_r['err_lin_max']/coll_r['err_no_max']:.1f}x (transient, ~{DT_FOC*1e6:.0f} us)")

# ════════════════════════════════════════
# Plot
# ════════════════════════════════════════
plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'DejaVu Serif'],
    'font.size': 9,
})

fig, axes = plt.subplots(2, 2, figsize=(12, 8))

# (a) Error vs frequency
ax = axes[0, 0]
freqs_arr = [r['freq'] for r in sin_results]
pct_no = [r['err_no_rms'] / r['iq_range'] * 100 for r in sin_results]
pct_lin = [r['err_lin2_rms'] / r['iq_range'] * 100 for r in sin_results]
pct_os = [r['err_os_rms'] / r['iq_range'] * 100 for r in sin_results]
ax.semilogy(freqs_arr, pct_no, 'r-o', label='No prediction', markersize=5, linewidth=1.5)
ax.semilogy(freqs_arr, pct_lin, 'b-s', label='Linear (2-point)', markersize=5, linewidth=1.5)
ax.semilogy(freqs_arr, pct_os, 'g-^', label='Linear (least-squares, 14pt)', markersize=5, linewidth=1.5)
ax.set_xlabel('Motion Frequency (Hz)')
ax.set_ylabel('Relative RMS Error (%)')
ax.set_title('(a) Prediction Error vs Motion Frequency')
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

# (b) 100 Hz waveform
ax = axes[0, 1]
r100 = sin_results[5]  # 100 Hz
n_show = min(200, len(r100['iq_ideal']))
t_show = t_foc[:n_show] * 1000
ax.plot(t_show, r100['iq_ideal'][:n_show], 'k-', label='Ideal (needed at apply time)', linewidth=1.5)
ax.plot(t_show, r100['iq_no_pred'][:n_show], 'r--', label='No prediction (1 cycle late)', linewidth=1.0)
ax.plot(t_show, r100['iq_lin_os'][:n_show], 'g-', label='Linear prediction', linewidth=1.0)
ax.set_xlabel('Time (ms)')
ax.set_ylabel('$i_q$ ref (A)')
ax.set_title('(b) 100 Hz Sinusoidal: Waveform')
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)

# (c) Step response
ax = axes[1, 0]
t_step_ms = t_foc * 1000
ax.plot(t_step_ms, step_r['iq_ideal'], 'k-', label='Ideal', linewidth=1.5)
ax.plot(t_step_ms, step_r['iq_no_pred'], 'r--', label='No prediction', linewidth=1.0)
ax.plot(t_step_ms, step_r['iq_lin'], 'b-', label='Linear prediction', linewidth=1.0)
ax.set_xlabel('Time (ms)')
ax.set_ylabel('$i_q$ ref (A)')
ax.set_title('(c) Step Response')
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

# (d) Collision
ax = axes[1, 1]
ax.plot(t_foc * 1000, coll_r['iq_ideal'], 'k-', label='Ideal', linewidth=1.5)
ax.plot(t_foc * 1000, coll_r['iq_no_pred'], 'r--', label='No prediction', linewidth=1.0)
ax.plot(t_foc * 1000, coll_r['iq_lin'], 'b-', label='Linear prediction', linewidth=1.0)
ax.axvline(coll_r['coll_time'] * 1000, color='orange', ls=':', lw=1.5, label='Collision')
ax.set_xlabel('Time (ms)')
ax.set_ylabel('$i_q$ ref (A)')
ax.set_title('(d) Collision Event (Worst Case)')
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

fig.suptitle('FOC Delay Compensation via Linear Extrapolation\n'
             f'Impedance: {F_IMPEDANCE/1000:.0f} kHz | FOC: {F_FOC/1000:.0f} kHz | '
             f'FOC delay: {DT_FOC*1e6:.1f} $\\mu$s | Oversampling: {OVERSAMPLE_RATIO}x',
             fontsize=11, fontweight='bold')
plt.tight_layout()

out_path = 'D:/robot/impedance_control/figures/fig_prediction_test_v2.png'
plt.savefig(out_path, dpi=200, bbox_inches='tight', facecolor='white')
plt.close()
print(f"\nFigure saved: {out_path}")

# Final summary
print("\n" + "=" * 80)
print("SUMMARY")
print("=" * 80)
r100 = sin_results[5]
r500 = sin_results[-1]
print(f"The real delay: FOC processing takes {DT_FOC*1e6:.1f} us from read to apply")
print(f"")
print(f"At 100 Hz (typical robot arm motion):")
print(f"  No prediction error: {r100['err_no_rms']/r100['iq_range']*100:.4f}%")
print(f"  Linear prediction:   {r100['err_os_rms']/r100['iq_range']*100:.4f}%")
imp100 = r100['err_no_rms'] / r100['err_os_rms'] if r100['err_os_rms'] > 0 else 0
print(f"  Improvement: {imp100:.1f}x")
print(f"")
print(f"At 500 Hz (high-speed motion):")
print(f"  No prediction error: {r500['err_no_rms']/r500['iq_range']*100:.4f}%")
print(f"  Linear prediction:   {r500['err_os_rms']/r500['iq_range']*100:.4f}%")
imp500 = r500['err_no_rms'] / r500['err_os_rms'] if r500['err_os_rms'] > 0 else 0
print(f"  Improvement: {imp500:.1f}x")
