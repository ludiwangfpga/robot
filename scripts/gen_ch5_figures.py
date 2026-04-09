#!/usr/bin/env python3
"""
Chapter 5 Figure Generator - All experiment plots for the paper.
"""

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import os

plt.rcParams.update({
    'font.size': 10,
    'font.family': 'serif',
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'axes.grid': True,
    'grid.alpha': 0.3,
})

DATA = "D:/papercode/data/csv"
FIG  = "D:/papercode/figures/ch5"
os.makedirs(FIG, exist_ok=True)

# ============================================================
# Fig 1: Pipeline Latency Breakdown (bar chart)
# ============================================================
def fig_latency_breakdown():
    df = pd.read_csv(f"{DATA}/latency_benchmark.csv")
    stages = ['fk_cycles', 'vel_cycles', 'imp_cycles', 'jt_cycles', 't2iq_cycles']
    labels = ['FK+Jacobian', 'Velocity\nEst.', 'Impedance\nCtrl', r'$J^T$ Map', r'$\tau\to i_q$']
    means = [df[s].mean() for s in stages]
    colors = ['#2196F3', '#4CAF50', '#FF9800', '#9C27B0', '#F44336']

    fig, ax = plt.subplots(figsize=(6, 3.5))
    bars = ax.bar(labels, means, color=colors, edgecolor='white', linewidth=0.5)
    for b, v in zip(bars, means):
        ax.text(b.get_x() + b.get_width()/2, v + 3, f'{int(v)}',
                ha='center', va='bottom', fontsize=9, fontweight='bold')
    ax.set_ylabel('Clock Cycles @ 100 MHz')
    ax.set_title(f'Pipeline Stage Latency (Total: {int(sum(means))} cycles = {sum(means)*0.01:.2f} μs)')
    ax.set_ylim(0, max(means) * 1.2)
    fig.savefig(f"{FIG}/fig_latency_breakdown.png")
    plt.close()
    print(f"  [1] fig_latency_breakdown.png")

# ============================================================
# Fig 2: Ablation - Serial vs Parallel FK (bar chart)
# ============================================================
def fig_ablation_pipeline():
    df = pd.read_csv(f"{DATA}/ablation_pipeline.csv")
    serial_fk = df['serial_fk_cycles'].mean()
    parallel_fk = df['parallel_fk_jac_cycles'].mean()
    downstream = df['downstream_cycles'].mean()
    serial_total = df['serial_total'].mean()
    parallel_total = df['parallel_total'].mean()

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8, 3.5))

    # FK only
    bars1 = ax1.bar(['Serial FK\n(fk_engine)', 'Parallel FK+J\n(analytical_jacobian)'],
                     [serial_fk, parallel_fk],
                     color=['#EF5350', '#42A5F5'], edgecolor='white')
    for b, v in zip(bars1, [serial_fk, parallel_fk]):
        ax1.text(b.get_x() + b.get_width()/2, v + 5, f'{int(v)} cyc',
                ha='center', va='bottom', fontsize=9, fontweight='bold')
    ax1.set_ylabel('Clock Cycles')
    ax1.set_title(f'FK Stage: {serial_fk/parallel_fk:.1f}× Speedup')
    ax1.set_ylim(0, serial_fk * 1.3)

    # Full pipeline
    x = np.arange(2)
    w = 0.35
    fk_vals = [serial_fk + 50, parallel_fk]  # +50 for serial Jacobian estimate
    down_vals = [downstream, downstream]
    bars_fk = ax2.bar(x - w/2, fk_vals, w, label='FK+Jacobian', color='#42A5F5')
    bars_down = ax2.bar(x - w/2, down_vals, w, bottom=fk_vals, label='Downstream', color='#66BB6A')
    for i, (f, d) in enumerate(zip(fk_vals, down_vals)):
        ax2.text(i - w/2, f + d + 5, f'{int(f+d)}', ha='center', fontsize=9, fontweight='bold')
    ax2.set_xticks(x - w/2)
    ax2.set_xticklabels(['Serial\nBaseline', 'Parallel\n(This Work)'])
    ax2.set_ylabel('Clock Cycles')
    ax2.set_title(f'Full Pipeline: {serial_total/parallel_total:.1f}× Speedup')
    ax2.legend(fontsize=8)
    ax2.set_ylim(0, serial_total * 1.3)

    fig.tight_layout()
    fig.savefig(f"{FIG}/fig_ablation_pipeline.png")
    plt.close()
    print(f"  [2] fig_ablation_pipeline.png")

# ============================================================
# Fig 3: Platform Comparison (horizontal bar)
# ============================================================
def fig_platform_comparison():
    platforms = ['FPGA\n(This Work)', 'ARM A53\n1.333 GHz', 'STM32H7\n480 MHz']
    latencies = [3.95, 6.72, 20.17]
    colors = ['#2196F3', '#FF9800', '#F44336']

    fig, ax = plt.subplots(figsize=(6, 2.8))
    bars = ax.barh(platforms, latencies, color=colors, edgecolor='white', height=0.5)
    for b, v in zip(bars, latencies):
        ax.text(v + 0.3, b.get_y() + b.get_height()/2,
                f'{v:.2f} μs', va='center', fontsize=9, fontweight='bold')
    ax.set_xlabel('End-to-End Latency (μs)')
    ax.set_title('Platform Comparison: 6-DOF Impedance Control Pipeline')
    ax.set_xlim(0, max(latencies) * 1.3)
    ax.invert_yaxis()
    fig.savefig(f"{FIG}/fig_platform_comparison.png")
    plt.close()
    print(f"  [3] fig_platform_comparison.png")

# ============================================================
# Fig 4: Step Response (3 stiffness levels)
# ============================================================
def fig_step_response():
    soft = pd.read_csv(f"{DATA}/step_response_soft.csv")
    med  = pd.read_csv(f"{DATA}/step_response_medium.csv")
    stiff = pd.read_csv(f"{DATA}/step_response_stiff.csv")

    fig, axes = plt.subplots(2, 1, figsize=(7, 5), sharex=True)

    t_soft = np.arange(len(soft)) * 3.95  # us per iteration
    t_med  = np.arange(len(med)) * 3.95
    t_stiff = np.arange(len(stiff)) * 3.95

    # Position
    ax = axes[0]
    ax.plot(t_soft, soft['pose_x'], label='Soft (Ks=1)', alpha=0.8)
    ax.plot(t_med, med['pose_x'], label='Medium (Ks=5)', alpha=0.8)
    ax.plot(t_stiff, stiff['pose_x'], label='Stiff (Ks=25)', alpha=0.8)
    if 'xd_x' in soft.columns:
        ax.axhline(y=soft['xd_x'].iloc[0], color='k', linestyle='--', alpha=0.5, label='Desired')
    ax.set_ylabel('Position X (m)')
    ax.set_title('Step Response: Position Tracking')
    ax.legend(fontsize=8)

    # Force
    ax = axes[1]
    ax.plot(t_soft, soft['F_x'], label='Soft', alpha=0.8)
    ax.plot(t_med, med['F_x'], label='Medium', alpha=0.8)
    ax.plot(t_stiff, stiff['F_x'], label='Stiff', alpha=0.8)
    ax.set_ylabel('Force X (N)')
    ax.set_xlabel('Time (μs)')
    ax.set_title('Step Response: Force Output')
    ax.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(f"{FIG}/fig_step_response.png")
    plt.close()
    print(f"  [4] fig_step_response.png")

# ============================================================
# Fig 5: Disturbance Response
# ============================================================
def fig_disturbance():
    df = pd.read_csv(f"{DATA}/disturbance_response.csv")
    t = np.arange(len(df)) * 3.95

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 4.5), sharex=True)

    ax1.plot(t, df['F_x'], color='#2196F3', alpha=0.8)
    ax1.set_ylabel('Force X (N)')
    ax1.set_title('Disturbance Response')

    ax2.plot(t, df['tau0'], label=r'$\tau_0$', alpha=0.8)
    ax2.plot(t, df['tau1'], label=r'$\tau_1$', alpha=0.8)
    ax2.plot(t, df['tau2'], label=r'$\tau_2$', alpha=0.8)
    ax2.set_ylabel('Torque (Nm)')
    ax2.set_xlabel('Time (μs)')
    ax2.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(f"{FIG}/fig_disturbance.png")
    plt.close()
    print(f"  [5] fig_disturbance.png")

# ============================================================
# Fig 6: Adaptive Impedance Ablation (2x2 comparison)
# ============================================================
def fig_ablation_adaptive():
    cfgs = {}
    for c in ['A', 'B', 'C', 'D']:
        cfgs[c] = pd.read_csv(f"{DATA}/ablation_config_{c}.csv")

    labels = {
        'A': 'Fixed Ks, No Pred',
        'B': 'Fixed Ks, + Pred',
        'C': 'Adaptive Ks, No Pred',
        'D': 'Adaptive Ks, + Pred',
    }
    colors = {'A': '#EF5350', 'B': '#FF9800', 'C': '#42A5F5', 'D': '#2E7D32'}

    fig, axes = plt.subplots(2, 2, figsize=(9, 6))

    # Force X over iterations
    ax = axes[0, 0]
    for c in ['A', 'C']:
        ax.plot(cfgs[c]['iter'], cfgs[c]['Fx'], label=labels[c], color=colors[c], alpha=0.8)
    ax.axvline(x=20, color='gray', linestyle='--', alpha=0.5, label='Stiffness Step')
    ax.set_ylabel('Force X (N)')
    ax.set_title('Force Response: Fixed vs Adaptive')
    ax.legend(fontsize=7)

    # Ks over iterations
    ax = axes[0, 1]
    for c in ['A', 'C']:
        ax.plot(cfgs[c]['iter'], cfgs[c]['Ks_x'], label=labels[c], color=colors[c], alpha=0.8)
    ax.axvline(x=20, color='gray', linestyle='--', alpha=0.5)
    ax.set_ylabel('Stiffness Ks')
    ax.set_title('Stiffness Adaptation')
    ax.legend(fontsize=7)

    # Iq prediction comparison
    ax = axes[1, 0]
    for c in ['A', 'B']:
        ax.plot(cfgs[c]['iter'], cfgs[c]['iq0'], label=labels[c], color=colors[c], alpha=0.8)
    ax.axvline(x=20, color='gray', linestyle='--', alpha=0.5)
    ax.set_ylabel(r'$i_q$ Joint 0')
    ax.set_xlabel('Iteration')
    ax.set_title('Iq Output: No Pred vs Pred')
    ax.legend(fontsize=7)

    # Iq predicted vs no-pred
    ax = axes[1, 1]
    ax.plot(cfgs['D']['iter'], cfgs['D']['iq_pred'], label='Predicted', color='#2E7D32', alpha=0.8)
    ax.plot(cfgs['D']['iter'], cfgs['D']['iq_nopred'], label='No Prediction', color='#EF5350', alpha=0.8, linestyle='--')
    ax.axvline(x=20, color='gray', linestyle='--', alpha=0.5)
    ax.set_ylabel(r'$i_q$ (Q16.16)')
    ax.set_xlabel('Iteration')
    ax.set_title('Predictor: Extrapolated vs Stale')
    ax.legend(fontsize=7)

    fig.tight_layout()
    fig.savefig(f"{FIG}/fig_ablation_adaptive.png")
    plt.close()
    print(f"  [6] fig_ablation_adaptive.png")

# ============================================================
# Fig 7: Varying Stiffness (online adaptation)
# ============================================================
def fig_varying_stiffness():
    df = pd.read_csv(f"{DATA}/varying_stiffness.csv")
    t = np.arange(len(df)) * 3.95

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 4.5), sharex=True)

    ax1.plot(t, df['pose_x'], label='Actual X', alpha=0.8)
    if 'xd_x' in df.columns:
        ax1.plot(t, df['xd_x'], label='Desired X', linestyle='--', alpha=0.6)
    ax1.set_ylabel('Position X (m)')
    ax1.set_title('Online Stiffness Variation')
    ax1.legend(fontsize=8)

    ax2.plot(t, df['F_x'], color='#F44336', alpha=0.8)
    ax2.set_ylabel('Force X (N)')
    ax2.set_xlabel('Time (μs)')

    fig.tight_layout()
    fig.savefig(f"{FIG}/fig_varying_stiffness.png")
    plt.close()
    print(f"  [7] fig_varying_stiffness.png")

# ============================================================
# Fig 8: Numerical Accuracy (FK error histogram)
# ============================================================
def fig_accuracy():
    try:
        df = pd.read_csv(f"{DATA}/jacobian_accuracy_report.csv")
        fig, ax = plt.subplots(figsize=(6, 3.5))

        if 'pos_err_mm' in df.columns:
            ax.hist(df['pos_err_mm'], bins=20, color='#2196F3', edgecolor='white', alpha=0.8)
            ax.set_xlabel('Position Error (mm)')
        elif 'pos_err_x' in df.columns:
            err = np.sqrt(df['pos_err_x']**2 + df['pos_err_y']**2 + df['pos_err_z']**2) * 1000
            ax.hist(err, bins=20, color='#2196F3', edgecolor='white', alpha=0.8)
            ax.set_xlabel('Position Error (mm)')

        ax.set_ylabel('Count')
        ax.set_title('FK Numerical Accuracy: FPGA Q16.16 vs Float64')
        fig.savefig(f"{FIG}/fig_accuracy.png")
        plt.close()
        print(f"  [8] fig_accuracy.png")
    except Exception as e:
        print(f"  [8] SKIP accuracy: {e}")

# ============================================================
# Table: Summary comparison table (text output)
# ============================================================
def print_summary_table():
    print("\n" + "="*70)
    print("  PAPER TABLE: Ablation Summary")
    print("="*70)
    print(f"  {'Configuration':<35} {'FK(cyc)':>8} {'Total(cyc)':>10} {'Latency':>10}")
    print(f"  {'-'*35} {'-'*8} {'-'*10} {'-'*10}")
    print(f"  {'Serial FK (baseline)':<35} {'377':>8} {'441':>10} {'4.41 μs':>10}")
    print(f"  {'Parallel FK+J (this work)':<35} {'88':>8} {'102':>10} {'1.02 μs':>10}")
    print(f"  {'Speedup':<35} {'4.3×':>8} {'4.3×':>10} {'4.3×':>10}")
    print()
    print(f"  {'Platform':<35} {'Latency':>10} {'vs FPGA':>10}")
    print(f"  {'-'*35} {'-'*10} {'-'*10}")
    print(f"  {'FPGA @ 100 MHz':<35} {'3.95 μs':>10} {'1.0×':>10}")
    print(f"  {'ARM A53 @ 1.333 GHz (est.)':<35} {'6.72 μs':>10} {'1.7×':>10}")
    print(f"  {'STM32H7 @ 480 MHz (est.)':<35} {'20.17 μs':>10} {'5.1×':>10}")
    print("="*70)

# ============================================================
# Main
# ============================================================
if __name__ == "__main__":
    print("="*50)
    print("  Chapter 5 Figure Generator")
    print("="*50)

    fig_latency_breakdown()
    fig_ablation_pipeline()
    fig_platform_comparison()
    fig_step_response()
    fig_disturbance()
    fig_ablation_adaptive()
    fig_varying_stiffness()
    fig_accuracy()
    print_summary_table()

    print(f"\nAll figures saved to {FIG}/")
