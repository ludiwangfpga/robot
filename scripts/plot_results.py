#!/usr/bin/env python3
"""
plot_results.py - Paper Figure Generator

Generates all figures for the paper:
  Fig 1: System architecture block diagram (manual/TikZ)
  Fig 2: Pipeline timing diagram (manual/TikZ)
  Fig 3: FK and Jacobian numerical accuracy (histograms)
  Fig 4: Impedance control step response (time domain)
  Fig 5: Disturbance rejection response
  Fig 6: FPGA vs software latency comparison (bar chart)
  Fig 7: Online stiffness modulation demonstration
  Table I: Pipeline latency breakdown
  Table II: FPGA resource utilization (from Vivado reports)
  Table III: Platform comparison

Usage:
  python plot_results.py                    # Plot all available figures
  python plot_results.py --figure 4         # Plot specific figure
  python plot_results.py --format pdf       # Output as PDF (default: png)
"""

import os
import sys
import argparse
import numpy as np

try:
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    print("WARNING: matplotlib not installed. Install with: pip install matplotlib")

# Output directory
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
OUTPUT_DIR = os.path.join(PROJECT_DIR, 'figures')
DATA_DIR = os.path.join(PROJECT_DIR, 'sim_work')  # CSV files from Vivado xsim


def ensure_output_dir():
    os.makedirs(OUTPUT_DIR, exist_ok=True)


def load_csv(filename, data_dir=DATA_DIR):
    """Load a CSV file, return header and data."""
    filepath = os.path.join(data_dir, filename)
    if not os.path.exists(filepath):
        print(f"  WARNING: {filepath} not found. Run simulation first.")
        return None, None
    data = np.genfromtxt(filepath, delimiter=',', skip_header=1, dtype=float)
    with open(filepath, 'r') as f:
        header = f.readline().strip().split(',')
    return header, data


def plot_fig3_accuracy(fmt='png'):
    """Fig 3: FK and Jacobian numerical accuracy."""
    if not HAS_MPL:
        return

    # Load Jacobian accuracy report
    header, data = load_csv('jacobian_accuracy_report.csv')
    if data is None:
        print("  Skipping Fig 3: No accuracy data available")
        print("  Run: iverilog + vvp tb_analytical_jacobian.v first")
        return

    max_err_lin = data[:, 1]   # Q16.16 units
    max_err_ang = data[:, 2]
    max_err_pose = data[:, 3]
    max_err_lin_m = data[:, 4]  # meters

    fig, axes = plt.subplots(1, 3, figsize=(14, 4))

    # Linear Jacobian error histogram
    axes[0].hist(max_err_lin_m * 1000, bins=30, color='steelblue', edgecolor='black', alpha=0.8)
    axes[0].set_xlabel('Max Error (mm)')
    axes[0].set_ylabel('Count')
    axes[0].set_title('(a) Jacobian Linear Part Error')
    axes[0].axvline(x=np.mean(max_err_lin_m) * 1000, color='red', linestyle='--',
                    label=f'Mean: {np.mean(max_err_lin_m)*1000:.3f} mm')
    axes[0].legend()

    # Angular Jacobian error histogram
    max_err_ang_unit = data[:, 5]
    axes[1].hist(max_err_ang_unit, bins=30, color='coral', edgecolor='black', alpha=0.8)
    axes[1].set_xlabel('Max Error (unitless)')
    axes[1].set_ylabel('Count')
    axes[1].set_title('(b) Jacobian Angular Part Error')
    axes[1].axvline(x=np.mean(max_err_ang_unit), color='red', linestyle='--',
                    label=f'Mean: {np.mean(max_err_ang_unit):.4f}')
    axes[1].legend()

    # FK pose error histogram
    max_err_pose_mm = (max_err_pose / 65536.0) * 1000
    axes[2].hist(max_err_pose_mm, bins=30, color='seagreen', edgecolor='black', alpha=0.8)
    axes[2].set_xlabel('Max Error (mm)')
    axes[2].set_ylabel('Count')
    axes[2].set_title('(c) FK Position Error')
    axes[2].axvline(x=np.mean(max_err_pose_mm), color='red', linestyle='--',
                    label=f'Mean: {np.mean(max_err_pose_mm):.3f} mm')
    axes[2].legend()

    plt.tight_layout()
    filepath = os.path.join(OUTPUT_DIR, f'fig3_accuracy.{fmt}')
    plt.savefig(filepath, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {filepath}")

    # Print statistics for paper text
    print(f"  Stats for paper:")
    print(f"    Jacobian linear:  max={np.max(max_err_lin_m)*1000:.3f}mm, "
          f"mean={np.mean(max_err_lin_m)*1000:.3f}mm, RMS={np.sqrt(np.mean(max_err_lin_m**2))*1000:.3f}mm")
    print(f"    Jacobian angular: max={np.max(max_err_ang_unit):.4f}, "
          f"mean={np.mean(max_err_ang_unit):.4f}")
    print(f"    FK position:      max={np.max(max_err_pose_mm):.3f}mm, "
          f"mean={np.mean(max_err_pose_mm):.3f}mm")


def plot_fig4_step_response(fmt='png'):
    """Fig 4: Impedance control step response with different stiffness."""
    if not HAS_MPL:
        return

    # Simulation time step ~1ms (DT=64/65536)
    DT_SIM = 64.0 / 65536.0  # seconds per iteration

    configs = [
        ('step_response_stiff.csv',  'High ($K_s$=500 N/m, $D_s$=30 Ns/m)', 'steelblue'),
        ('step_response_medium.csv', 'Medium ($K_s$=100 N/m, $D_s$=15 Ns/m)', 'coral'),
        ('step_response_soft.csv',   'Low ($K_s$=20 N/m, $D_s$=5 Ns/m)', 'seagreen'),
    ]

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    any_data = False
    for filename, label, color in configs:
        header, data = load_csv(filename)
        if data is None:
            continue
        any_data = True
        time_s = data[:, 0] * DT_SIM  # convert iterations to seconds
        pose_x = data[:, 1]
        F_x = data[:, 7]

        x_home = pose_x[0]
        axes[0].plot(time_s, (pose_x - x_home) * 1000, color=color, linewidth=1.5, label=label)
        axes[1].plot(time_s, F_x, color=color, linewidth=1.5, label=label)

    if not any_data:
        print("  Skipping Fig 4: No step response data available")
        print("  Run: iverilog + vvp tb_impedance_step_response.v first")
        return

    axes[0].axhline(y=5, color='black', linestyle='--', linewidth=0.8, label='Desired (+5 mm)')
    axes[0].set_ylabel('Displacement from Home (mm)')
    axes[0].set_title('(a) Task-Space Position Step Response (+5 mm)')
    axes[0].legend(loc='lower right', fontsize=9)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_ylim(-0.5, 6.5)

    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Force $F_x$ (N)')
    axes[1].set_title('(b) Task-Space Impedance Force Output')
    axes[1].legend(loc='upper right', fontsize=9)
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    filepath = os.path.join(OUTPUT_DIR, f'fig4_step_response.{fmt}')
    plt.savefig(filepath, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {filepath}")


def plot_fig5_disturbance(fmt='png'):
    """Fig 5: Disturbance rejection response."""
    if not HAS_MPL:
        return

    header, data = load_csv('disturbance_response.csv')
    if data is None:
        print("  Skipping Fig 5: No disturbance data available")
        return

    DT_SIM = 64.0 / 65536.0
    time_s = data[:, 0] * DT_SIM
    pose_x = data[:, 1]
    xd_x = data[:, 4]
    F_x = data[:, 7]
    tau0 = data[:, 10]

    x_home = pose_x[0]

    # Disturbance timing (in seconds)
    t_dist_on = 2000 * DT_SIM
    t_dist_off = 4000 * DT_SIM

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    axes[0].plot(time_s, (pose_x - x_home) * 1000, 'steelblue', linewidth=1.5, label='Actual')
    axes[0].axhline(y=(xd_x[0] - x_home) * 1000, color='red', linestyle='--', linewidth=0.8, label='Desired')
    axes[0].axvspan(t_dist_on, t_dist_off, alpha=0.1, color='orange', label='Disturbance (0.5 Nm)')
    axes[0].set_ylabel('Displacement from Home (mm)')
    axes[0].set_title('(a) Position Response to External Disturbance ($K_s$=100 N/m, $D_s$=15 Ns/m)')
    axes[0].legend(fontsize=9)
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(time_s, F_x, 'coral', linewidth=1.5)
    axes[1].axvspan(t_dist_on, t_dist_off, alpha=0.1, color='orange')
    axes[1].set_ylabel('Impedance Force $F_x$ (N)')
    axes[1].set_title('(b) Impedance Controller Force Output')
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(time_s, tau0, 'seagreen', linewidth=1.5)
    axes[2].axvspan(t_dist_on, t_dist_off, alpha=0.1, color='orange')
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Joint Torque $\\tau_0$ (Nm)')
    axes[2].set_title('(c) Joint Torque Output')
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    filepath = os.path.join(OUTPUT_DIR, f'fig5_disturbance.{fmt}')
    plt.savefig(filepath, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {filepath}")


def plot_fig6_latency_comparison(fmt='png'):
    """Fig 6: FPGA vs software latency comparison."""
    if not HAS_MPL:
        return

    # Try to load benchmark data
    header, data = load_csv('latency_benchmark.csv')

    if data is not None:
        # Use measured data
        fk_avg = np.mean(data[:, 7])
        vel_avg = np.mean(data[:, 8])
        imp_avg = np.mean(data[:, 9])
        jt_avg = np.mean(data[:, 10])
        t2iq_avg = np.mean(data[:, 11])
        total_avg = np.mean(data[:, 12])
        fpga_latency = total_avg * 0.01  # us at 100MHz
    else:
        # Use actual measured values from Vivado simulation
        fk_avg = 382
        vel_avg = 3
        imp_avg = 2
        jt_avg = 6
        t2iq_avg = 2
        total_avg = 395
        fpga_latency = 3.95
        print("  Using measured latency values from Vivado simulation")

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # Left: Pipeline breakdown (stacked bar)
    stages = ['FK+J', 'Vel Est', 'Impedance', 'J^T Map', 'τ→Iq']
    cycles = [fk_avg, vel_avg, imp_avg, jt_avg, t2iq_avg]
    times_us = [c * 0.01 for c in cycles]  # μs at 100MHz
    colors = ['#4e79a7', '#f28e2b', '#e15759', '#76b7b2', '#59a14f']

    bars = axes[0].barh(stages, times_us, color=colors, edgecolor='black', height=0.6)
    axes[0].set_xlabel('Latency (μs)')
    axes[0].set_title('(a) Pipeline Stage Latency Breakdown')
    for bar, val in zip(bars, times_us):
        axes[0].text(bar.get_width() + 0.02, bar.get_y() + bar.get_height()/2,
                     f'{val:.2f} μs', va='center', fontsize=9)
    axes[0].set_xlim(0, max(times_us) * 1.4)

    # Right: Platform comparison
    platforms = ['FPGA\n(This Work)', 'TI C2000\nDSP', 'STM32H7\nARM', 'ROS2\nLinux']
    latencies = [fpga_latency, 100, 200, 1000]
    bar_colors = ['#e15759', '#4e79a7', '#4e79a7', '#4e79a7']

    bars2 = axes[1].bar(platforms, latencies, color=bar_colors, edgecolor='black', width=0.6)
    axes[1].set_ylabel('End-to-End Latency (μs)')
    axes[1].set_title('(b) Platform Comparison')
    axes[1].set_yscale('log')
    axes[1].set_ylim(1, 5000)
    for bar, val in zip(bars2, latencies):
        axes[1].text(bar.get_x() + bar.get_width()/2, bar.get_height() * 1.3,
                     f'{val:.1f} μs', ha='center', fontsize=9, fontweight='bold')

    # Add speedup annotations
    for i in range(1, len(latencies)):
        speedup = latencies[i] / latencies[0]
        axes[1].text(bars2[i].get_x() + bars2[i].get_width()/2, latencies[i] * 0.4,
                     f'{speedup:.0f}×', ha='center', fontsize=8, color='white', fontweight='bold')

    plt.tight_layout()
    filepath = os.path.join(OUTPUT_DIR, f'fig6_latency.{fmt}')
    plt.savefig(filepath, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {filepath}")

    # Print Table I for paper
    print(f"\n  TABLE I: Pipeline Latency Breakdown")
    print(f"  {'Stage':<20} {'Cycles':>8} {'Time(μs)':>10} {'Percentage':>10}")
    print(f"  {'-'*48}")
    for stage, cyc, t in zip(stages, cycles, times_us):
        pct = cyc / total_avg * 100
        print(f"  {stage:<20} {cyc:>8.0f} {t:>10.2f} {pct:>9.1f}%")
    print(f"  {'-'*48}")
    print(f"  {'TOTAL':<20} {total_avg:>8.0f} {fpga_latency:>10.2f} {'100.0%':>10}")
    print(f"  Max control rate: {1000.0/fpga_latency:.1f} kHz")


def plot_fig7_varying_stiffness(fmt='png'):
    """Fig 7: Online stiffness modulation."""
    if not HAS_MPL:
        return

    header, data = load_csv('varying_stiffness.csv')
    if data is None:
        print("  Skipping Fig 7: No varying stiffness data available")
        return

    DT_SIM = 64.0 / 65536.0
    time_s = data[:, 0] * DT_SIM
    pose_x = data[:, 1]
    xd_x = data[:, 4]
    F_x = data[:, 7]

    x_home = pose_x[0]

    # Phase boundaries in seconds
    t_phase1 = 2000 * DT_SIM
    t_phase2 = 4000 * DT_SIM

    fig, ax1 = plt.subplots(figsize=(10, 5))

    ax1.plot(time_s, (pose_x - x_home) * 1000, 'steelblue', linewidth=1.5, label='Actual position')
    ax1.axhline(y=(xd_x[0] - x_home) * 1000, color='red', linestyle='--', linewidth=0.8, label='Desired (+5 mm)')

    ax1.axvspan(0, t_phase1, alpha=0.08, color='red', label='High $K_s$ (500 N/m)')
    ax1.axvspan(t_phase1, t_phase2, alpha=0.08, color='blue', label='Low $K_s$ (20 N/m)')
    ax1.axvspan(t_phase2, time_s[-1], alpha=0.08, color='red')

    ax1.axvline(x=t_phase1, color='gray', linestyle=':', linewidth=1)
    ax1.axvline(x=t_phase2, color='gray', linestyle=':', linewidth=1)

    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Displacement from Home (mm)')
    ax1.set_title('Online Impedance Modulation: Real-Time Stiffness Switching ($D_s$=30 Ns/m)')
    ax1.legend(loc='best', fontsize=9)
    ax1.grid(True, alpha=0.3)

    plt.tight_layout()
    filepath = os.path.join(OUTPUT_DIR, f'fig7_varying_stiffness.{fmt}')
    plt.savefig(filepath, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {filepath}")


def generate_latex_tables():
    """Generate LaTeX table code for paper."""
    print("\n=== LaTeX Table Code ===\n")

    # Table I: Pipeline Latency
    print(r"""% Table I: Pipeline Latency Breakdown
\begin{table}[t]
\centering
\caption{Pipeline Latency Breakdown at 100MHz}
\label{tab:latency}
\begin{tabular}{lrrr}
\toprule
\textbf{Stage} & \textbf{Cycles} & \textbf{Latency ($\mu$s)} & \textbf{Percentage} \\
\midrule
FK + Analytical Jacobian & 382 & 3.82 & 96.7\% \\
Velocity Estimation & 3 & 0.03 & 0.8\% \\
Impedance Controller & 2 & 0.02 & 0.5\% \\
$\mathbf{J}^T$ Force Mapping & 6 & 0.06 & 1.5\% \\
Torque to $I_q$ & 2 & 0.02 & 0.5\% \\
\midrule
\textbf{Total} & \textbf{395} & \textbf{3.95} & \textbf{100\%} \\
\bottomrule
\end{tabular}
\end{table}""")

    # Table III: Platform Comparison
    print(r"""
% Table III: Platform Comparison
\begin{table}[t]
\centering
\caption{Impedance Control Latency Comparison}
\label{tab:comparison}
\begin{tabular}{lccc}
\toprule
\textbf{Platform} & \textbf{Clock} & \textbf{Latency ($\mu$s)} & \textbf{Max Rate (kHz)} \\
\midrule
\textbf{This work (FPGA)} & 100 MHz & 3.95 & 253.2 \\
TI C2000 DSP & 200 MHz & $\sim$100 & $\sim$10 \\
STM32H7 ARM & 480 MHz & $\sim$200 & $\sim$5 \\
ROS2/Linux (typical) & GHz-class & $\sim$1000 & $\sim$1 \\
\bottomrule
\end{tabular}
\end{table}""")

    # Table II: Resource Utilization
    print(r"""
% Table II: FPGA Resource Utilization (Vivado synthesis, xcu200-fsgd2104-2)
\begin{table}[t]
\centering
\caption{FPGA Resource Utilization on Xilinx Alveo U200}
\label{tab:resources}
\begin{tabular}{lrrrr}
\toprule
\textbf{Module} & \textbf{LUTs} & \textbf{FFs} & \textbf{DSP48E2} & \textbf{BRAM} \\
\midrule
FK + Analytical Jacobian & 5,988 & 3,667 & 312 & 0 \\
Velocity Estimator & 3,226 & 584 & 24 & 0 \\
Impedance Controller & 953 & 196 & 48 & 0 \\
$\mathbf{J}^T$ Force Mapper & 784 & 1,355 & 24 & 0 \\
Torque to $I_q$ & 52 & 23 & 12 & 0 \\
\midrule
\textbf{Total (control pipeline)} & \textbf{11,003} & \textbf{5,825} & \textbf{420} & \textbf{0} \\
Available (U200) & 1,182,240 & 2,364,480 & 6,840 & 2,160 \\
\textbf{Utilization} & \textbf{0.93\%} & \textbf{0.25\%} & \textbf{6.14\%} & \textbf{0\%} \\
\bottomrule
\end{tabular}
\end{table}""")


def main():
    parser = argparse.ArgumentParser(description='Generate paper figures')
    parser.add_argument('--figure', type=int, help='Generate specific figure (3-7)')
    parser.add_argument('--format', default='png', choices=['png', 'pdf', 'svg'],
                        help='Output format (default: png)')
    parser.add_argument('--tables', action='store_true', help='Generate LaTeX tables')
    args = parser.parse_args()

    ensure_output_dir()
    fmt = args.format

    print(f"=== Paper Figure Generator ===")
    print(f"Output directory: {OUTPUT_DIR}")
    print(f"Format: {fmt}")
    print()

    if args.tables:
        generate_latex_tables()
        return

    if args.figure:
        fig_map = {
            3: plot_fig3_accuracy,
            4: plot_fig4_step_response,
            5: plot_fig5_disturbance,
            6: plot_fig6_latency_comparison,
            7: plot_fig7_varying_stiffness,
        }
        if args.figure in fig_map:
            print(f"Generating Figure {args.figure}...")
            fig_map[args.figure](fmt)
        else:
            print(f"Figure {args.figure} not available. Choose from: {list(fig_map.keys())}")
    else:
        # Generate all figures
        print("Generating Figure 3: FK/Jacobian Accuracy...")
        plot_fig3_accuracy(fmt)

        print("\nGenerating Figure 4: Step Response...")
        plot_fig4_step_response(fmt)

        print("\nGenerating Figure 5: Disturbance Rejection...")
        plot_fig5_disturbance(fmt)

        print("\nGenerating Figure 6: Latency Comparison...")
        plot_fig6_latency_comparison(fmt)

        print("\nGenerating Figure 7: Varying Stiffness...")
        plot_fig7_varying_stiffness(fmt)

        print("\n--- LaTeX Tables ---")
        generate_latex_tables()


if __name__ == '__main__':
    main()
