#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Academic-style system architecture diagram for bilateral teleoperation
with FPGA-based impedance control and parallel FOC.

Generates a publication-ready figure (IEEE style).
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import numpy as np

# ── IEEE style settings ──
plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'CMU Serif', 'DejaVu Serif'],
    'font.size': 8,
    'axes.linewidth': 0.5,
    'text.usetex': False,
})

fig, ax = plt.subplots(1, 1, figsize=(12, 7.5))
ax.set_xlim(0, 24)
ax.set_ylim(0, 15)
ax.set_aspect('equal')
ax.axis('off')

# ── Color palette ──
C_KIN = '#dbeafe'       # kinematics - light blue
C_KIN_E = '#3b82f6'
C_CTRL = '#fef3c7'      # control - light yellow
C_CTRL_E = '#f59e0b'
C_FOC = '#dcfce7'       # FOC - light green
C_FOC_E = '#22c55e'
C_FB = '#fee2e2'        # feedback - light red
C_FB_E = '#ef4444'
C_HW = '#f3f4f6'        # hardware - light gray
C_HW_E = '#6b7280'
C_FPGA = '#6366f1'      # FPGA boundary


def draw_block(ax, x, y, w, h, text, subtext=None, color='#fff', edge='#333',
               time_text=None, fontsize=7.5, bold=True):
    """Draw a rounded rectangle module block."""
    box = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.08",
                         facecolor=color, edgecolor=edge, linewidth=1.0,
                         zorder=3)
    ax.add_patch(box)
    if subtext:
        ax.text(x + w/2, y + h*0.62, text, ha='center', va='center',
                fontsize=fontsize, fontweight='bold' if bold else 'normal', zorder=4)
        ax.text(x + w/2, y + h*0.28, subtext, ha='center', va='center',
                fontsize=fontsize - 1.5, style='italic', color='#555', zorder=4)
    else:
        ax.text(x + w/2, y + h/2, text, ha='center', va='center',
                fontsize=fontsize, fontweight='bold' if bold else 'normal', zorder=4)
    if time_text:
        ax.text(x + w - 0.15, y + h - 0.15, time_text, ha='right', va='top',
                fontsize=5.5, color='#888', style='italic', zorder=4)


def draw_arrow(ax, x1, y1, x2, y2, color='#333', lw=0.8, style='->', text=None,
               text_offset=(0, 0), text_color='#666', text_side='right'):
    """Draw an arrow between two points."""
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle=style, color=color, lw=lw),
                zorder=2)
    if text:
        mx = (x1 + x2) / 2 + text_offset[0]
        my = (y1 + y2) / 2 + text_offset[1]
        ha = 'left' if text_side == 'right' else 'right'
        ax.text(mx, my, text, fontsize=6, color=text_color,
                ha=ha, va='center', style='italic', zorder=4)


def draw_dashed_box(ax, x, y, w, h, label, color=C_FPGA):
    """Draw FPGA boundary dashed box."""
    box = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.15",
                         facecolor='none', edgecolor=color,
                         linewidth=1.2, linestyle='--', zorder=1)
    ax.add_patch(box)
    ax.text(x + w - 0.2, y + h - 0.15, label, ha='right', va='top',
            fontsize=7, fontweight='bold', color=color, zorder=4,
            bbox=dict(facecolor='white', edgecolor='none', pad=1))


# ════════════════════════════════════════════
# MASTER ARM (left side)
# ════════════════════════════════════════════
master_x = 0.5
BW = 4.8  # block width
BH = 0.7  # block height

# Title
ax.text(master_x + BW/2, 14.5, 'Master Arm (Operator)', ha='center',
        fontsize=10, fontweight='bold', color='#1a1a1a')

# Encoder
draw_block(ax, master_x, 13.2, BW, BH, 'Encoder × 6',
           'Joint angle measurement', C_HW, C_HW_E)
draw_arrow(ax, master_x + BW/2, 13.2, master_x + BW/2, 12.6,
           text='$q_{master}$[6]', text_offset=(0.15, 0))

# FPGA boundary
draw_dashed_box(ax, master_x - 0.2, 7.5, BW + 0.4, 5.0, 'FPGA')

# FK (master)
draw_block(ax, master_x, 11.5, BW, BH, 'Forward Kinematics',
           '$q_{master}$ → $x_{master}$', C_KIN, C_KIN_E)
draw_arrow(ax, master_x + BW/2, 11.5, master_x + BW/2, 10.9,
           text='$x_{master}$', text_offset=(0.15, 0))

# Force feedback label
ax.text(master_x + BW/2, 10.7, '── Force Feedback Path ──', ha='center',
        fontsize=7, color=C_FB_E, fontweight='bold')

# Force feedback mapping
draw_block(ax, master_x, 9.6, BW, BH, 'Force Feedback Mapping',
           '$τ_{fb} = K_t · iq_{slave}$', C_FB, C_FB_E)
draw_arrow(ax, master_x + BW/2, 9.6, master_x + BW/2, 9.0,
           text='$τ_{fb}$[6]', text_offset=(0.15, 0))

# FOC master
draw_block(ax, master_x, 8.0, BW, BH, 'FOC × 6 (Parallel)',
           'Force feedback actuation', C_FOC, C_FOC_E)
draw_arrow(ax, master_x + BW/2, 8.0, master_x + BW/2, 7.0)

# Motor
draw_block(ax, master_x, 6.2, BW, BH, 'BLDC Motor × 6',
           'Operator feels contact force', C_HW, C_HW_E)


# ════════════════════════════════════════════
# COMMUNICATION (center)
# ════════════════════════════════════════════
comm_x = 7.0

# ROS 2 box
ros_box = FancyBboxPatch((comm_x, 10.0), 2.0, 0.8, boxstyle="round,pad=0.1",
                          facecolor='#f0f0f0', edgecolor='#999', linewidth=1.0, zorder=3)
ax.add_patch(ros_box)
ax.text(comm_x + 1.0, 10.4, 'ROS 2\nNetwork', ha='center', va='center',
        fontsize=7.5, fontweight='bold', color='#555', zorder=4)

# Forward arrow (q_master): master → slave
ax.annotate('', xy=(comm_x + 2.0, 11.8), xytext=(master_x + BW, 11.8),
            arrowprops=dict(arrowstyle='->', color=C_KIN_E, lw=1.5), zorder=2)
ax.annotate('', xy=(9.8, 11.8), xytext=(comm_x + 2.0, 11.8),
            arrowprops=dict(arrowstyle='->', color=C_KIN_E, lw=1.5), zorder=2)
ax.text(comm_x + 1.0, 12.05, '$q_{master}$[6] (Position sync)',
        ha='center', fontsize=6.5, color=C_KIN_E, fontweight='bold', zorder=4)

# Backward arrow (iq_slave): slave → master
ax.annotate('', xy=(master_x + BW, 9.2), xytext=(comm_x, 9.2),
            arrowprops=dict(arrowstyle='->', color=C_FB_E, lw=1.5), zorder=2)
ax.annotate('', xy=(comm_x, 9.2), xytext=(9.8, 9.2),
            arrowprops=dict(arrowstyle='->', color=C_FB_E, lw=1.5), zorder=2)
ax.text(comm_x + 1.0, 8.95, '$iq_{slave}$[6] (Force feedback)',
        ha='center', fontsize=6.5, color=C_FB_E, fontweight='bold', zorder=4)


# ════════════════════════════════════════════
# SLAVE ARM (right side)
# ════════════════════════════════════════════
slave_x = 10.0

ax.text(slave_x + BW/2, 14.5, 'Slave Arm (Task Side)', ha='center',
        fontsize=10, fontweight='bold', color='#1a1a1a')

# Encoder
draw_block(ax, slave_x, 13.2, BW, BH, 'Encoder × 6',
           'Joint angle measurement', C_HW, C_HW_E)
draw_arrow(ax, slave_x + BW/2, 13.2, slave_x + BW/2, 12.6,
           text='$q_{slave}$[6]', text_offset=(0.15, 0))

# FPGA boundary
draw_dashed_box(ax, slave_x - 0.2, 4.3, BW + 0.4, 8.2,
                'FPGA — Impedance Control Pipeline (395 cyc = 3.95 μs)')

# FK + Jacobian
draw_block(ax, slave_x, 11.5, BW, BH, 'FK + Analytical Jacobian',
           '$q$ → $x$, $J$(6×6)  |  $q_{master}$ → $x_d$',
           C_KIN, C_KIN_E, time_text='382 cyc')
draw_arrow(ax, slave_x + BW/2, 11.5, slave_x + BW/2, 10.9,
           text='$x$, $x_d$, $J$', text_offset=(0.15, 0))

# Velocity estimator
draw_block(ax, slave_x, 10.2, BW, BH, 'Velocity Estimator',
           '$\\dot{x}$ = IIR filter of $(x[n]-x[n-1])/dt$',
           C_KIN, C_KIN_E, time_text='3 cyc')
draw_arrow(ax, slave_x + BW/2, 10.2, slave_x + BW/2, 9.6,
           text='$\\dot{x}$', text_offset=(0.15, 0))

# Impedance control
draw_block(ax, slave_x, 8.9, BW, BH, 'Impedance Control Law',
           '$F = K_s·(x_d - x) + D_s·(\\dot{x}_d - \\dot{x})$',
           C_CTRL, C_CTRL_E, time_text='2 cyc')
draw_arrow(ax, slave_x + BW/2, 8.9, slave_x + BW/2, 8.3,
           text='$F$(6×1)', text_offset=(0.15, 0))

# JT mapping
draw_block(ax, slave_x, 7.6, BW, BH, '$J^T$ Torque Mapping',
           '$τ = J^T · F$', C_CTRL, C_CTRL_E, time_text='6 cyc')
draw_arrow(ax, slave_x + BW/2, 7.6, slave_x + BW/2, 7.0,
           text='$τ$[6]', text_offset=(0.15, 0))

# Torque to Iq
draw_block(ax, slave_x, 6.3, BW, BH, 'Torque → $I_q$',
           '$iq_{ref} = τ / K_t$', C_CTRL, C_CTRL_E, time_text='2 cyc')
draw_arrow(ax, slave_x + BW/2, 6.3, slave_x + BW/2, 5.7,
           text='$iq_{ref}$[6]', text_offset=(0.15, 0))

# FOC slave
draw_block(ax, slave_x, 4.7, BW, BH, 'FOC × 6 (Fully Parallel)',
           'Clarke → Park → PI → SVPWM', C_FOC, C_FOC_E)
draw_arrow(ax, slave_x + BW/2, 4.7, slave_x + BW/2, 4.1,
           text='PWM × 18', text_offset=(0.15, 0))

# Motor
draw_block(ax, slave_x, 3.3, BW, BH, 'BLDC Motor × 6',
           'Compliant tracking of master arm', C_HW, C_HW_E)


# ════════════════════════════════════════════
# Pipeline bracket (right side annotation)
# ════════════════════════════════════════════
bx = slave_x + BW + 0.3
ax.annotate('', xy=(bx + 0.15, 12.2), xytext=(bx + 0.15, 5.0),
            arrowprops=dict(arrowstyle='-', color=C_FPGA, lw=1.0), zorder=2)
ax.plot([bx, bx + 0.15], [12.2, 12.2], color=C_FPGA, lw=1.0, zorder=2)
ax.plot([bx, bx + 0.15], [5.0, 5.0], color=C_FPGA, lw=1.0, zorder=2)
ax.text(bx + 0.4, 8.6, '395 cycles\n= 3.95 μs\n@ 100 MHz\n\n253.2 kHz\nmax rate',
        ha='left', va='center', fontsize=6.5, color=C_FPGA,
        fontweight='bold', linespacing=1.4, zorder=4)


# ════════════════════════════════════════════
# Legend
# ════════════════════════════════════════════
legend_y = 2.2
legend_items = [
    (C_KIN, C_KIN_E, 'Kinematics'),
    (C_CTRL, C_CTRL_E, 'Impedance Control'),
    (C_FOC, C_FOC_E, 'FOC Motor Drive'),
    (C_FB, C_FB_E, 'Force Feedback'),
    (C_HW, C_HW_E, 'Hardware'),
]
total_w = len(legend_items) * 3.2
start_x = 12.0 / 2 - total_w / 2 + 3
for i, (fc, ec, label) in enumerate(legend_items):
    lx = start_x + i * 3.2
    box = FancyBboxPatch((lx, legend_y), 0.4, 0.3, boxstyle="round,pad=0.02",
                          facecolor=fc, edgecolor=ec, linewidth=1.0, zorder=3)
    ax.add_patch(box)
    ax.text(lx + 0.6, legend_y + 0.15, label, ha='left', va='center',
            fontsize=7, color='#444', zorder=4)


# ════════════════════════════════════════════
# Performance summary
# ════════════════════════════════════════════
perf_y = 1.0
ax.text(8.0, perf_y, 'Q16.16 fixed-point  |  Target: Xilinx Alveo U200  |  '
        'LUT: 0.93%  |  DSP: 6.14%  |  FOC clock: 36.864 MHz (independent domain)',
        ha='center', va='center', fontsize=6.5, color='#777', style='italic', zorder=4)


# ── Save ──
out_path = 'D:/robot/impedance_control/figures/fig_system_architecture.png'
plt.savefig(out_path, dpi=300, bbox_inches='tight', facecolor='white', pad_inches=0.3)
plt.close()
print(f'Saved: {out_path}')

out_pdf = 'D:/robot/impedance_control/figures/fig_system_architecture.pdf'
fig2, ax2 = plt.subplots(1, 1, figsize=(12, 7.5))
ax2.set_xlim(0, 24)
ax2.set_ylim(0, 15)
ax2.set_aspect('equal')
ax2.axis('off')
plt.close(fig2)
print(f'To generate PDF, re-run with pdf backend or use: plt.savefig(..., format="pdf")')
