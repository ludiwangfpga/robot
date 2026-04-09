#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
FPGA System Architecture Diagram - IEEE/RA-L Top Journal Style

Focuses on the FPGA internal architecture:
  - Impedance control pipeline (sys_clk 100 MHz)
  - 6-channel parallel FOC (foc_clk 36.864 MHz)
  - Two clock domains
  - Signal flow with data widths
  - External interfaces (encoder, ADC, PWM)
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import numpy as np

# ── IEEE style ──
plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'CMU Serif', 'DejaVu Serif'],
    'font.size': 8,
    'axes.linewidth': 0.5,
    'text.usetex': False,
})

fig, ax = plt.subplots(1, 1, figsize=(12, 15))
ax.set_xlim(-1.0, 22.0)
ax.set_ylim(-1.5, 27.0)
ax.set_aspect('equal')
ax.axis('off')

# ══════════════════════════════════════
# Color palette
# ══════════════════════════════════════
C_ENC  = '#e8f0fe'  ; CE_ENC  = '#4285f4'   # encoder/input - blue
C_KIN  = '#d1e7fd'  ; CE_KIN  = '#1a73e8'   # kinematics - darker blue
C_VEL  = '#c6dafc'  ; CE_VEL  = '#1557b0'   # velocity - even darker
C_IMP  = '#fff3cd'  ; CE_IMP  = '#e6a800'   # impedance ctrl - gold
C_JT   = '#ffeeba'  ; CE_JT   = '#d4890e'   # JT mapping - amber
C_T2IQ = '#ffe0b2'  ; CE_T2IQ = '#c46000'   # torque-to-iq - orange
C_FOC  = '#d4edda'  ; CE_FOC  = '#28a745'   # FOC - green
C_FOC_SUB = '#e8f5e9'; CE_FOC_SUB = '#388e3c'  # FOC submodules
C_PWM  = '#f8d7da'  ; CE_PWM  = '#dc3545'   # PWM output - red
C_ADC  = '#e2e3e5'  ; CE_ADC  = '#6c757d'   # ADC - gray
C_CLK  = '#e8daef'  ; CE_CLK  = '#7b2cbf'   # clock domain


def block(x, y, w, h, label, sub=None, fc='#fff', ec='#333', fs=8,
          cyc=None, bold=True, ls='-', lw=1.0, alpha=1.0):
    """Draw a module block."""
    box = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.06",
                          facecolor=fc, edgecolor=ec, linewidth=lw,
                          linestyle=ls, alpha=alpha, zorder=3)
    ax.add_patch(box)
    if sub:
        ax.text(x + w/2, y + h*0.63, label, ha='center', va='center',
                fontsize=fs, fontweight='bold' if bold else 'normal', zorder=4)
        ax.text(x + w/2, y + h*0.30, sub, ha='center', va='center',
                fontsize=fs - 1.5, color='#555', zorder=4)
    else:
        ax.text(x + w/2, y + h/2, label, ha='center', va='center',
                fontsize=fs, fontweight='bold' if bold else 'normal', zorder=4)
    if cyc:
        ax.text(x + w - 0.12, y + 0.12, cyc, ha='right', va='bottom',
                fontsize=6, color='#888', style='italic', zorder=4)


def arrow(x1, y1, x2, y2, color='#333', lw=1.0, text=None, tofs=(0,0),
          tc='#555', tfs=6.5, ha='left', style='->'):
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle=style, color=color, lw=lw), zorder=2)
    if text:
        mx = (x1+x2)/2 + tofs[0]
        my = (y1+y2)/2 + tofs[1]
        ax.text(mx, my, text, fontsize=tfs, color=tc, ha=ha, va='center', zorder=4)


def brace_right(x, y_top, y_bot, text, color='#555'):
    """Draw a right brace annotation."""
    mid = (y_top + y_bot) / 2
    ax.plot([x, x+0.2], [y_top, y_top], color=color, lw=0.8, zorder=2)
    ax.plot([x, x+0.2], [y_bot, y_bot], color=color, lw=0.8, zorder=2)
    ax.plot([x+0.2, x+0.2], [y_top, y_bot], color=color, lw=0.8, zorder=2)
    ax.plot([x+0.2, x+0.4], [mid, mid], color=color, lw=0.8, zorder=2)
    ax.text(x+0.55, mid, text, fontsize=7, color=color, ha='left', va='center',
            fontweight='bold', zorder=4)


# ══════════════════════════════════════
# FPGA boundary
# ══════════════════════════════════════
fpga_box = FancyBboxPatch((0.3, 1.8), 20.4, 22.8, boxstyle="round,pad=0.2",
                           facecolor='none', edgecolor='#4a4a8a',
                           linewidth=1.8, linestyle='--', zorder=1)
ax.add_patch(fpga_box)
ax.text(10, 24.8, 'FPGA  (Xilinx Alveo U200)', ha='center', va='center',
        fontsize=11, fontweight='bold', color='#4a4a8a', zorder=4,
        bbox=dict(facecolor='white', edgecolor='none', pad=2))

# ── Clock domain boxes ──
# sys_clk domain
sys_box = FancyBboxPatch((0.8, 8.5), 10.4, 15.5, boxstyle="round,pad=0.15",
                          facecolor='#f8f9ff', edgecolor=CE_CLK,
                          linewidth=1.2, linestyle=':', alpha=0.5, zorder=0)
ax.add_patch(sys_box)
ax.text(5.5, 23.7, 'sys_clk = 100 MHz', ha='center', fontsize=8,
        fontweight='bold', color=CE_CLK, zorder=4,
        bbox=dict(facecolor='white', edgecolor=CE_CLK, pad=2, boxstyle='round,pad=0.15'))

# foc_clk domain
foc_box = FancyBboxPatch((0.8, 2.2), 19.4, 5.8, boxstyle="round,pad=0.15",
                          facecolor='#f5fff5', edgecolor=CE_FOC,
                          linewidth=1.2, linestyle=':', alpha=0.5, zorder=0)
ax.add_patch(foc_box)
ax.text(10, 7.7, 'foc_clk = 36.864 MHz', ha='center', fontsize=8,
        fontweight='bold', color=CE_FOC, zorder=4,
        bbox=dict(facecolor='white', edgecolor=CE_FOC, pad=2, boxstyle='round,pad=0.15'))


# ══════════════════════════════════════
# External inputs (top)
# ══════════════════════════════════════
# Encoder input
block(1.5, 25.2, 4.0, 0.9, 'Encoder × 6', 'Incremental / Absolute',
      fc=C_ENC, ec=CE_ENC, fs=8)
arrow(3.5, 25.2, 3.5, 24.4, CE_ENC, 1.2,
      '$q_i$ [6] (Q16.16)', tofs=(0.15, 0), tc=CE_ENC)

# xd input (desired pose from master)
block(7.0, 25.2, 4.5, 0.9, 'Master Arm (ROS 2)',
      '$q_{master}$ → FK → $x_d$',
      fc='#fff0f0', ec='#cc4444', fs=8)
arrow(9.25, 25.2, 9.25, 24.4, '#cc4444', 1.2,
      '$x_d$, $\\dot{x}_d$ [12]', tofs=(0.15, 0), tc='#cc4444')

# ── Pipeline width ──
PW = 8.0   # pipeline block width
PX = 1.5   # pipeline x start

# ══════════════════════════════════════
# Stage 1: FK + Analytical Jacobian
# ══════════════════════════════════════
block(PX, 22.5, PW, 1.2,
      'Forward Kinematics + Analytical Jacobian',
      'DH parameters → $T_{0→6}$ → $x$, $R$ | $J(q) \\in \\mathbb{R}^{6×6}$',
      fc=C_KIN, ec=CE_KIN, fs=8.5, cyc='382 cyc')

arrow(PX + PW/2, 22.5, PX + PW/2, 21.7, CE_KIN, 1.0,
      '$x$ (6×1), $J$ (6×6)', tofs=(0.15, 0), tc=CE_KIN, tfs=6.5)

# ══════════════════════════════════════
# Stage 2: Velocity Estimator
# ══════════════════════════════════════
block(PX, 20.8, PW, 1.0,
      'Velocity Estimator (IIR)',
      '$\\dot{x}[n] = α·\\frac{x[n]-x[n-1]}{Δt} + (1-α)·\\dot{x}[n-1]$',
      fc=C_VEL, ec=CE_VEL, fs=8.5, cyc='3 cyc')

arrow(PX + PW/2, 20.8, PX + PW/2, 20.0, CE_VEL, 1.0,
      '$\\dot{x}$ (6×1)', tofs=(0.15, 0), tc=CE_VEL, tfs=6.5)

# ══════════════════════════════════════
# Stage 3: Impedance Control Law
# ══════════════════════════════════════
block(PX, 19.0, PW, 1.2,
      'Task-Space Impedance Control',
      '$F = K_s \\cdot (x_d - x) + D_s \\cdot (\\dot{x}_d - \\dot{x})$',
      fc=C_IMP, ec=CE_IMP, fs=8.5, cyc='2 cyc')

# xd arrow into impedance controller
arrow(9.25, 24.0, 9.25, 19.6, '#cc4444', 0.8, style='->')
ax.plot([9.25, PX + PW], [19.6, 19.6], color='#cc4444', lw=0.8, zorder=2)

arrow(PX + PW/2, 19.0, PX + PW/2, 18.2, CE_IMP, 1.0,
      '$F$ (6×1) [N, Nm]', tofs=(0.15, 0), tc=CE_IMP, tfs=6.5)

# ══════════════════════════════════════
# Stage 4: J^T Torque Mapping
# ══════════════════════════════════════
block(PX, 17.2, PW, 1.2,
      '$J^T$ Torque Mapping',
      '$τ = J^T \\cdot F$   (matrix-vector mult 6×6)',
      fc=C_JT, ec=CE_JT, fs=8.5, cyc='6 cyc')

# J feeds back from FK to JT
arrow(PX + PW + 0.3, 22.8, PX + PW + 0.3, 17.8, CE_KIN, 0.8, style='->')
ax.plot([PX + PW, PX + PW + 0.3], [22.8, 22.8], color=CE_KIN, lw=0.8, zorder=2)
ax.plot([PX + PW, PX + PW + 0.3], [17.8, 17.8], color=CE_KIN, lw=0.8, zorder=2)
ax.text(PX + PW + 0.45, 20.3, '$J$', fontsize=7, color=CE_KIN, ha='left',
        va='center', fontweight='bold', rotation=90, zorder=4)

arrow(PX + PW/2, 17.2, PX + PW/2, 16.4, CE_JT, 1.0,
      '$τ_i$ [6] (Q16.16) [Nm]', tofs=(0.15, 0), tc=CE_JT, tfs=6.5)

# ══════════════════════════════════════
# Stage 5: Torque → Iq
# ══════════════════════════════════════
block(PX, 15.4, PW, 1.2,
      'Torque → $I_q$ Converter',
      '$iq_{ref,i} = \\mathrm{clamp}(τ_i / K_t,\\, I_{min},\\, I_{max})$',
      fc=C_T2IQ, ec=CE_T2IQ, fs=8.5, cyc='2 cyc')

arrow(PX + PW/2, 15.4, PX + PW/2, 14.6, CE_T2IQ, 1.0,
      '$iq_{ref}$ [6] (signed 16-bit)', tofs=(0.15, 0), tc=CE_T2IQ, tfs=6.5)

# ══════════════════════════════════════
# Clock Domain Crossing
# ══════════════════════════════════════
block(PX, 13.8, PW, 0.8,
      'Clock Domain Crossing (Double-FF Sync)',
      fc=C_CLK, ec=CE_CLK, fs=7.5, bold=False)
ax.text(PX + PW/2, 13.55, 'sys_clk (100 MHz) → foc_clk (36.864 MHz)',
        ha='center', fontsize=6, color=CE_CLK, style='italic', zorder=4)

arrow(PX + PW/2, 13.8, PX + PW/2, 13.0, CE_CLK, 1.0)

# ══════════════════════════════════════
# Pipeline timing bracket (right side)
# ══════════════════════════════════════
BX = PX + PW + 1.5
brace_right(BX, 23.7, 15.4, '395 cycles\n= 3.95 μs\n@ 100 MHz\n\nControl rate:\n253.2 kHz',
            color='#4a4a8a')

# Individual stage brackets
BX2 = BX + 4.5
stages = [
    (23.7, 22.5, '382 cyc', '3.82 μs'),
    (21.8, 20.8, '3 cyc', '30 ns'),
    (20.2, 19.0, '2 cyc', '20 ns'),
    (18.4, 17.2, '6 cyc', '60 ns'),
    (16.6, 15.4, '2 cyc', '20 ns'),
]
for yt, yb, cyc_text, time_text in stages:
    mid = (yt + yb) / 2
    ax.plot([BX2, BX2+0.1], [yt, yt], color='#aaa', lw=0.5)
    ax.plot([BX2, BX2+0.1], [yb, yb], color='#aaa', lw=0.5)
    ax.plot([BX2+0.1, BX2+0.1], [yt, yb], color='#aaa', lw=0.5)
    ax.text(BX2+0.25, mid, f'{cyc_text}\n{time_text}', fontsize=5.5,
            color='#888', ha='left', va='center', zorder=4)


# ══════════════════════════════════════
# FOC Array (6 parallel channels)
# ══════════════════════════════════════
# Main FOC block
foc_y = 2.5
foc_h = 4.8
foc_w = 18.0

# Draw 6 parallel FOC channels
ch_w = 3.0
ch_h = 3.8
ch_gap = 0.25
ch_y = foc_y + 0.3
ch_start_x = 1.0

for i in range(6):
    cx = ch_start_x + i * (ch_w + ch_gap)

    # Channel box
    ch_box = FancyBboxPatch((cx, ch_y), ch_w, ch_h, boxstyle="round,pad=0.05",
                             facecolor=C_FOC, edgecolor=CE_FOC,
                             linewidth=0.8, zorder=3)
    ax.add_patch(ch_box)

    # Channel label
    ax.text(cx + ch_w/2, ch_y + ch_h - 0.2, f'FOC Ch.{i}',
            ha='center', fontsize=7, fontweight='bold', color=CE_FOC, zorder=4)

    # Sub-modules inside each channel
    sub_w = ch_w - 0.3
    sub_x = cx + 0.15
    sub_fs = 6

    subs = [
        (ch_y + 3.0, 'Clarke', C_FOC_SUB),
        (ch_y + 2.4, 'Park', C_FOC_SUB),
        (ch_y + 1.8, 'PI × 2', C_FOC_SUB),
        (ch_y + 1.2, 'Inv. Park', C_FOC_SUB),
        (ch_y + 0.55, 'SVPWM', C_FOC_SUB),
    ]
    for sy, slabel, sc in subs:
        sub_box = FancyBboxPatch((sub_x, sy), sub_w, 0.48, boxstyle="round,pad=0.02",
                                  facecolor=sc, edgecolor=CE_FOC_SUB,
                                  linewidth=0.4, zorder=4)
        ax.add_patch(sub_box)
        ax.text(sub_x + sub_w/2, sy + 0.24, slabel, ha='center', va='center',
                fontsize=sub_fs, color='#2d5a2d', zorder=5)

    # Input arrow (iq_ref)
    arrow(cx + ch_w/2, ch_y + ch_h + 0.6, cx + ch_w/2, ch_y + ch_h,
          CE_T2IQ, 0.6)

    # Output arrow (PWM)
    arrow(cx + ch_w/2, ch_y, cx + ch_w/2, ch_y - 0.5,
          CE_PWM, 0.8)
    ax.text(cx + ch_w/2, ch_y - 0.7, 'PWM×3', ha='center', fontsize=5.5,
            color=CE_PWM, zorder=4)

    # ADC input (from left)
    ax.text(cx + ch_w + 0.05, ch_y + 2.9, 'ADC', fontsize=4.5,
            color=CE_ADC, ha='left', va='center', rotation=90, zorder=4)

    # Encoder input (from left)
    ax.text(cx - 0.05, ch_y + 2.9, '$φ_i$', fontsize=5,
            color=CE_ENC, ha='right', va='center', zorder=4)

# iq_ref distribution line
iq_line_y = ch_y + ch_h + 0.8
ax.plot([PX + PW/2, PX + PW/2], [13.0, iq_line_y], color=CE_T2IQ, lw=1.0, zorder=2)
ax.plot([ch_start_x + ch_w/2, ch_start_x + 5*(ch_w+ch_gap) + ch_w/2],
        [iq_line_y, iq_line_y], color=CE_T2IQ, lw=1.0, zorder=2)
for i in range(6):
    cx = ch_start_x + i * (ch_w + ch_gap) + ch_w/2
    ax.plot([cx, cx], [iq_line_y, iq_line_y - 0.2], color=CE_T2IQ, lw=0.8, zorder=2)


# ══════════════════════════════════════
# FOC frequency annotation
# ══════════════════════════════════════
ax.text(20.0, 5.5,
        'FOC rate:\n36.864 MHz / 2048\n= 18 kHz',
        ha='center', va='center', fontsize=7, color=CE_FOC,
        fontweight='bold', zorder=4,
        bbox=dict(facecolor='white', edgecolor=CE_FOC, pad=3,
                  boxstyle='round,pad=0.2'))

# ══════════════════════════════════════
# External outputs (bottom)
# ══════════════════════════════════════
# PWM output
block(1.0, 0.2, 5.0, 0.8, 'Half-Bridge Driver × 6',
      '18 PWM signals → BLDC × 6', fc=C_PWM, ec=CE_PWM, fs=7.5)

# ADC input
block(7.5, 0.2, 5.0, 0.8, 'Current ADC × 6',
      '12-bit × 3-phase × 6 channels', fc=C_ADC, ec=CE_ADC, fs=7.5)

# Motor
block(14.0, 0.2, 5.5, 0.8, 'BLDC Motor × 6',
      '6-DOF Slave Arm Joints', fc='#fff', ec='#333', fs=7.5)


# ══════════════════════════════════════
# Legend
# ══════════════════════════════════════
leg_y = -0.6
leg_items = [
    (C_KIN, CE_KIN, 'Kinematics'),
    (C_IMP, CE_IMP, 'Impedance Control'),
    (C_FOC, CE_FOC, 'FOC Drive'),
    (C_CLK, CE_CLK, 'Clock Crossing'),
    (C_PWM, CE_PWM, 'PWM Output'),
]
total_leg = len(leg_items) * 3.5
lx_start = 10 - total_leg/2
for i, (fc, ec, label) in enumerate(leg_items):
    lx = lx_start + i * 3.5
    b = FancyBboxPatch((lx, leg_y), 0.4, 0.3, boxstyle="round,pad=0.02",
                        facecolor=fc, edgecolor=ec, linewidth=0.8, zorder=3)
    ax.add_patch(b)
    ax.text(lx + 0.55, leg_y + 0.15, label, ha='left', va='center',
            fontsize=6.5, color='#444', zorder=4)

# ══════════════════════════════════════
# Title
# ══════════════════════════════════════
ax.text(10, 26.2,
        'FPGA-Based Impedance Control and 6-Channel Parallel FOC Architecture',
        ha='center', fontsize=12, fontweight='bold', zorder=4)
ax.text(10, 25.7,
        'Q16.16 Fixed-Point | Dual Clock Domain | 6-DOF Slave Arm',
        ha='center', fontsize=8.5, color='#666', zorder=4)

# ══════════════════════════════════════
# Save
# ══════════════════════════════════════
out_png = 'D:/robot/impedance_control/figures/fig_fpga_architecture.png'
plt.savefig(out_png, dpi=300, bbox_inches='tight', facecolor='white', pad_inches=0.2)

out_pdf = 'D:/robot/impedance_control/figures/fig_fpga_architecture.pdf'
plt.savefig(out_pdf, dpi=300, bbox_inches='tight', facecolor='white', pad_inches=0.2)

plt.close()
print(f'Saved: {out_png}')
print(f'Saved: {out_pdf}')
