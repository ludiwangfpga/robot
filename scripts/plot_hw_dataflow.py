#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Hardware Data Flow System Architecture Diagram
Style reference: ACCL+ (OSDI'24) - hardware-centric, showing data paths
between physical components (CPU, FPGA, RAM, ADC, Encoder, Driver, Motor)

Focus: How data physically flows through the hardware, not algorithms.
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle
import numpy as np

plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'CMU Serif', 'DejaVu Serif'],
    'font.size': 8,
    'text.usetex': False,
})

fig, ax = plt.subplots(1, 1, figsize=(16, 11))
ax.set_xlim(-0.5, 32)
ax.set_ylim(-1.5, 21)
ax.set_aspect('equal')
ax.axis('off')

# ══════════════════════════════════════
# Colors
# ══════════════════════════════════════
C_CPU    = '#e3f2fd'  ; CE_CPU    = '#1565c0'
C_MEM    = '#fff8e1'  ; CE_MEM    = '#f57f17'
C_FPGA   = '#f3e5f5'  ; CE_FPGA   = '#7b1fa2'
C_BRAM   = '#fce4ec'  ; CE_BRAM   = '#c62828'
C_LOGIC  = '#e8eaf6'  ; CE_LOGIC  = '#283593'
C_FOC    = '#e8f5e9'  ; CE_FOC    = '#2e7d32'
C_IO     = '#efebe9'  ; CE_IO     = '#4e342e'
C_MOTOR  = '#eceff1'  ; CE_MOTOR  = '#37474f'
C_BUS    = '#fff3e0'  ; CE_BUS    = '#e65100'
C_CLK    = '#f1f8e9'  ; CE_CLK    = '#558b2f'


def hw_block(x, y, w, h, label, sub=None, fc='#fff', ec='#333',
             fs=8, bold=True, lw=1.2, sub_fs=6.5, sub_color='#555'):
    box = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.08",
                          facecolor=fc, edgecolor=ec, linewidth=lw, zorder=3)
    ax.add_patch(box)
    if sub:
        ax.text(x + w/2, y + h*0.62, label, ha='center', va='center',
                fontsize=fs, fontweight='bold' if bold else 'normal', zorder=4)
        ax.text(x + w/2, y + h*0.28, sub, ha='center', va='center',
                fontsize=sub_fs, color=sub_color, zorder=4)
    else:
        ax.text(x + w/2, y + h/2, label, ha='center', va='center',
                fontsize=fs, fontweight='bold' if bold else 'normal', zorder=4)


def data_arrow(x1, y1, x2, y2, color='#333', lw=1.2, text=None,
               tofs=(0, 0.15), tfs=6, tc='#333', style='->', ha='center',
               va='bottom', text_bg=True, zorder=2):
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle=style, color=color, lw=lw,
                                shrinkA=1, shrinkB=1), zorder=zorder)
    if text:
        mx = (x1 + x2)/2 + tofs[0]
        my = (y1 + y2)/2 + tofs[1]
        bbox = dict(facecolor='white', edgecolor='none', pad=0.5, alpha=0.85) if text_bg else None
        ax.text(mx, my, text, fontsize=tfs, color=tc, ha=ha, va=va,
                bbox=bbox, zorder=zorder + 1)


def bus_line(x1, y1, x2, y2, color=CE_BUS, lw=2.5, label=None, label_pos=0.5,
             label_side='top'):
    ax.plot([x1, x2], [y1, y2], color=color, lw=lw, zorder=2, solid_capstyle='round')
    if label:
        mx = x1 + (x2 - x1) * label_pos
        my = y1 + (y2 - y1) * label_pos
        offset = 0.2 if label_side == 'top' else -0.2
        va = 'bottom' if label_side == 'top' else 'top'
        ax.text(mx, my + offset, label, fontsize=6, color=color,
                ha='center', va=va, fontweight='bold',
                bbox=dict(facecolor='white', edgecolor='none', pad=0.3, alpha=0.9),
                zorder=4)


def boundary_box(x, y, w, h, label, color='#666', fs=9):
    box = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.15",
                          facecolor='none', edgecolor=color,
                          linewidth=1.5, linestyle='--', zorder=1)
    ax.add_patch(box)
    ax.text(x + 0.3, y + h - 0.3, label, ha='left', va='top',
            fontsize=fs, fontweight='bold', color=color, zorder=4,
            bbox=dict(facecolor='white', edgecolor='none', pad=1.5))


# ══════════════════════════════════════════════════════════════
# LAYER 1: Host PC (top-left)
# ══════════════════════════════════════════════════════════════
boundary_box(-0.2, 15.5, 8.5, 5.0, 'Host PC', CE_CPU)

hw_block(0.2, 18.5, 3.5, 1.5, 'CPU', 'ROS 2 / Linux\nTrajectory Planner',
         fc=C_CPU, ec=CE_CPU, fs=9)
hw_block(4.5, 18.5, 3.5, 1.5, 'DDR4 RAM', 'Shared Memory\nDMA Buffer',
         fc=C_MEM, ec=CE_MEM, fs=9)
hw_block(0.2, 16.0, 3.5, 1.5, 'Master Arm\nInterface', 'USB / UART\n7-DOF Encoder',
         fc=C_IO, ec=CE_IO, fs=8)

# CPU <-> DDR
bus_line(3.7, 19.25, 4.5, 19.25, CE_MEM, 2.5, 'DDR4 Bus', 0.5)

# CPU <-> Master Arm Interface
data_arrow(1.95, 18.5, 1.95, 17.5, CE_CPU, 1.0,
           '$q_{m}$[7]', tofs=(0.6, 0), tfs=6, tc=CE_CPU)

# CPU -> DDR: xd, Ks, Ds parameters
ax.text(6.25, 18.3, '$x_d$, $K_s$, $D_s$\nparameters', fontsize=5.5,
        ha='center', color=CE_MEM, style='italic', zorder=4)

# ══════════════════════════════════════════════════════════════
# PCIe / AXI connection (Host <-> FPGA)
# ══════════════════════════════════════════════════════════════
pcie_y = 14.8
hw_block(2.0, pcie_y, 4.5, 0.8, 'PCIe Gen3 x16 / AXI4-MM',
         fc=C_BUS, ec=CE_BUS, fs=7.5, lw=1.5)

data_arrow(4.25, 15.5, 4.25, pcie_y + 0.8, CE_BUS, 1.5,
           'DMA Write/Read', tofs=(1.5, 0), tfs=6, tc=CE_BUS)

data_arrow(4.25, pcie_y, 4.25, 13.8, CE_BUS, 1.5,
           '$x_d$, $K_s$, $D_s$ → Registers', tofs=(2.0, 0), tfs=5.5, tc=CE_BUS)


# ══════════════════════════════════════════════════════════════
# LAYER 2: FPGA (main area)
# ══════════════════════════════════════════════════════════════
boundary_box(-0.2, 2.0, 31.7, 11.5, 'FPGA  —  Xilinx Alveo U200  (xcu200)',
             CE_FPGA, fs=10)

# ── Clock Generation ──
hw_block(0.3, 11.5, 3.0, 1.3, 'PLL / MMCM', 'Clock Generation',
         fc=C_CLK, ec=CE_CLK, fs=7.5)
ax.text(0.5, 12.55, '100 MHz', fontsize=5.5, color=CE_CLK, fontweight='bold', zorder=4)
ax.text(0.5, 12.25, '36.864 MHz', fontsize=5.5, color=CE_CLK, fontweight='bold', zorder=4)

# ── AXI-Lite Register File ──
hw_block(0.3, 9.5, 3.0, 1.5, 'AXI-Lite\nRegister File',
         '$x_d$, $K_s$, $D_s$\n$K_p$, $K_i$',
         fc=C_BUS, ec=CE_BUS, fs=7.5, sub_fs=5.5)

# ── BRAM (lookup tables, intermediate) ──
hw_block(0.3, 7.5, 3.0, 1.5, 'BRAM', 'sin/cos LUT\nDH Parameters\nIntermediate Data',
         fc=C_BRAM, ec=CE_BRAM, fs=8)

# ── Connection from PCIe to register file ──
data_arrow(3.3, 13.8, 1.8, 11.5 + 1.3, CE_BUS, 1.0)

# ══════════════════════════════════════════════════════════════
# Encoder Interface (left input)
# ══════════════════════════════════════════════════════════════
hw_block(4.0, 11.5, 3.2, 1.3, 'Encoder\nInterface × 6',
         'SPI / SSI Deserializer',
         fc=C_IO, ec=CE_IO, fs=7.5)

# External encoder
hw_block(4.0, 13.8, 3.2, 1.0, 'Encoder × 6',
         '12-bit Absolute', fc=C_MOTOR, ec=CE_MOTOR, fs=7.5, sub_fs=6)

data_arrow(5.6, 13.8, 5.6, 12.8, CE_IO, 1.2,
           'SPI (φ, 12-bit)', tofs=(1.3, 0), tfs=5.5, tc=CE_IO)

# ══════════════════════════════════════════════════════════════
# Impedance Control Pipeline (center, sys_clk domain)
# ══════════════════════════════════════════════════════════════
# Pipeline boundary
pipe_x = 7.8
pipe_w = 10.5
pipe_y = 3.0
pipe_h = 10.0
pipe_box = FancyBboxPatch((pipe_x, pipe_y), pipe_w, pipe_h,
                           boxstyle="round,pad=0.1",
                           facecolor='#fafafe', edgecolor=CE_LOGIC,
                           linewidth=1.2, linestyle=':', zorder=0, alpha=0.6)
ax.add_patch(pipe_box)
ax.text(pipe_x + pipe_w/2, pipe_y + pipe_h - 0.2,
        'Impedance Control Pipeline  (sys_clk = 100 MHz)',
        ha='center', fontsize=8, fontweight='bold', color=CE_LOGIC, zorder=4)

# Pipeline stages as horizontal flow (left to right)
BW = 2.8  # block width
BH = 1.1  # block height
sy = 8.8  # stage y position

# Stage 1: FK + Jacobian
hw_block(pipe_x + 0.3, sy, BW, BH, 'FK + Jacobian',
         '382 cycles\n6×6 matrix output',
         fc=C_LOGIC, ec=CE_LOGIC, fs=7.5, sub_fs=5.5)

# Stage 2: Velocity Est
hw_block(pipe_x + 0.3 + BW + 0.4, sy, BW - 0.5, BH, 'Velocity\nEstimator',
         '3 cycles',
         fc=C_LOGIC, ec=CE_LOGIC, fs=7.5, sub_fs=5.5)

# Stage 3: Impedance Ctrl
hw_block(pipe_x + 0.3 + 2*(BW + 0.4) - 0.5, sy, BW - 0.3, BH, 'Impedance\nControl',
         '2 cycles',
         fc=C_LOGIC, ec=CE_LOGIC, fs=7.5, sub_fs=5.5)

# Stage arrows (horizontal)
s1_right = pipe_x + 0.3 + BW
s2_left = pipe_x + 0.3 + BW + 0.4
s2_right = s2_left + BW - 0.5
s3_left = pipe_x + 0.3 + 2*(BW + 0.4) - 0.5
s3_right = s3_left + BW - 0.3

data_arrow(s1_right, sy + BH/2, s2_left, sy + BH/2, CE_LOGIC, 1.0,
           '$x$, $J$', tofs=(0, 0.15), tfs=5.5, tc=CE_LOGIC)
data_arrow(s2_right, sy + BH/2, s3_left, sy + BH/2, CE_LOGIC, 1.0,
           '$\\dot{x}$', tofs=(0, 0.15), tfs=5.5, tc=CE_LOGIC)

# Stage 4 & 5 (second row)
sy2 = 6.5
hw_block(pipe_x + 0.3, sy2, BW, BH, '$J^T$ Mapping',
         '6 cycles\n$τ = J^T \\cdot F$',
         fc=C_LOGIC, ec=CE_LOGIC, fs=7.5, sub_fs=5.5)

hw_block(pipe_x + 0.3 + BW + 0.4, sy2, BW - 0.5, BH, 'Torque →\n$I_q$ Convert',
         '2 cycles',
         fc=C_LOGIC, ec=CE_LOGIC, fs=7.5, sub_fs=5.5)

# Impedance -> JT (downward)
data_arrow(s3_left + (BW-0.3)/2, sy, pipe_x + 0.3 + BW/2, sy2 + BH,
           CE_LOGIC, 1.0, '$F$ (6×1)', tofs=(-1.5, 0), tfs=5.5, tc=CE_LOGIC)

# JT -> Torque2Iq
jt_right = pipe_x + 0.3 + BW
t2iq_left = pipe_x + 0.3 + BW + 0.4
data_arrow(jt_right, sy2 + BH/2, t2iq_left, sy2 + BH/2, CE_LOGIC, 1.0,
           '$τ$[6]', tofs=(0, 0.15), tfs=5.5, tc=CE_LOGIC)

# Total latency annotation
t2iq_right = t2iq_left + BW - 0.5
hw_block(t2iq_right + 0.4, sy2, 2.8, BH,
         '395 cycles\n= 3.95 μs', '→ 253.2 kHz',
         fc='#ede7f6', ec=CE_FPGA, fs=8, sub_fs=7, bold=True, lw=1.5, sub_color=CE_FPGA)

# Pipeline DSP usage annotation
ax.text(pipe_x + pipe_w/2, pipe_y + 0.3,
        'Resources: 11k LUT (0.93%) | 420 DSP48 (6.14%) | Q16.16 Fixed-Point',
        ha='center', fontsize=6, color='#777', style='italic', zorder=4)

# ── Data inputs to pipeline ──
# Encoder -> FK
data_arrow(7.2, 12.15, pipe_x + 0.3, sy + BH/2, CE_IO, 1.0,
           '$q_i$ (Q16.16)', tofs=(0, 0.15), tfs=5.5, tc=CE_IO)

# Register file -> Impedance (xd, Ks, Ds)
data_arrow(3.3, 10.25, pipe_x + 0.3 + 2*(BW + 0.4) - 0.5, sy + BH/2,
           CE_BUS, 0.8, '$x_d$, $K_s$, $D_s$', tofs=(0, 0.15), tfs=5.5, tc=CE_BUS)

# BRAM -> FK (DH params, sin/cos)
data_arrow(3.3, 8.25, pipe_x + 0.3, sy + BH*0.3, CE_BRAM, 0.8,
           'DH, sin/cos', tofs=(0, -0.2), tfs=5.5, tc=CE_BRAM, va='top')


# ══════════════════════════════════════════════════════════════
# Clock Domain Crossing
# ══════════════════════════════════════════════════════════════
cdc_x = 19.0
cdc_y = 4.8
hw_block(cdc_x, cdc_y, 2.5, 0.8, 'CDC (2-FF Sync)',
         fc=C_CLK, ec=CE_CLK, fs=7, bold=False, lw=0.8)
ax.text(cdc_x + 1.25, cdc_y - 0.15, '100 MHz → 36.864 MHz',
        ha='center', fontsize=5, color=CE_CLK, style='italic', zorder=4)

# Pipeline output -> CDC
data_arrow(t2iq_left + (BW-0.5)/2, sy2, cdc_x + 1.25, cdc_y + 0.8,
           CE_LOGIC, 1.0, '$iq_{ref}$[6]\n(signed 16-bit)',
           tofs=(0.8, 0), tfs=5.5, tc=CE_LOGIC)


# ══════════════════════════════════════════════════════════════
# FOC Array (right side, foc_clk domain)
# ══════════════════════════════════════════════════════════════
foc_x = 22.0
foc_w = 9.0
foc_y = 2.5
foc_h = 10.3

foc_boundary = FancyBboxPatch((foc_x, foc_y), foc_w, foc_h,
                               boxstyle="round,pad=0.1",
                               facecolor='#f5fff5', edgecolor=CE_FOC,
                               linewidth=1.2, linestyle=':', zorder=0, alpha=0.5)
ax.add_patch(foc_boundary)
ax.text(foc_x + foc_w/2, foc_y + foc_h - 0.2,
        'FOC × 6 Parallel  (foc_clk = 36.864 MHz)',
        ha='center', fontsize=8, fontweight='bold', color=CE_FOC, zorder=4)

# Draw 6 FOC channels (2 rows × 3 columns)
foc_ch_w = 2.5
foc_ch_h = 3.2
foc_gap_x = 0.35
foc_gap_y = 0.5

for row in range(2):
    for col in range(3):
        i = row * 3 + col
        cx = foc_x + 0.3 + col * (foc_ch_w + foc_gap_x)
        cy = foc_y + 0.3 + (1 - row) * (foc_ch_h + foc_gap_y)

        ch_box = FancyBboxPatch((cx, cy), foc_ch_w, foc_ch_h,
                                 boxstyle="round,pad=0.04",
                                 facecolor=C_FOC, edgecolor=CE_FOC,
                                 linewidth=0.8, zorder=3)
        ax.add_patch(ch_box)
        ax.text(cx + foc_ch_w/2, cy + foc_ch_h - 0.18,
                f'Ch.{i}  (Joint {i})', ha='center', fontsize=6,
                fontweight='bold', color=CE_FOC, zorder=4)

        # Internal sub-blocks
        sub_names = ['Clarke', 'Park', 'PI×2', 'Inv.Park', 'SVPWM']
        sub_w = foc_ch_w - 0.2
        for j, sname in enumerate(sub_names):
            sy_sub = cy + foc_ch_h - 0.55 - j * 0.48
            sub_box = FancyBboxPatch((cx + 0.1, sy_sub), sub_w, 0.38,
                                      boxstyle="round,pad=0.02",
                                      facecolor='#c8e6c9', edgecolor='#66bb6a',
                                      linewidth=0.3, zorder=4)
            ax.add_patch(sub_box)
            ax.text(cx + 0.1 + sub_w/2, sy_sub + 0.19, sname,
                    ha='center', va='center', fontsize=5, color='#2e7d32', zorder=5)

# CDC -> FOC array
data_arrow(cdc_x + 2.5, cdc_y + 0.4, foc_x, foc_y + foc_h/2,
           CE_FOC, 1.2, '$iq_{ref}$[6]', tofs=(0, 0.15), tfs=6, tc=CE_FOC)

# FOC rate annotation
ax.text(foc_x + foc_w - 0.2, foc_y + 0.25,
        '18 kHz per channel', ha='right', fontsize=6,
        color=CE_FOC, fontweight='bold', style='italic', zorder=4)


# ══════════════════════════════════════════════════════════════
# External: ADC (input to FOC)
# ══════════════════════════════════════════════════════════════
hw_block(22.5, 13.8, 3.5, 1.0, 'Current ADC × 6',
         '12-bit, 3-phase', fc=C_IO, ec=CE_IO, fs=7.5, sub_fs=6)
hw_block(26.5, 13.8, 3.5, 1.0, 'Shunt Resistor\n× 18',
         fc=C_MOTOR, ec=CE_MOTOR, fs=7, sub_fs=5.5)

data_arrow(24.25, 13.8, 24.25, 12.8, CE_IO, 1.0,
           'SPI ($I_a, I_b, I_c$)', tofs=(1.3, 0), tfs=5.5, tc=CE_IO)
data_arrow(26.5, 14.3, 26.0, 14.3, CE_MOTOR, 0.8)

# Encoder -> FOC (phi)
hw_block(27.0, 11.8, 3.0, 1.0, 'Encoder × 6',
         '12-bit φ (mech.)', fc=C_MOTOR, ec=CE_MOTOR, fs=7.5, sub_fs=5.5)
data_arrow(28.5, 11.8, 28.5, 10.0, CE_IO, 0.8,
           'SPI (φ)', tofs=(0.5, 0), tfs=5.5, tc=CE_IO)

# Note: same encoders feed both FK and FOC
ax.annotate('', xy=(7.2, 12.8), xytext=(5.6, 13.8),
            arrowprops=dict(arrowstyle='->', color=CE_IO, lw=0.8), zorder=2)


# ══════════════════════════════════════════════════════════════
# External: PWM Output & Motor Driver
# ══════════════════════════════════════════════════════════════
hw_block(22.0, 0.0, 4.0, 1.2, 'Gate Driver × 6',
         'Half-Bridge, 18 PWM signals',
         fc=C_IO, ec=CE_IO, fs=8)
hw_block(27.0, 0.0, 4.0, 1.2, 'BLDC Motor × 6',
         '6-DOF Slave Arm Joints',
         fc=C_MOTOR, ec=CE_MOTOR, fs=8)

# FOC -> PWM -> Driver -> Motor
data_arrow(foc_x + foc_w/2, foc_y, 24.0, 1.2, CE_FOC, 1.2,
           'PWM × 18\n(GPIO)', tofs=(0.8, 0), tfs=6, tc=CE_FOC)
data_arrow(26.0, 0.6, 27.0, 0.6, CE_MOTOR, 1.2,
           '3-phase\npower', tofs=(0, 0.15), tfs=5.5, tc=CE_MOTOR)

# Motor -> Shunt -> ADC (feedback)
data_arrow(29.0, 1.2, 28.25, 13.8, '#999', 0.8,
           'Phase current\n(analog)', tofs=(0.5, 0), tfs=5, tc='#999', style='->')


# ══════════════════════════════════════════════════════════════
# Force Feedback Path (FOC iq_actual -> Host -> Master)
# ══════════════════════════════════════════════════════════════
# iq_actual from FOC -> registers -> PCIe -> CPU
data_arrow(foc_x + foc_w, foc_y + foc_h - 1.5, 31.0, 16.0,
           CE_FPGA, 0.8, style='->')
data_arrow(31.0, 16.0, 31.0, 19.25, '#cc4444', 0.8, style='->')
data_arrow(31.0, 19.25, 8.0, 19.25, '#cc4444', 0.8, style='->')
ax.text(31.3, 17.5, '$iq_{actual}$[6]\n→ Force\nFeedback',
        ha='left', fontsize=6, color='#cc4444', zorder=4,
        bbox=dict(facecolor='white', edgecolor='none', pad=1))
# Label: PCIe DMA Read
ax.text(19, 19.5, 'PCIe DMA Read → ROS 2 → Master Arm Force Feedback',
        ha='center', fontsize=6, color='#cc4444', style='italic', zorder=4)


# ══════════════════════════════════════════════════════════════
# Data flow summary arrows (thick, colored)
# ══════════════════════════════════════════════════════════════
# Title
ax.text(16, 20.5,
        'Hardware Data Flow: FPGA-Based Impedance Control with 6-Channel Parallel FOC',
        ha='center', fontsize=13, fontweight='bold', zorder=4)
ax.text(16, 19.9,
        'Bilateral Teleoperation System  —  7-DOF Master / 6-DOF Slave',
        ha='center', fontsize=9, color='#666', zorder=4)

# ══════════════════════════════════════════════════════════════
# Legend
# ══════════════════════════════════════════════════════════════
leg_y = -1.2
leg_items = [
    (C_CPU, CE_CPU, 'CPU / Host'),
    (C_MEM, CE_MEM, 'Memory'),
    (C_BRAM, CE_BRAM, 'On-chip BRAM'),
    (C_LOGIC, CE_LOGIC, 'Control Logic'),
    (C_FOC, CE_FOC, 'FOC Drive'),
    (C_IO, CE_IO, 'I/O Interface'),
    (C_MOTOR, CE_MOTOR, 'External HW'),
    (C_BUS, CE_BUS, 'Bus / Register'),
]
total = len(leg_items) * 3.5
lx_start = 16 - total/2
for i, (fc, ec, label) in enumerate(leg_items):
    lx = lx_start + i * 3.5
    b = FancyBboxPatch((lx, leg_y), 0.4, 0.3, boxstyle="round,pad=0.02",
                        facecolor=fc, edgecolor=ec, linewidth=0.8, zorder=3)
    ax.add_patch(b)
    ax.text(lx + 0.55, leg_y + 0.15, label, ha='left', va='center',
            fontsize=6.5, color='#444', zorder=4)


# ══════════════════════════════════════════════════════════════
# Save
# ══════════════════════════════════════════════════════════════
out_png = 'D:/robot/impedance_control/figures/fig_hw_dataflow.png'
out_pdf = 'D:/robot/impedance_control/figures/fig_hw_dataflow.pdf'
plt.savefig(out_png, dpi=300, bbox_inches='tight', facecolor='white', pad_inches=0.2)
plt.savefig(out_pdf, dpi=300, bbox_inches='tight', facecolor='white', pad_inches=0.2)
plt.close()
print(f'Saved: {out_png}')
print(f'Saved: {out_pdf}')
