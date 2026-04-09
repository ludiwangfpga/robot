#!/usr/bin/env python3
"""
Generate a publication-quality system architecture figure for
FPGA-based bilateral teleoperation impedance control system.
Suitable for IEEE RA-L / ICRA submission.
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import matplotlib.patheffects as pe
from matplotlib.font_manager import FontProperties
import os

# ── Global style ──────────────────────────────────────────────────────
plt.rcParams.update({
    'font.family': ['Microsoft YaHei', 'SimHei', 'SimSun', 'DejaVu Sans'],
    'font.size': 7.5,
    'axes.unicode_minus': False,
    'pdf.fonttype': 42,      # TrueType in PDF
    'ps.fonttype': 42,
    'mathtext.fontset': 'stix',
})
# Serif font for English labels where desired
_serif = FontProperties(family='Times New Roman', size=7.5)

# ── Colour palette ────────────────────────────────────────────────────
C_HOST_BG    = '#D5ECD4'   # light green – host / PS
C_PS_BG      = '#C8E6C9'
C_PL_BG      = '#DCEEFB'   # light blue – PL fabric
C_PL_INNER   = '#C6DDEF'
C_PIPELINE   = '#B3D4F0'
C_FOC_BG     = '#BDD7EE'
C_HW_BG      = '#FDEBD0'   # light orange – hardware
C_HW_INNER   = '#F5CBA7'
C_BLOCK      = '#FFFFFF'   # white inner blocks
C_HEADER     = '#2C3E50'   # dark text
C_ANNOT      = '#C0392B'   # red annotation

fig, ax = plt.subplots(figsize=(10.5, 7.2))
ax.set_xlim(0, 10.5)
ax.set_ylim(0, 7.2)
ax.set_aspect('equal')
ax.axis('off')

# ======================================================================
# Helper functions
# ======================================================================
def box(x, y, w, h, color, edgecolor='#555555', lw=0.8, zorder=2,
        radius=0.08, style='round,pad=0.02'):
    """Draw a rounded rectangle and return (patch, center_x, center_y)."""
    b = FancyBboxPatch((x, y), w, h, boxstyle=style,
                       facecolor=color, edgecolor=edgecolor,
                       linewidth=lw, zorder=zorder)
    ax.add_patch(b)
    return b, x + w/2, y + h/2

def label(x, y, text, fontsize=7, fontweight='normal', color='black',
          ha='center', va='center', zorder=5, **kw):
    ax.text(x, y, text, fontsize=fontsize, fontweight=fontweight,
            color=color, ha=ha, va=va, zorder=zorder, **kw)

def arrow(x1, y1, x2, y2, color='black', lw=0.9, style='->', zorder=4,
          linestyle='-', connectionstyle='arc3,rad=0', shrinkA=2, shrinkB=2):
    a = FancyArrowPatch((x1, y1), (x2, y2),
                        arrowstyle=style, color=color,
                        linewidth=lw, linestyle=linestyle, zorder=zorder,
                        connectionstyle=connectionstyle,
                        mutation_scale=8, shrinkA=shrinkA, shrinkB=shrinkB)
    ax.add_patch(a)
    return a

# ======================================================================
# 1. HOST SIDE (left column)
# ======================================================================
host_x, host_y, host_w, host_h = 0.15, 1.4, 1.95, 5.0
box(host_x, host_y, host_w, host_h, C_HOST_BG, lw=1.2)
label(host_x + host_w/2, host_y + host_h - 0.22,
      'Host PC  /  上位机', fontsize=8, fontweight='bold', color=C_HEADER)

# Sub-blocks inside host
bx = host_x + 0.15
bw = host_w - 0.3

box(bx, 5.35, bw, 0.55, C_BLOCK, lw=0.6)
label(bx + bw/2, 5.68, 'Ubuntu 22.04', fontsize=6.5, fontweight='bold')
label(bx + bw/2, 5.47, 'ROS2 Humble Node', fontsize=6)

box(bx, 4.55, bw, 0.60, C_BLOCK, lw=0.6)
label(bx + bw/2, 4.90, 'Trajectory Planning', fontsize=6.5, fontweight='bold')
label(bx + bw/2, 4.72, '轨迹规划', fontsize=6, color='#555')
label(bx + bw/2, 4.55+0.03, 'Joint Cmd Topics', fontsize=5.5, color='#777', va='bottom')

box(bx, 3.70, bw, 0.62, C_BLOCK, lw=0.6)
label(bx + bw/2, 4.06, 'PYNQ Overlay', fontsize=6.5, fontweight='bold')
label(bx + bw/2, 3.88, 'Manager / 管理器', fontsize=6, color='#555')
label(bx + bw/2, 3.70+0.03, '.bit + .hwh download', fontsize=5.5, color='#777', va='bottom')

box(bx, 2.90, bw, 0.55, C_BLOCK, lw=0.6)
label(bx + bw/2, 3.22, 'Monitoring', fontsize=6.5, fontweight='bold')
label(bx + bw/2, 3.04, '& Logging / 监控日志', fontsize=6, color='#555')

# Host → KV260 arrow (AXI)
arrow(host_x + host_w, 4.0, 2.55, 4.0, lw=1.2, style='->')
label(2.10 + 0.20, 4.18, 'AXI', fontsize=6, fontweight='bold', color='#2471A3')

# ======================================================================
# 2. KV260 FPGA SoM (centre)
# ======================================================================
kv_x, kv_y, kv_w, kv_h = 2.55, 0.45, 5.35, 6.35
box(kv_x, kv_y, kv_w, kv_h, '#E8E8E8', lw=1.4, edgecolor='#333')
label(kv_x + kv_w/2, kv_y + kv_h - 0.20,
      'Kria KV260 FPGA SoM', fontsize=9, fontweight='bold', color=C_HEADER)

# ── 2a. PS block ──────────────────────────────────────────────────────
ps_x, ps_y, ps_w, ps_h = 2.70, 5.15, 2.15, 1.30
box(ps_x, ps_y, ps_w, ps_h, C_PS_BG, lw=1.0)
label(ps_x + ps_w/2, ps_y + ps_h - 0.18,
      'PS (ARM Cortex-A53)', fontsize=7, fontweight='bold', color=C_HEADER)

box(ps_x+0.10, 5.55, ps_w-0.20, 0.35, C_BLOCK, lw=0.5)
label(ps_x + ps_w/2, 5.73, 'PYNQ Runtime', fontsize=6.5, fontweight='bold')

box(ps_x+0.10, 5.18, ps_w-0.20, 0.30, C_BLOCK, lw=0.5)
label(ps_x + ps_w/2, 5.33, 'AXI-Lite Register Bridge', fontsize=6)

# PS → PL arrow
arrow(ps_x + ps_w/2, ps_y, ps_x + ps_w/2, 5.05, lw=1.0, style='->')
label(ps_x + ps_w + 0.05, ps_y - 0.10, 'AXI-Lite', fontsize=5.5,
      color='#2471A3', ha='left')

# ── 2b. PL block ──────────────────────────────────────────────────────
pl_x, pl_y, pl_w, pl_h = 2.70, 0.60, 5.05, 4.40
box(pl_x, pl_y, pl_w, pl_h, C_PL_BG, lw=1.0)
label(pl_x + pl_w/2, pl_y + pl_h - 0.17,
      'PL (Programmable Logic / 可编程逻辑)', fontsize=7.5, fontweight='bold',
      color=C_HEADER)

# Register Bus
box(pl_x+0.10, 4.50, pl_w-0.20, 0.30, '#E8F8F5', lw=0.6)
label(pl_x + pl_w/2, 4.65, 'Register Bus  (AXI-Lite Slave)', fontsize=6.5)

# ── Impedance Control Pipeline ────────────────────────────────────────
pip_x, pip_y, pip_w, pip_h = pl_x+0.10, 2.15, pl_w-0.20, 2.25
box(pip_x, pip_y, pip_w, pip_h, C_PL_INNER, lw=0.8, edgecolor='#4A90D9')
label(pip_x + pip_w/2, pip_y + pip_h - 0.15,
      'Impedance Control Pipeline  /  阻抗控制流水线',
      fontsize=7, fontweight='bold', color='#1A5276')

# Pipeline stages – arranged in two rows
stages = [
    ('FK Engine\n+ Analytical Jacobian\n解析雅可比', '382+42 cyc\n6×CORDIC'),
    ('Velocity\nEstimator\n速度估计', '3 cyc'),
    ('Impedance\nController\n阻抗控制器', '2 cyc'),
    ('J$^T$ Force\nMapper', '6 cyc\n6×6 mat-vec'),
    ('Torque→Iq\nConverter', '2 cyc'),
    ('iq Predictor\n线性外推', 'linear\nextrap.'),
]

sx_start = pip_x + 0.10
sy = 3.30
sw = 0.73
sh = 0.85
gap = 0.08

for i, (name, note) in enumerate(stages):
    sx = sx_start + i * (sw + gap)
    box(sx, sy, sw, sh, C_BLOCK, lw=0.5)
    label(sx + sw/2, sy + sh/2 + 0.10, name, fontsize=5, fontweight='bold',
          color='#1A3C5E', linespacing=1.1)
    label(sx + sw/2, sy + 0.10, note, fontsize=4.5, color='#777',
          linespacing=1.05)
    # arrows between stages
    if i > 0:
        arrow(sx - gap, sy + sh/2, sx, sy + sh/2, lw=0.7, style='->')

# Adaptive Impedance (parallel with FK)  – sits below FK block
box(sx_start, 2.25, sw*2 + gap, 0.55, '#FFF9E6', lw=0.5, edgecolor='#D4AC0D')
label(sx_start + (sw*2+gap)/2, 2.60, 'Adaptive Impedance (RLS)', fontsize=5.5,
      fontweight='bold', color='#7D6608')
label(sx_start + (sw*2+gap)/2, 2.38, '自适应阻抗  48 cyc  (parallel w/ FK)',
      fontsize=5, color='#999')

# Dashed arrow from Adaptive to Impedance Controller
arrow(sx_start + sw*2 + gap, 2.52,
      sx_start + 2*(sw+gap) + sw/2, sy,
      lw=0.7, style='->', linestyle='--', color='#D4AC0D')

# ── FOC Array ─────────────────────────────────────────────────────────
foc_x, foc_y, foc_w, foc_h = pl_x+0.10, 0.70, pl_w-0.20, 1.30
box(foc_x, foc_y, foc_w, foc_h, C_FOC_BG, lw=0.8, edgecolor='#4A90D9')
label(foc_x + foc_w/2, foc_y + foc_h - 0.14,
      'FOC Array  /  磁场定向控制阵列  (6 channels ×36 kHz)',
      fontsize=6.5, fontweight='bold', color='#1A5276')

foc_stages = ['Clarke\nTransform', 'Park\nTransform', 'PI\nControllers',
              'Inv. Park\nTransform', 'SVPWM']
fsx = foc_x + 0.25
fsy = 0.80
fsw = 0.85
fsh = 0.55
fgap = 0.10
for i, s in enumerate(foc_stages):
    fx = fsx + i * (fsw + fgap)
    box(fx, fsy, fsw, fsh, C_BLOCK, lw=0.5)
    label(fx + fsw/2, fsy + fsh/2, s, fontsize=5.5, fontweight='bold',
          color='#1A3C5E', linespacing=1.1)
    if i > 0:
        arrow(fx - fgap, fsy + fsh/2, fx, fsy + fsh/2, lw=0.6, style='->')

# Pipeline → FOC arrow
arrow(pip_x + pip_w/2, pip_y, foc_x + foc_w/2, foc_y + foc_h,
      lw=1.0, style='->')
label(pip_x + pip_w/2 + 0.55, 1.97, '$i_q^*$', fontsize=7, color='#2471A3')

# Register Bus → Pipeline arrow
arrow(pl_x + pl_w/2, 4.50, pl_x + pl_w/2, pip_y + pip_h,
      lw=0.8, style='->')

# ── Clock domain annotations (dashed boxes) ──────────────────────────
# sys_clk box
clk_sys = mpatches.FancyBboxPatch(
    (pip_x - 0.02, pip_y - 0.02), pip_w + 0.04, pip_h + 0.04 + 0.45,
    boxstyle='round,pad=0.03', facecolor='none',
    edgecolor='#8E44AD', linewidth=0.8, linestyle='--', zorder=3)
ax.add_patch(clk_sys)
label(pip_x + pip_w - 0.05, pip_y + pip_h + 0.32, 'sys_clk 100 MHz',
      fontsize=5.5, fontweight='bold', color='#8E44AD', ha='right')

# foc_clk box
clk_foc = mpatches.FancyBboxPatch(
    (foc_x - 0.02, foc_y - 0.02), foc_w + 0.04, foc_h + 0.04,
    boxstyle='round,pad=0.03', facecolor='none',
    edgecolor='#D35400', linewidth=0.8, linestyle='--', zorder=3)
ax.add_patch(clk_foc)
label(foc_x + foc_w - 0.05, foc_y + foc_h + 0.04, 'foc_clk 73.728 MHz',
      fontsize=5.5, fontweight='bold', color='#D35400', ha='right')

# ======================================================================
# 3. HARDWARE INTERFACE (right column)
# ======================================================================
hw_x, hw_y, hw_w, hw_h = 8.15, 0.45, 2.15, 6.35
box(hw_x, hw_y, hw_w, hw_h, C_HW_BG, lw=1.2)
label(hw_x + hw_w/2, hw_y + hw_h - 0.22,
      'Hardware  /  硬件接口', fontsize=8, fontweight='bold', color=C_HEADER)

hbx = hw_x + 0.12
hbw = hw_w - 0.24

box(hbx, 5.50, hbw, 0.80, C_HW_INNER, lw=0.6)
label(hbx + hbw/2, 5.98, '6× BLDC Motor Drivers', fontsize=6, fontweight='bold')
label(hbx + hbw/2, 5.78, '无刷电机驱动器', fontsize=5.5, color='#555')
label(hbx + hbw/2, 5.58, 'PWM Outputs', fontsize=5.5, color='#777')

box(hbx, 4.40, hbw, 0.85, C_HW_INNER, lw=0.6)
label(hbx + hbw/2, 4.93, '6× Incremental Encoders', fontsize=6, fontweight='bold')
label(hbx + hbw/2, 4.73, '增量编码器', fontsize=5.5, color='#555')
label(hbx + hbw/2, 4.50, 'Position Feedback', fontsize=5.5, color='#777')

box(hbx, 3.25, hbw, 0.85, C_HW_INNER, lw=0.6)
label(hbx + hbw/2, 3.78, '6× Current Sensors', fontsize=6, fontweight='bold')
label(hbx + hbw/2, 3.58, 'ADC / 电流传感器', fontsize=5.5, color='#555')
label(hbx + hbw/2, 3.35, 'Phase Currents $i_a, i_b$', fontsize=5.5, color='#777')

box(hbx, 1.60, hbw, 1.30, C_HW_INNER, lw=0.8, edgecolor='#A04000')
label(hbx + hbw/2, 2.50, '6-DOF EPDobot', fontsize=7, fontweight='bold',
      color='#A04000')
label(hbx + hbw/2, 2.27, 'Robot Arm', fontsize=6.5, fontweight='bold',
      color='#A04000')
label(hbx + hbw/2, 2.04, '机器人手臂', fontsize=6, color='#555')
label(hbx + hbw/2, 1.78, 'Q16.16 Fixed-Point', fontsize=5.5, color='#777')

# ── Arrows: KV260 PL ↔ Hardware ──────────────────────────────────────
# FOC → Motor Drivers (PWM)
arrow(pl_x + pl_w, 1.10, hw_x, 5.90, lw=1.2, style='->', color='#2471A3')
label((pl_x + pl_w + hw_x)/2, 3.80, 'PWM', fontsize=6, fontweight='bold',
      color='#2471A3', rotation=68, ha='center',
      bbox=dict(boxstyle='round,pad=0.08', facecolor='white', edgecolor='none', alpha=0.85))

# Encoders → Pipeline (position feedback)
arrow(hw_x, 4.82, pl_x + pl_w, 3.70, lw=1.1, style='->', color='#27AE60')
label((pl_x + pl_w + hw_x)/2, 4.40, '$\\theta$ (pos)',
      fontsize=5.5, fontweight='bold', color='#27AE60', ha='center',
      rotation=-20,
      bbox=dict(boxstyle='round,pad=0.08', facecolor='white', edgecolor='none', alpha=0.85))

# Current sensors → FOC (current feedback)
arrow(hw_x, 3.50, pl_x + pl_w, 0.95, lw=1.1, style='->', color='#E67E22')
label((pl_x + pl_w + hw_x)/2, 2.45, '$i_a, i_b$',
      fontsize=6, fontweight='bold', color='#E67E22', ha='center',
      rotation=-60,
      bbox=dict(boxstyle='round,pad=0.08', facecolor='white', edgecolor='none', alpha=0.85))

# Motor drivers → Robot Arm
arrow(hbx + hbw/2, 5.50, hbx + hbw/2, 2.95, lw=0.8, style='->')
# Robot Arm → Encoders (feedback loop)
arrow(hbx + hbw/2 + 0.30, 2.95, hbx + hbw/2 + 0.30, 4.40,
      lw=0.7, style='->', linestyle='--', color='#777')

# ======================================================================
# 4. KEY ANNOTATIONS (bottom)
# ======================================================================
ann_y = 0.12
annotations = [
    'Pipeline Latency: 395 cycles = 3.95 μs @ 100 MHz',
    'Control BW: 253 kHz',
    'FOC: 36 kHz/ch',
    'LUT: 11,003 (0.93%)   DSP: 420 (6.14%)',
]
ann_text = '    |    '.join(annotations)
label(5.25, ann_y, ann_text, fontsize=5.8, fontweight='bold',
      color=C_ANNOT, ha='center',
      bbox=dict(boxstyle='round,pad=0.15', facecolor='#FFF5F5',
                edgecolor=C_ANNOT, linewidth=0.6))

# ======================================================================
# Save
# ======================================================================
out_dir = r'D:\papercode\figures'
os.makedirs(out_dir, exist_ok=True)

fig.tight_layout(pad=0.3)
fig.savefig(os.path.join(out_dir, 'fig_system_architecture.png'),
            dpi=300, bbox_inches='tight', pad_inches=0.08)
fig.savefig(os.path.join(out_dir, 'fig_system_architecture.pdf'),
            dpi=300, bbox_inches='tight', pad_inches=0.08)
plt.close(fig)
print('Done – saved PNG and PDF to', out_dir)
