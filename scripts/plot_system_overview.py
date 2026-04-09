#!/usr/bin/env python3
"""
Bilateral Teleoperation System Overview - Abstract Architecture Diagram
High-level conceptual diagram without specific algorithm names.
"""
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

fig, ax = plt.subplots(1, 1, figsize=(16, 7))
ax.set_xlim(-0.5, 15.5)
ax.set_ylim(0, 7)
ax.set_aspect('equal')
ax.axis('off')

# ── Color scheme ──
C_HUMAN   = '#FFF3E0'  # warm cream - human
C_MASTER  = '#E3F2FD'  # light blue - master
C_NET     = '#F3E5F5'  # light purple - network
C_HOST    = '#E8F5E9'  # light green - host pc
C_FPGA    = '#FFF8E1'  # light yellow - fpga
C_SLAVE   = '#FFEBEE'  # light red - slave
C_ENV     = '#F1F8E9'  # light lime - environment

C_BORDER_M = '#1565C0'
C_BORDER_N = '#7B1FA2'
C_BORDER_H = '#2E7D32'
C_BORDER_F = '#F57F17'
C_BORDER_S = '#C62828'

def rounded_box(x, y, w, h, label, facecolor, edgecolor, fontsize=10,
                lw=2.0, sublabel=None, sublabel_size=7.5):
    box = FancyBboxPatch((x, y), w, h,
                         boxstyle="round,pad=0.12",
                         facecolor=facecolor, edgecolor=edgecolor,
                         linewidth=lw, zorder=2)
    ax.add_patch(box)
    if sublabel:
        ax.text(x + w/2, y + h/2 + 0.15, label,
                ha='center', va='center', fontsize=fontsize,
                fontweight='bold', color=edgecolor, zorder=3)
        ax.text(x + w/2, y + h/2 - 0.22, sublabel,
                ha='center', va='center', fontsize=sublabel_size,
                color='#555555', zorder=3)
    else:
        ax.text(x + w/2, y + h/2, label,
                ha='center', va='center', fontsize=fontsize,
                fontweight='bold', color=edgecolor, zorder=3)

def arrow(x1, y1, x2, y2, label='', color='#333', lw=1.8,
          style='->', label_offset=(0, 0.18), fontsize=7.5,
          label_color='#333'):
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle=style, color=color,
                                lw=lw, shrinkA=3, shrinkB=3),
                zorder=4)
    if label:
        mx = (x1 + x2) / 2 + label_offset[0]
        my = (y1 + y2) / 2 + label_offset[1]
        ax.text(mx, my, label, ha='center', va='center',
                fontsize=fontsize, color=label_color, zorder=5)

# ═══════════════════════════════════════════════════════
#  Layout (left to right):
#  Operator | Master Arm | Network | Host PC | FPGA | Slave Arm | Environment
# ═══════════════════════════════════════════════════════

# ── Row heights ──
y_top = 4.2    # command path (top)
y_bot = 1.8    # feedback path (bottom)
y_mid = 3.0    # center line
bh = 1.6       # box height

# ── 1. Human Operator ──
rounded_box(0.5, y_mid - bh/2, 1.6, bh, 'Operator', C_HUMAN, '#E65100',
            fontsize=11, sublabel='(Human)')

# ── 2. Master Arm ──
rounded_box(2.7, y_mid - bh/2, 1.8, bh, 'Master Arm', C_MASTER, C_BORDER_M,
            fontsize=11, sublabel='7-DOF')

# ── 3. Network (ROS 2) ──
rounded_box(5.1, y_mid - bh/2, 2.0, bh, 'Network', C_NET, C_BORDER_N,
            fontsize=11, sublabel='ROS 2 / Ethernet')

# ── 4. Host PC ──
rounded_box(7.7, y_mid - bh/2, 1.8, bh, 'Host PC', C_HOST, C_BORDER_H,
            fontsize=11, sublabel='CPU + Memory')

# ── 5. FPGA ──
rounded_box(10.1, y_mid - bh/2, 2.2, bh, 'FPGA', C_FPGA, C_BORDER_F,
            fontsize=12, sublabel='Arm Controller\n+ Motor Driver',
            sublabel_size=8)

# ── 6. Slave Arm ──
rounded_box(12.9, y_mid - bh/2, 1.6, bh, 'Slave Arm', C_SLAVE, C_BORDER_S,
            fontsize=11, sublabel='6-DOF')

# ═══════════════════════════════════════════════════════
#  Arrows - Command path (top, left→right)
# ═══════════════════════════════════════════════════════
y_cmd = y_mid + 0.25
y_fb  = y_mid - 0.25

# Operator → Master
arrow(1.8, y_cmd, 2.3, y_cmd, 'Motion', '#E65100', label_offset=(0, 0.2))
# Master → Network
arrow(4.1, y_cmd, 4.7, y_cmd, 'Pose cmd', C_BORDER_N, label_offset=(0, 0.2))
# Network → Host PC
arrow(6.7, y_cmd, 7.3, y_cmd, '', C_BORDER_H)
# Host PC → FPGA
arrow(9.1, y_cmd, 9.7, y_cmd, 'PCIe', C_BORDER_F, label_offset=(0, 0.2))
# FPGA → Slave
arrow(11.7, y_cmd, 12.3, y_cmd, 'Torque', C_BORDER_S, label_offset=(0, 0.2))

# ═══════════════════════════════════════════════════════
#  Arrows - Feedback path (bottom, right→left)
# ═══════════════════════════════════════════════════════
# Slave → FPGA
arrow(12.3, y_fb, 11.7, y_fb, 'Sensor', C_BORDER_S,
      label_offset=(0, -0.22))
# FPGA → Host PC
arrow(9.7, y_fb, 9.1, y_fb, 'PCIe', C_BORDER_F,
      label_offset=(0, -0.22))
# Host PC → Network
arrow(7.3, y_fb, 6.7, y_fb, '', C_BORDER_H)
# Network → Master
arrow(4.7, y_fb, 4.1, y_fb, 'Force fb', C_BORDER_N,
      label_offset=(0, -0.22))
# Master → Operator
arrow(2.3, y_fb, 1.8, y_fb, 'Haptic', '#E65100',
      label_offset=(0, -0.22))

# ═══════════════════════════════════════════════════════
#  Labels for paths
# ═══════════════════════════════════════════════════════
ax.text(7.0, y_mid + 1.35, 'Command Path',
        ha='center', va='center', fontsize=9, fontstyle='italic',
        color='#555')
ax.annotate('', xy=(11.5, y_mid + 1.15), xytext=(2.5, y_mid + 1.15),
            arrowprops=dict(arrowstyle='->', color='#999', lw=1.0))

ax.text(7.0, y_mid - 1.35, 'Feedback Path',
        ha='center', va='center', fontsize=9, fontstyle='italic',
        color='#555')
ax.annotate('', xy=(2.5, y_mid - 1.15), xytext=(11.5, y_mid - 1.15),
            arrowprops=dict(arrowstyle='->', color='#999', lw=1.0))

# ═══════════════════════════════════════════════════════
#  Latency annotations
# ═══════════════════════════════════════════════════════
# Brace-like annotation for FPGA computation
ax.annotate('', xy=(9.7, y_mid + bh/2 + 0.15),
            xytext=(11.7, y_mid + bh/2 + 0.15),
            arrowprops=dict(arrowstyle='-', color=C_BORDER_F, lw=1.2))
ax.text(10.7, y_mid + bh/2 + 0.38,
        'Computation: 31.7 μs',
        ha='center', va='center', fontsize=7.5,
        color=C_BORDER_F, fontweight='bold',
        bbox=dict(boxstyle='round,pad=0.15', fc='white', ec=C_BORDER_F,
                  lw=0.8, alpha=0.9))

# Network latency
ax.text(5.7, y_mid + bh/2 + 0.38,
        'Communication: ~1.1 ms',
        ha='center', va='center', fontsize=7.5,
        color=C_BORDER_N, fontweight='bold',
        bbox=dict(boxstyle='round,pad=0.15', fc='white', ec=C_BORDER_N,
                  lw=0.8, alpha=0.9))

# ── Title ──
ax.text(7.0, 6.55,
        'Bilateral Teleoperation System Architecture',
        ha='center', va='center', fontsize=15, fontweight='bold',
        color='#222')
ax.text(7.0, 6.15,
        'FPGA-Accelerated Low-Latency Control with Force Feedback',
        ha='center', va='center', fontsize=10, color='#666')

# ── Bottom note ──
ax.text(7.0, 0.35,
        'Round-trip: ~2.3 ms  |  Computation: 31.7 μs (1.4%)  |  Communication: ~2.2 ms (98.6%)',
        ha='center', va='center', fontsize=8.5, color='#777',
        bbox=dict(boxstyle='round,pad=0.3', fc='#F5F5F5', ec='#CCC',
                  lw=0.8))

plt.tight_layout()

out_dir = 'D:/robot/impedance_control/figures'
fig.savefig(f'{out_dir}/fig_system_overview.png', dpi=300, bbox_inches='tight',
            facecolor='white')
fig.savefig(f'{out_dir}/fig_system_overview.pdf', bbox_inches='tight',
            facecolor='white')
print(f'Saved: {out_dir}/fig_system_overview.png')
print(f'Saved: {out_dir}/fig_system_overview.pdf')
plt.close()
