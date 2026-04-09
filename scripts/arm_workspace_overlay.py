"""
6-DOF 机械臂工作空间可视化 — 侧视图 (X-Z) + 俯视图 (X-Y)
包含: 简化连杆示意图、Monte Carlo 工作空间边界、尺寸标注、关节角范围
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch, Arc
from scipy.spatial import ConvexHull

# ── matplotlib 中文 & 学术风格 ──────────────────────────────────
plt.rcParams.update({
    "font.family": "SimHei",
    "axes.unicode_minus": False,
    "font.size": 10,
    "axes.labelsize": 12,
    "axes.titlesize": 14,
    "figure.dpi": 150,
    "savefig.dpi": 300,
})

# ── DH 参数 ─────────────────────────────────────────────────────
#       a(mm)   alpha(rad)      d(mm)   theta_min(deg)  theta_max(deg)
DH = np.array([
    [55,    np.pi / 2,   166,  -180,  160],
    [200,   0,             0,     0,  180],
    [56,    np.pi / 2,     0,   -80,   83],
    [0,    -np.pi / 2,   192,   -30,  305],
    [0,     np.pi / 2,     0,  -110,  110],
    [0,     0,            55,   -30,  305],
])

JOINT_RANGES_STR = [
    "[-180°, 160°]",
    "[0°, 180°]",
    "[-80°, 83°]",
    "[-30°, 305°]",
    "[-110°, 110°]",
    "[-30°, 305°]",
]

LINK_LABELS = {
    "d1": 166, "a1": 55, "a2": 200, "a3": 56, "d4": 192, "d6": 55,
}

N_SAMPLES = 200_000
np.random.seed(42)


# ── 正运动学 (向量化) ────────────────────────────────────────────
def dh_matrix(a, alpha, d, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    T = np.zeros((len(theta), 4, 4))
    T[:, 0, 0] = ct;  T[:, 0, 1] = -st * ca
    T[:, 0, 2] = st * sa;  T[:, 0, 3] = a * ct
    T[:, 1, 0] = st;  T[:, 1, 1] = ct * ca
    T[:, 1, 2] = -ct * sa;  T[:, 1, 3] = a * st
    T[:, 2, 1] = sa;  T[:, 2, 2] = ca;  T[:, 2, 3] = d
    T[:, 3, 3] = 1.0
    return T


def fk_batch(joint_angles_deg):
    N = joint_angles_deg.shape[0]
    T_acc = np.broadcast_to(np.eye(4), (N, 4, 4)).copy()
    for i in range(6):
        theta_i = np.deg2rad(joint_angles_deg[:, i])
        T_i = dh_matrix(DH[i, 0], DH[i, 1], DH[i, 2], theta_i)
        T_acc = np.einsum("nij,njk->nik", T_acc, T_i)
    return T_acc[:, :3, 3]


def fk_single(angles_deg):
    """单组关节角正运动学, 返回各关节位置 (7x3: base + 6 joints)."""
    T = np.eye(4)
    positions = [T[:3, 3].copy()]  # base
    for i in range(6):
        theta = np.deg2rad(angles_deg[i])
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(DH[i, 1]), np.sin(DH[i, 1])
        a, d = DH[i, 0], DH[i, 2]
        Ti = np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,   sa,       ca,      d     ],
            [0,   0,        0,       1     ],
        ])
        T = T @ Ti
        positions.append(T[:3, 3].copy())
    return np.array(positions)


# ── Monte Carlo 采样 ─────────────────────────────────────────────
print(f"Monte Carlo 采样 {N_SAMPLES} 点 ...")
theta_min = DH[:, 3]
theta_max = DH[:, 4]
samples = np.random.uniform(theta_min, theta_max, size=(N_SAMPLES, 6))

BATCH = 50_000
positions = []
for start in range(0, N_SAMPLES, BATCH):
    end = min(start + BATCH, N_SAMPLES)
    positions.append(fk_batch(samples[start:end]))
    print(f"  {end}/{N_SAMPLES}")
pos = np.vstack(positions)
x, y, z = pos[:, 0], pos[:, 1], pos[:, 2]

print(f"X: [{x.min():.1f}, {x.max():.1f}]")
print(f"Y: [{y.min():.1f}, {y.max():.1f}]")
print(f"Z: [{z.min():.1f}, {z.max():.1f}]")
print(f"Max reach: {np.sqrt(x**2 + y**2 + z**2).max():.1f} mm")

# ── Home position 各关节坐标 ─────────────────────────────────────
home_angles = [0, 0, 0, 0, 0, 0]
joint_pos = fk_single(home_angles)  # (7, 3)
print("\nHome position joint locations (x, y, z):")
for i, p in enumerate(joint_pos):
    label = "Base" if i == 0 else f"J{i}"
    print(f"  {label}: ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})")


# ── 辅助: 画尺寸标注线 ──────────────────────────────────────────
def draw_dim_line(ax, p1, p2, label, offset_dir, offset_dist=30,
                  color="k", fontsize=8):
    """在 p1-p2 之间画带箭头的尺寸标注."""
    mx = (p1[0] + p2[0]) / 2 + offset_dir[0] * offset_dist
    my = (p1[1] + p2[1]) / 2 + offset_dir[1] * offset_dist
    ax.annotate("", xy=p2, xytext=p1,
                arrowprops=dict(arrowstyle="<->", color=color, lw=1.0))
    ax.text(mx, my, label, fontsize=fontsize, color=color,
            ha="center", va="center",
            bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.8))


def draw_angle_arc(ax, center, radius, angle_start, angle_end, label,
                   color="purple", fontsize=7):
    """在关节处画角度范围弧线."""
    arc = Arc(center, 2 * radius, 2 * radius,
              angle=0, theta1=angle_start, theta2=angle_end,
              color=color, lw=1.2, linestyle="-")
    ax.add_patch(arc)
    mid_angle = np.deg2rad((angle_start + angle_end) / 2)
    tx = center[0] + (radius + 12) * np.cos(mid_angle)
    ty = center[1] + (radius + 12) * np.sin(mid_angle)
    ax.text(tx, ty, label, fontsize=fontsize, color=color,
            ha="center", va="center")


# ══════════════════════════════════════════════════════════════════
#  FIGURE 1: 侧视图 (X-Z plane)
# ══════════════════════════════════════════════════════════════════
print("\n绘制侧视图 ...")
fig1, ax1 = plt.subplots(figsize=(12, 9))

# --- workspace boundary (X-Z projection) ---
# 2D convex hull on (x, z)
pts_xz = np.column_stack([x, z])
hull_xz = ConvexHull(pts_xz)
hull_verts = pts_xz[hull_xz.vertices]
hull_verts = np.vstack([hull_verts, hull_verts[0]])  # close polygon

ax1.fill(hull_verts[:, 0], hull_verts[:, 1],
         color="deepskyblue", alpha=0.15, label="可达工作空间 (凸包)")
ax1.plot(hull_verts[:, 0], hull_verts[:, 1],
         "b--", lw=1.0, alpha=0.6)

# --- max/min reach arcs ---
theta_arc = np.linspace(-np.pi, np.pi, 500)
max_r = 630
min_r = 50
ax1.plot(max_r * np.cos(theta_arc), max_r * np.sin(theta_arc),
         "r--", lw=0.8, alpha=0.5, label=f"最大可达半径 ≈{max_r} mm")
ax1.plot(min_r * np.cos(theta_arc), min_r * np.sin(theta_arc),
         "g--", lw=0.8, alpha=0.5, label=f"最小可达半径 ≈{min_r} mm")

# --- robot arm schematic (side view: X-Z) ---
jx = joint_pos[:, 0]
jz = joint_pos[:, 2]

# links
ax1.plot(jx, jz, "-", color="dimgray", lw=4, solid_capstyle="round", zorder=10)

# joints
joint_colors = ["red", "orange", "gold", "limegreen", "dodgerblue", "mediumpurple"]
for i in range(7):
    if i == 0:
        ax1.plot(jx[i], jz[i], "s", color="red", markersize=10, zorder=12)
        ax1.text(jx[i] - 15, jz[i] - 20, "Base", fontsize=8,
                 fontweight="bold", ha="center", color="red")
    else:
        c = joint_colors[(i - 1) % len(joint_colors)]
        ax1.plot(jx[i], jz[i], "o", color=c, markersize=8,
                 markeredgecolor="k", markeredgewidth=0.5, zorder=12)
        # label offset to avoid overlap
        offx = 12 if i % 2 == 0 else -12
        offz = 15
        ha = "left" if offx > 0 else "right"
        ax1.text(jx[i] + offx, jz[i] + offz,
                 f"J{i}", fontsize=9, fontweight="bold", ha=ha, color=c)

# --- link dimension annotations ---
# d1: base to J1 (vertical)
draw_dim_line(ax1, (jx[0], jz[0]), (jx[1], jz[1]),
              f"d1={LINK_LABELS['d1']}", offset_dir=(-1, 0), offset_dist=40,
              color="darkred", fontsize=8)

# a1: J1 horizontal offset
if abs(jx[2] - jx[1]) > 1 or True:
    draw_dim_line(ax1, (jx[1], jz[1]), (jx[2], jz[2]),
                  f"a1+a2={LINK_LABELS['a1']}+{LINK_LABELS['a2']}",
                  offset_dir=(0, 1), offset_dist=25, color="darkblue", fontsize=8)

# a3 + d4 segment
draw_dim_line(ax1, (jx[3], jz[3]), (jx[4], jz[4]),
              f"a3+d4={LINK_LABELS['a3']}+{LINK_LABELS['d4']}",
              offset_dir=(0, -1), offset_dist=30, color="darkgreen", fontsize=8)

# d6
draw_dim_line(ax1, (jx[5], jz[5]), (jx[6], jz[6]),
              f"d6={LINK_LABELS['d6']}", offset_dir=(1, 0), offset_dist=25,
              color="purple", fontsize=8)

# --- joint angle range annotations ---
arc_radius = 28
# J2 (most visible in side view, rotation in X-Z plane)
draw_angle_arc(ax1, (jx[2], jz[2]), arc_radius, 0, 180,
               f"J2: {JOINT_RANGES_STR[1]}", color="darkorange", fontsize=7)

# --- height range annotation ---
z_min_ws, z_max_ws = z.min(), z.max()
ax1.annotate("", xy=(max_r + 60, z_max_ws), xytext=(max_r + 60, z_min_ws),
             arrowprops=dict(arrowstyle="<->", color="crimson", lw=1.2))
ax1.text(max_r + 75, (z_max_ws + z_min_ws) / 2,
         f"Z范围\n{z_min_ws:.0f}~{z_max_ws:.0f} mm",
         fontsize=8, color="crimson", ha="left", va="center",
         bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="crimson", alpha=0.7))

# --- max reach annotation ---
ax1.annotate("", xy=(max_r, 0), xytext=(0, 0),
             arrowprops=dict(arrowstyle="<->", color="red", lw=1.2))
ax1.text(max_r / 2, -30, f"最大可达 ≈{max_r} mm",
         fontsize=9, color="red", ha="center", va="top",
         bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="red", alpha=0.7))

# --- joint angle range table (text box) ---
range_text = "关节角范围:\n"
for i in range(6):
    range_text += f"  J{i+1}: {JOINT_RANGES_STR[i]}\n"
ax1.text(0.02, 0.02, range_text.strip(), transform=ax1.transAxes,
         fontsize=8, verticalalignment="bottom", fontfamily="SimHei",
         bbox=dict(boxstyle="round,pad=0.4", fc="lightyellow",
                   ec="gray", alpha=0.9))

# --- style ---
ax1.set_xlabel("X (mm)")
ax1.set_ylabel("Z (mm)")
ax1.set_title("6-DOF机械臂侧视图及工作空间", fontsize=15, fontweight="bold")
ax1.set_aspect("equal")
ax1.grid(True, color="lightgray", linewidth=0.5, alpha=0.7)
ax1.legend(loc="upper right", fontsize=9, framealpha=0.9)
ax1.axhline(0, color="gray", lw=0.5, ls="-")
ax1.axvline(0, color="gray", lw=0.5, ls="-")

fig1.tight_layout()
fig1.savefig("D:/papercode/figures/arm_workspace_side.png",
             dpi=300, bbox_inches="tight", facecolor="white")
print("已保存: D:/papercode/figures/arm_workspace_side.png")
plt.close(fig1)


# ══════════════════════════════════════════════════════════════════
#  FIGURE 2: 俯视图 (X-Y plane)
# ══════════════════════════════════════════════════════════════════
print("\n绘制俯视图 ...")
fig2, ax2 = plt.subplots(figsize=(12, 9))

# --- workspace boundary (X-Y projection) ---
pts_xy = np.column_stack([x, y])
hull_xy = ConvexHull(pts_xy)
hull_verts_xy = pts_xy[hull_xy.vertices]
hull_verts_xy = np.vstack([hull_verts_xy, hull_verts_xy[0]])

ax2.fill(hull_verts_xy[:, 0], hull_verts_xy[:, 1],
         color="deepskyblue", alpha=0.15, label="可达工作空间 (凸包)")
ax2.plot(hull_verts_xy[:, 0], hull_verts_xy[:, 1],
         "b--", lw=1.0, alpha=0.6)

# --- max/min reach circles ---
reach_xy = np.sqrt(x**2 + y**2)
max_r_xy = reach_xy.max()
theta_circle = np.linspace(0, 2 * np.pi, 500)
ax2.plot(max_r_xy * np.cos(theta_circle), max_r_xy * np.sin(theta_circle),
         "r--", lw=0.8, alpha=0.5, label=f"最大水平可达 ≈{max_r_xy:.0f} mm")

min_r_xy = reach_xy.min()
ax2.plot(min_r_xy * np.cos(theta_circle), min_r_xy * np.sin(theta_circle),
         "g--", lw=0.8, alpha=0.5, label=f"最小水平可达 ≈{min_r_xy:.0f} mm")

# --- J1 sweep arc annotation ---
j1_min, j1_max = -180, 160
sweep_theta = np.linspace(np.deg2rad(j1_min), np.deg2rad(j1_max), 300)
sweep_r = 150  # annotation radius
ax2.plot(sweep_r * np.cos(sweep_theta), sweep_r * np.sin(sweep_theta),
         "-", color="purple", lw=2.0, alpha=0.7)
# arrow at end
ax2.annotate("", xy=(sweep_r * np.cos(np.deg2rad(j1_max)),
                      sweep_r * np.sin(np.deg2rad(j1_max))),
             xytext=(sweep_r * np.cos(np.deg2rad(j1_max - 10)),
                     sweep_r * np.sin(np.deg2rad(j1_max - 10))),
             arrowprops=dict(arrowstyle="->", color="purple", lw=1.5))
ax2.text(sweep_r * 0.5, sweep_r * 0.8,
         f"J1旋转范围\n{JOINT_RANGES_STR[0]}",
         fontsize=8, color="purple", ha="center",
         bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="purple", alpha=0.8))

# --- robot arm schematic (top view: X-Y) ---
jx_top = joint_pos[:, 0]
jy_top = joint_pos[:, 1]

ax2.plot(jx_top, jy_top, "-", color="dimgray", lw=4,
         solid_capstyle="round", zorder=10)

for i in range(7):
    if i == 0:
        ax2.plot(jx_top[i], jy_top[i], "s", color="red", markersize=10, zorder=12)
        ax2.text(jx_top[i] + 10, jy_top[i] - 15, "Base", fontsize=8,
                 fontweight="bold", color="red")
    else:
        c = joint_colors[(i - 1) % len(joint_colors)]
        ax2.plot(jx_top[i], jy_top[i], "o", color=c, markersize=8,
                 markeredgecolor="k", markeredgewidth=0.5, zorder=12)
        offx = 10 if i % 2 == 0 else -10
        ha = "left" if offx > 0 else "right"
        ax2.text(jx_top[i] + offx, jy_top[i] + 10,
                 f"J{i}", fontsize=9, fontweight="bold", ha=ha, color=c)

# --- max reach annotation ---
ax2.annotate("", xy=(max_r_xy, 0), xytext=(0, 0),
             arrowprops=dict(arrowstyle="<->", color="red", lw=1.2))
ax2.text(max_r_xy / 2, -25, f"最大水平可达 ≈{max_r_xy:.0f} mm",
         fontsize=9, color="red", ha="center", va="top",
         bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="red", alpha=0.7))

# --- joint angle range table ---
range_text = "关节角范围:\n"
for i in range(6):
    range_text += f"  J{i+1}: {JOINT_RANGES_STR[i]}\n"
ax2.text(0.02, 0.02, range_text.strip(), transform=ax2.transAxes,
         fontsize=8, verticalalignment="bottom", fontfamily="SimHei",
         bbox=dict(boxstyle="round,pad=0.4", fc="lightyellow",
                   ec="gray", alpha=0.9))

# --- style ---
ax2.set_xlabel("X (mm)")
ax2.set_ylabel("Y (mm)")
ax2.set_title("6-DOF机械臂俯视图及工作空间", fontsize=15, fontweight="bold")
ax2.set_aspect("equal")
ax2.grid(True, color="lightgray", linewidth=0.5, alpha=0.7)
ax2.legend(loc="upper right", fontsize=9, framealpha=0.9)
ax2.axhline(0, color="gray", lw=0.5, ls="-")
ax2.axvline(0, color="gray", lw=0.5, ls="-")

fig2.tight_layout()
fig2.savefig("D:/papercode/figures/arm_workspace_top.png",
             dpi=300, bbox_inches="tight", facecolor="white")
print("已保存: D:/papercode/figures/arm_workspace_top.png")
plt.close(fig2)

print("\n全部完成。")
