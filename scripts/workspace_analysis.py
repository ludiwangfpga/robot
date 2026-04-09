"""
6-DOF 机器人工作空间分析 (Monte Carlo 采样)
标准DH参数, 500k采样点, 生成俯视图/正视图/组合图
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from scipy.spatial import ConvexHull

# ── matplotlib 中文 & 学术风格 ──────────────────────────────────
plt.rcParams.update({
    "font.family": "SimHei",
    "axes.unicode_minus": False,
    "font.size": 11,
    "axes.labelsize": 12,
    "axes.titlesize": 14,
    "figure.dpi": 150,
    "savefig.dpi": 300,
})

# ── DH 参数 (标准DH) ───────────────────────────────────────────
#       a(mm)   alpha(rad)      d(mm)   theta_min(deg)  theta_max(deg)
DH = np.array([
    [55,    np.pi / 2,   166,  -180,  160],
    [200,   0,             0,     0,  180],
    [56,    np.pi / 2,     0,   -80,   83],
    [0,    -np.pi / 2,   192,   -30,  305],
    [0,     np.pi / 2,     0,  -110,  110],
    [0,     0,            55,   -30,  305],
])

N_SAMPLES = 500_000
np.random.seed(42)


# ── 正运动学 ────────────────────────────────────────────────────
def dh_matrix(a, alpha, d, theta):
    """标准DH变换矩阵 (向量化, theta 为 1-D array)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    T = np.zeros((len(theta), 4, 4))
    T[:, 0, 0] = ct
    T[:, 0, 1] = -st * ca
    T[:, 0, 2] = st * sa
    T[:, 0, 3] = a * ct
    T[:, 1, 0] = st
    T[:, 1, 1] = ct * ca
    T[:, 1, 2] = -ct * sa
    T[:, 1, 3] = a * st
    T[:, 2, 1] = sa
    T[:, 2, 2] = ca
    T[:, 2, 3] = d
    T[:, 3, 3] = 1.0
    return T


def forward_kinematics_batch(joint_angles_deg):
    """批量正运动学, joint_angles_deg: (N, 6), 返回 (N, 3) 末端位置."""
    N = joint_angles_deg.shape[0]
    T_acc = np.broadcast_to(np.eye(4), (N, 4, 4)).copy()
    for i in range(6):
        a_i = DH[i, 0]
        alpha_i = DH[i, 1]
        d_i = DH[i, 2]
        theta_i = np.deg2rad(joint_angles_deg[:, i])
        T_i = dh_matrix(a_i, alpha_i, d_i, theta_i)
        T_acc = np.einsum("nij,njk->nik", T_acc, T_i)
    return T_acc[:, :3, 3]


# ── 采样 ────────────────────────────────────────────────────────
print(f"采样 {N_SAMPLES} 组关节角 ...")
theta_min = DH[:, 3]
theta_max = DH[:, 4]
samples = np.random.uniform(theta_min, theta_max, size=(N_SAMPLES, 6))

BATCH = 50_000
positions = []
for start in range(0, N_SAMPLES, BATCH):
    end = min(start + BATCH, N_SAMPLES)
    positions.append(forward_kinematics_batch(samples[start:end]))
    print(f"  已完成 {end}/{N_SAMPLES}")
pos = np.vstack(positions)
x, y, z = pos[:, 0], pos[:, 1], pos[:, 2]

# ── 工作空间统计 ─────────────────────────────────────────────────
reach = np.sqrt(x**2 + y**2 + z**2)
reach_xy = np.sqrt(x**2 + y**2)
max_reach = reach.max()
min_reach = reach.min()

print("\n========== 工作空间统计 ==========")
print(f"最大可达距离 (max reach):  {max_reach:.1f} mm")
print(f"最小可达距离 (min reach):  {min_reach:.1f} mm")
print(f"X 范围: [{x.min():.1f}, {x.max():.1f}] mm  (跨度 {x.max()-x.min():.1f} mm)")
print(f"Y 范围: [{y.min():.1f}, {y.max():.1f}] mm  (跨度 {y.max()-y.min():.1f} mm)")
print(f"Z 范围: [{z.min():.1f}, {z.max():.1f}] mm  (跨度 {z.max()-z.min():.1f} mm)")

try:
    hull = ConvexHull(pos)
    volume = hull.volume
    print(f"近似工作空间体积 (凸包): {volume:.0f} mm^3  ({volume/1e9:.4f} m^3)")
except Exception as e:
    print(f"凸包计算失败: {e}")
    volume = None
print("==================================\n")


# ── 绘图辅助 ─────────────────────────────────────────────────────
def plot_workspace_view(ax, h, v, hlabel, vlabel, title, max_r_h, max_r_v):
    """用 hexbin 绘制工作空间投影."""
    hb = ax.hexbin(h, v, gridsize=250, cmap="inferno", mincnt=1,
                   linewidths=0.0, reduce_C_function=np.sum)
    cb = plt.colorbar(hb, ax=ax, shrink=0.75, pad=0.02)
    cb.set_label("采样点密度", fontsize=10)

    # 基座
    ax.plot(0, 0, "r^", markersize=8, zorder=5, label="基座")

    # 最大可达半径
    if max_r_v is None:
        # 俯视图: 画圆
        circle = Circle((0, 0), max_r_h, fill=False, edgecolor="red",
                         linestyle="--", linewidth=1.0, label=f"最大可达 {max_r_h:.0f} mm")
        ax.add_patch(circle)
    else:
        # 正视图: 标注水平和垂直范围线
        ax.axvline(max_r_h, color="red", ls="--", lw=0.8, alpha=0.6)
        ax.axvline(-max_r_h, color="red", ls="--", lw=0.8, alpha=0.6)

    ax.set_xlabel(hlabel, fontsize=12)
    ax.set_ylabel(vlabel, fontsize=12)
    ax.set_title(title, fontsize=14, fontweight="bold")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)


# ── 俯视图 (X-Y) ────────────────────────────────────────────────
fig1, ax1 = plt.subplots(figsize=(7, 7))
max_reach_xy = reach_xy.max()
plot_workspace_view(ax1, x, y, "X (mm)", "Y (mm)",
                    "俯视图 (Top View)", max_reach_xy, None)
fig1.savefig("D:/papercode/figures/workspace_top_view.png", bbox_inches="tight")
print("已保存: D:/papercode/figures/workspace_top_view.png")
plt.close(fig1)

# ── 正视图 (X-Z) ────────────────────────────────────────────────
fig2, ax2 = plt.subplots(figsize=(7, 7))
max_reach_xz = np.sqrt(x**2 + z**2).max()
plot_workspace_view(ax2, x, z, "X (mm)", "Z (mm)",
                    "正视图 (Front View)", x.max(), max_reach_xz)
fig2.savefig("D:/papercode/figures/workspace_front_view.png", bbox_inches="tight")
print("已保存: D:/papercode/figures/workspace_front_view.png")
plt.close(fig2)

# ── 组合图 ───────────────────────────────────────────────────────
fig3, (ax3, ax4) = plt.subplots(1, 2, figsize=(14, 6.5))
plot_workspace_view(ax3, x, y, "X (mm)", "Y (mm)",
                    "俯视图 (Top View)", max_reach_xy, None)
plot_workspace_view(ax4, x, z, "X (mm)", "Z (mm)",
                    "正视图 (Front View)", x.max(), max_reach_xz)
fig3.tight_layout(pad=2.0)
fig3.savefig("D:/papercode/figures/workspace_combined.png", bbox_inches="tight")
print("已保存: D:/papercode/figures/workspace_combined.png")
plt.close(fig3)

print("\n全部完成。")
