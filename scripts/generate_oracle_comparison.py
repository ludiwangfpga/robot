#!/usr/bin/env python3
"""
生成包含Oracle基准线的自适应阻抗控制对比图
展示：固定Ks vs 自适应Ks vs 理论最优Ks
"""

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
from scipy import signal

# 设置图表样式
plt.rcParams.update({
    'font.size': 11, 'font.family': 'serif',
    'figure.dpi': 300, 'savefig.dpi': 300, 'savefig.bbox': 'tight',
    'axes.grid': True, 'grid.alpha': 0.3,
})

FIG = "D:/papercode/figures/ch5"
os.makedirs(FIG, exist_ok=True)

# 电机参数
I_RATED = 1.68    # A
TAU_HOLD = 0.45   # Nm
KT = 0.27         # Nm/A

# ============================================================
# 阅读CSV数据（已有的4个配置）
# ============================================================
import csv

def read_csv(filename):
    """读取CSV文件并返回字典格式数据"""
    data = {}
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        # 初始化每个字段
        for field in fieldnames:
            data[field] = []
        # 读取数据
        for row in reader:
            for field in fieldnames:
                data[field].append(float(row[field]))
    # 转换为numpy数组
    for field in fieldnames:
        data[field] = np.array(data[field])
    return data

csv_dir = "D:/papercode/data/csv"
print("读取CSV数据...")
A = read_csv(f"{csv_dir}/ablation_config_A_new.csv")
B = read_csv(f"{csv_dir}/ablation_config_B_new.csv")
C = read_csv(f"{csv_dir}/ablation_config_C_new.csv")
D = read_csv(f"{csv_dir}/ablation_config_D_new.csv")

t = A['t']  # 时间轴

# ============================================================
# 读取Oracle配置E的数据（由gen_ablation_comparison.py完整仿真生成）
# ============================================================
print("读取Oracle基准线数据...")
E = read_csv(f"{csv_dir}/ablation_config_E_new.csv")
print(f"  [OK] 已读取 Oracle Ks=600 完整仿真数据")

# ============================================================
# 图1：包含Oracle的6面板对比图
# ============================================================
print("生成对比图：Figure 1 - 6面板（含Oracle基准线）...")

fig, axes = plt.subplots(3, 2, figsize=(14, 10))

# (a) Position
ax = axes[0,0]
ax.plot(t, A['x']*1000, '#EF5350', lw=1.5, label='A: Fixed Ks=300')
ax.plot(t, D['x']*1000, '#2E7D32', lw=1.5, label='D: Adaptive+Pred')
ax.plot(t, E['x']*1000, '#1565C0', lw=2.0, ls='--', label='E: Oracle Ks=600')
ax.axhline(y=-3, color='k', ls='--', alpha=0.4, label='Desired')
ax.axhline(y=0, color='gray', ls=':', alpha=0.3, label='Wall')
ax.set_ylabel('Position (mm)'); ax.set_title('(a) End-Effector Position')
ax.legend(fontsize=8, loc='lower left')
ax.grid(True, alpha=0.3)

# (b) Contact Force - 关键对比！
ax = axes[0,1]
ax.plot(t, A['F_env'], '#EF5350', lw=1.5, label='A: Fixed Ks=300')
ax.plot(t, D['F_env'], '#2E7D32', lw=1.5, label='D: Adaptive+Pred')
ax.plot(t, E['F_env'], '#1565C0', lw=2.0, ls='--', label='E: Oracle Ks=600')
ax.set_ylabel('Contact Force (N)')
ax.set_title('(b) Contact Force - Theory vs Practice')
ax.legend(fontsize=8, loc='upper right')
ax.grid(True, alpha=0.3)
ax.fill_between(t, A['F_env'], E['F_env'], where=(A['F_env']>=E['F_env']),
                 alpha=0.2, color='blue', label='Theory bound')

# (c) Motor Torque
ax = axes[1,0]
ax.plot(t, A['tau_motor'], '#EF5350', lw=0.8, alpha=0.7, label='A: Fixed Ks=300')
ax.plot(t, D['tau_motor'], '#2E7D32', lw=0.8, alpha=0.7, label='D: Adaptive+Pred')
# Oracle理论值应平滑，对E应用平滑滤波去除chopper纹波
E_tau_smooth = np.convolve(E['tau_motor'], np.ones(11)/11, mode='same')
ax.plot(t, E_tau_smooth, '#1565C0', lw=1.2, alpha=0.8, ls='--', label='E: Oracle Ks=600 (smoothed)')
ax.axhline(y=TAU_HOLD, color='orange', ls='--', alpha=0.5, linewidth=1)
ax.axhline(y=-TAU_HOLD, color='orange', ls='--', alpha=0.5, linewidth=1)
ax.set_ylabel('Motor Torque (Nm)'); ax.set_title('(c) Motor Torque')
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

# (d) Impedance Parameters
ax = axes[1,1]
ax.plot(t, A['Ks'], '#EF5350', lw=1.5, label='A: Fixed Ks=300')

# ===== 这一段是绿线的波形数据 =====
ax.plot(t, D['Ks'], '#2E7D32', lw=1.5, label='D: Adaptive Ks')
# ===== 你修改 D['Ks'] 就能改变绿线的波形 =====

ax.plot(t, E['Ks'], '#1565C0', lw=2.0, ls='--', label='E: Oracle Ks=600')
ax.set_ylabel('Controller Stiffness Ks (N/m)')
ax.set_title('(d) Stiffness Adaptation')
ax.legend(fontsize=8, loc='upper left')
ax.grid(True, alpha=0.3)

# (e) Motor Current iq - 放大
ax = axes[2,0]
zmask = (t >= 9) & (t <= 18)
ax.plot(t[zmask], A['iq_actual'][zmask], '#EF5350', lw=0.6, alpha=0.6, label='A: Fixed')
ax.plot(t[zmask], D['iq_actual'][zmask], '#2E7D32', lw=0.6, alpha=0.6, label='D: Adaptive+Pred')
ax.plot(t[zmask], E['iq_actual'][zmask], '#1565C0', lw=0.8, alpha=0.7, label='E: Oracle')
ax.axhline(y=I_RATED, color='orange', ls=':', alpha=0.4)
ax.axhline(y=-I_RATED, color='orange', ls=':', alpha=0.4)
ax.set_ylabel(r'$i_q$ (A)'); ax.set_xlabel('Time (ms)')
ax.set_title(r'(e) Motor Current $i_q$ (9–18 ms zoom)')
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

# (f) Current tracking error
ax = axes[2,1]
err_A = np.abs(A['iq_ideal'] - A['iq_actual'])
err_D = np.abs(D['iq_ideal'] - D['iq_actual'])
err_E = np.abs(E['iq_ideal'] - E['iq_actual'])
win = 80
err_A_s = np.convolve(err_A, np.ones(win)/win, mode='same')
err_D_s = np.convolve(err_D, np.ones(win)/win, mode='same')
err_E_s = np.convolve(err_E, np.ones(win)/win, mode='same')
ax.plot(t, err_A_s, '#EF5350', lw=1.5, label='A: Fixed, No Pred')
ax.plot(t, err_D_s, '#2E7D32', lw=1.5, label='D: Adaptive + Pred')
ax.plot(t, err_E_s, '#1565C0', lw=2.0, ls='--', label='E: Oracle + Pred')
ax.set_ylabel(r'$|i_{q,ideal} - i_{q,actual}|$ (A)'); ax.set_xlabel('Time (ms)')
ax.set_title('(f) Current Tracking Error (smoothed)')
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

fig.suptitle('Ablation Study: Fixed vs Adaptive vs Oracle Impedance Control\n' +
             'Motor: 42BYGH48 (Kt=0.27 Nm/A, Irated=1.68A, L=4.1mH, R=1.65Ω)',
             fontsize=12, fontweight='bold', y=0.995)
fig.tight_layout()
output_path = f"{FIG}/消融实验_Oracle对比_6面板.png"
fig.savefig(output_path, dpi=300, bbox_inches='tight')
plt.close()
print(f"  [OK] Saved: {output_path}")

# ============================================================
# 图2：接触区域放大（8-30ms）
# ============================================================
print("生成对比图：Figure 2 - 接触区域放大...")

mask = (t >= 8) & (t <= 30)
fig, axes = plt.subplots(2, 2, figsize=(12, 7))

ax = axes[0,0]
ax.plot(t[mask], A['x'][mask]*1000, '#EF5350', lw=1.5, label='A: Fixed Ks=300')
ax.plot(t[mask], D['x'][mask]*1000, '#2E7D32', lw=1.5, label='D: Adaptive+Pred')
ax.plot(t[mask], E['x'][mask]*1000, '#1565C0', lw=2.0, ls='--', label='E: Oracle Ks=600')
ax.axhline(y=0, color='gray', ls=':', alpha=0.3)
ax.set_ylabel('Position (mm)'); ax.set_title('(a) Position')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)

ax = axes[0,1]
ax.plot(t[mask], A['F_env'][mask], '#EF5350', lw=1.5, label='A: Fixed Ks=300')
ax.plot(t[mask], D['F_env'][mask], '#2E7D32', lw=1.5, label='D: Adaptive+Pred')
ax.plot(t[mask], E['F_env'][mask], '#1565C0', lw=2.0, ls='--', label='E: Oracle Ks=600')
ax.set_ylabel('Contact Force (N)'); ax.set_title('(b) Contact Force')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)

ax = axes[1,0]
ax.plot(t[mask], A['iq_actual'][mask], '#EF5350', lw=0.6, alpha=0.6, label='A: Fixed')
ax.plot(t[mask], D['iq_actual'][mask], '#2E7D32', lw=0.6, alpha=0.6, label='D: Adaptive+Pred')
ax.plot(t[mask], E['iq_actual'][mask], '#1565C0', lw=0.8, alpha=0.7, label='E: Oracle')
ax.axhline(y=I_RATED, color='orange', ls=':', alpha=0.4)
ax.axhline(y=-I_RATED, color='orange', ls=':', alpha=0.4)
ax.set_ylabel(r'$i_q$ (A)'); ax.set_xlabel('Time (ms)')
ax.set_title('(c) Motor Current with Ripple')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)

ax = axes[1,1]
ax.plot(t[mask], A['tau_motor'][mask], '#EF5350', lw=0.8, alpha=0.7, label='A: Fixed Ks=300')
ax.plot(t[mask], D['tau_motor'][mask], '#2E7D32', lw=0.8, alpha=0.7, label='D: Adaptive+Pred')
# Oracle理论值应平滑
E_tau_smooth = np.convolve(E['tau_motor'], np.ones(11)/11, mode='same')
ax.plot(t[mask], E_tau_smooth[mask], '#1565C0', lw=1.2, alpha=0.8, ls='--', label='E: Oracle Ks=600 (smoothed)')
ax.axhline(y=TAU_HOLD, color='orange', ls='--', alpha=0.5, linewidth=1)
ax.set_ylabel('Torque (Nm)'); ax.set_xlabel('Time (ms)')
ax.set_title('(d) Motor Torque')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)

fig.suptitle('Zoomed: Contact Region (8–30 ms) with Oracle Baseline',
             fontsize=12, fontweight='bold')
fig.tight_layout()
output_path = f"{FIG}/消融实验_Oracle对比_接触区放大.png"
fig.savefig(output_path, dpi=300, bbox_inches='tight')
plt.close()
print(f"  [OK] Saved: {output_path}")

# ============================================================
# 计算和展示性能指标
# ============================================================
print("\n" + "="*90)
print("  性能指标对比 — 接触区域后期 (t > 12 ms)")
print("="*90)

post = t > 12

configs = [
    ('A: Fixed Ks=300, No Pred', A),
    ('B: Fixed Ks=300, + Pred', B),
    ('C: Adaptive Ks, No Pred', C),
    ('D: Adaptive Ks, + Pred', D),
    ('E: Oracle Ks=600, + Pred', E),
]

print(f"\n{'配置':<30} {'接触力峰值':>10} {'接触力RMS':>10} {'电流误差':>10} {'力矩峰值':>10}")
print(f"{'-'*30} {'-'*10} {'-'*10} {'-'*10} {'-'*10}")

results = {}
for name, cfg in configs:
    F_peak = np.max(np.abs(cfg['F_env'][post]))
    F_rms = np.sqrt(np.mean(cfg['F_env'][post]**2))
    iq_err = np.sqrt(np.mean((cfg['iq_ideal'][post] - cfg['iq_actual'][post])**2))
    tau_peak = np.max(np.abs(cfg['tau_motor'][post]))

    results[name] = {'F_peak': F_peak, 'F_rms': F_rms, 'iq_err': iq_err, 'tau_peak': tau_peak}

    print(f"{name:<30} {F_peak:9.2f}N {F_rms:9.2f}N {iq_err:9.3f}A {tau_peak:9.3f}Nm")

print("="*90)
print("\n相对改进幅度：")
print(f"  A vs E (理论上界):       {(results['A: Fixed Ks=300, No Pred']['F_peak'] - results['E: Oracle Ks=600, + Pred']['F_peak']) / results['A: Fixed Ks=300, No Pred']['F_peak'] * 100:.1f}% (最大可能改进)")
print(f"  D vs A (实际自适应):     {(results['A: Fixed Ks=300, No Pred']['F_peak'] - results['D: Adaptive Ks, + Pred']['F_peak']) / results['A: Fixed Ks=300, No Pred']['F_peak'] * 100:.1f}% (实现的改进)")
print(f"  D vs E (学习成本):       {(results['D: Adaptive Ks, + Pred']['F_peak'] - results['E: Oracle Ks=600, + Pred']['F_peak']) / results['A: Fixed Ks=300, No Pred']['F_peak'] * 100:.1f}% (还有这么多空间)")
print(f"  达成率 (D-A)/(E-A):      {(results['A: Fixed Ks=300, No Pred']['F_peak'] - results['D: Adaptive Ks, + Pred']['F_peak']) / (results['A: Fixed Ks=300, No Pred']['F_peak'] - results['E: Oracle Ks=600, + Pred']['F_peak']) * 100:.1f}% (接近理论最优的程度)")

print("\n" + "="*90)
print("[DONE] All figures generated successfully!")
print("="*90)
