#!/usr/bin/env python3
"""
生成符合IEEE T-RO、NIST标准的改进实验对比图表
基于多阶段固定刚度设计
"""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

plt.rcParams.update({
    'font.size': 11, 'font.family': 'serif',
    'figure.dpi': 300, 'savefig.dpi': 300, 'savefig.bbox': 'tight',
    'axes.grid': True, 'grid.alpha': 0.3,
})

FIG = "D:/papercode/figures/ch5"
os.makedirs(FIG, exist_ok=True)

# 读取改进的数据
def read_csv(filename):
    df = pd.read_csv(filename)
    return {col: df[col].values for col in df.columns}

csv_dir = "D:/papercode/data/csv"
A = read_csv(f"{csv_dir}/ablation_config_A_new.csv")
D = read_csv(f"{csv_dir}/ablation_config_D_new.csv")
E = read_csv(f"{csv_dir}/ablation_config_E_new.csv")

t = A['t']

# ============================================================
# 图1：软接触阶段独立分析（Phase 2: 22-35ms, K=500 N/m）
# ============================================================
print("生成改进图表1：软接触阶段5面板对比...")

soft_phase_idx = (t >= 22) & (t <= 35)
t_soft = t[soft_phase_idx]

fig, axes = plt.subplots(2, 3, figsize=(15, 8))

I_RATED = 1.68
TAU_HOLD = 0.45
KT = 0.27

# (a) 位置响应
ax = axes[0,0]
ax.plot(t_soft, A['x'][soft_phase_idx]*1000, '#EF5350', lw=2, label='A: Fixed Ks=300')
ax.plot(t_soft, D['x'][soft_phase_idx]*1000, '#2E7D32', lw=2, label='D: Adaptive Ks+Pred')
ax.plot(t_soft, E['x'][soft_phase_idx]*1000, '#1565C0', lw=2.5, ls='--', label='E: Oracle Ks=775')
ax.axhline(y=-3, color='k', ls='--', alpha=0.4, label='Target')
ax.set_ylabel('Position (mm)', fontsize=11)
ax.set_title('(a) End-Effector Position', fontsize=11, fontweight='bold')
ax.legend(fontsize=9, loc='best')
ax.grid(True, alpha=0.3)

# (b) 接触力 - 关键指标
ax = axes[0,1]
ax.plot(t_soft, A['F_env'][soft_phase_idx], '#EF5350', lw=2, label='A: Fixed Ks=300')
ax.plot(t_soft, D['F_env'][soft_phase_idx], '#2E7D32', lw=2, label='D: Adaptive Ks+Pred')
ax.plot(t_soft, E['F_env'][soft_phase_idx], '#1565C0', lw=2.5, ls='--', label='E: Oracle Ks=775')
ax.axhline(y=0, color='k', ls='-', alpha=0.3, linewidth=0.5)
ax.set_ylabel('Contact Force (N)', fontsize=11)
ax.set_title('(b) Contact Force (Phase 2, K=500)', fontsize=11, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)

# (c) 环境刚度估计
ax = axes[0,2]
ax.plot(t_soft, D['K_est'][soft_phase_idx], '#2E7D32', lw=2.5, label='K_env Estimate')
ax.axhline(y=500, color='blue', ls='--', linewidth=2, alpha=0.8, label='True K_env = 500')
ax.set_ylabel('Stiffness (N/m)', fontsize=11)
ax.set_title('(c) Environment Stiffness Learning', fontsize=11, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)
ax.set_ylim([0, 1000])

# (d) 自适应参数调节
ax = axes[1,0]
ax.plot(t_soft, A['Ks'][soft_phase_idx], '#EF5350', lw=2, label='A: Fixed Ks=300')
ax.plot(t_soft, D['Ks'][soft_phase_idx], '#2E7D32', lw=2, label='D: Adaptive Ks')
ax.plot(t_soft, E['Ks'][soft_phase_idx], '#1565C0', lw=2.5, ls='--', label='E: Oracle Ks=775')
ax.set_ylabel('Controller Stiffness (N/m)', fontsize=11)
ax.set_title('(d) Adaptive Stiffness Parameter', fontsize=11, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)

# (e) 力矩
ax = axes[1,1]
ax.plot(t_soft, A['tau_motor'][soft_phase_idx], '#EF5350', lw=1, alpha=0.8, label='A: Fixed Ks=300')
ax.plot(t_soft, D['tau_motor'][soft_phase_idx], '#2E7D32', lw=1, alpha=0.8, label='D: Adaptive Ks+Pred')
ax.plot(t_soft, E['tau_motor'][soft_phase_idx], '#1565C0', lw=1.2, alpha=0.8, ls='--', label='E: Oracle Ks=775')
ax.axhline(y=TAU_HOLD, color='orange', ls='--', alpha=0.5, linewidth=1)
ax.axhline(y=-TAU_HOLD, color='orange', ls='--', alpha=0.5, linewidth=1)
ax.set_ylabel('Motor Torque (Nm)', fontsize=11)
ax.set_title('(e) Motor Torque', fontsize=11, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)

# (f) 电流误差
ax = axes[1,2]
err_A = np.abs(A['iq_ideal'][soft_phase_idx] - A['iq_actual'][soft_phase_idx])
err_D = np.abs(D['iq_ideal'][soft_phase_idx] - D['iq_actual'][soft_phase_idx])
err_E = np.abs(E['iq_ideal'][soft_phase_idx] - E['iq_actual'][soft_phase_idx])
win = 50
err_A_s = np.convolve(err_A, np.ones(win)/win, mode='same')
err_D_s = np.convolve(err_D, np.ones(win)/win, mode='same')
err_E_s = np.convolve(err_E, np.ones(win)/win, mode='same')
ax.plot(t_soft, err_A_s, '#EF5350', lw=2, label='A: Fixed')
ax.plot(t_soft, err_D_s, '#2E7D32', lw=2, label='D: Adaptive+Pred')
ax.plot(t_soft, err_E_s, '#1565C0', lw=2.5, ls='--', label='E: Oracle+Pred')
ax.set_ylabel('Current Error |iq_ideal - iq_actual| (A)', fontsize=11)
ax.set_xlabel('Time (ms)', fontsize=11)
ax.set_title('(f) Current Tracking Error', fontsize=11, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)

fig.suptitle('PHASE 2: SOFT CONTACT (22-35ms, K_env = 500 N/m)\nAdaptive Learning in Soft Environment',
             fontsize=13, fontweight='bold', y=0.995)
fig.tight_layout()
fig.savefig(f"{FIG}/改进实验_软接触阶段_5面板.png", dpi=300, bbox_inches='tight')
plt.close()
print(">> Saved: 改进实验_软接触阶段_5面板.png")

# ============================================================
# 图2：硬接触阶段独立分析（Phase 3: 35-50ms, K=2000 N/m）
# ============================================================
print("生成改进图表2：硬接触阶段5面板对比...")

hard_phase_idx = (t >= 35)
t_hard = t[hard_phase_idx]

fig, axes = plt.subplots(2, 3, figsize=(15, 8))

# (a) 位置响应
ax = axes[0,0]
ax.plot(t_hard, A['x'][hard_phase_idx]*1000, '#EF5350', lw=2, label='A: Fixed Ks=300')
ax.plot(t_hard, D['x'][hard_phase_idx]*1000, '#2E7D32', lw=2, label='D: Adaptive Ks+Pred')
ax.plot(t_hard, E['x'][hard_phase_idx]*1000, '#1565C0', lw=2.5, ls='--', label='E: Oracle Ks=775')
ax.axhline(y=-3, color='k', ls='--', alpha=0.4, label='Target')
ax.set_ylabel('Position (mm)', fontsize=11)
ax.set_title('(a) End-Effector Position', fontsize=11, fontweight='bold')
ax.legend(fontsize=9, loc='best')
ax.grid(True, alpha=0.3)

# (b) 接触力 - 关键指标
ax = axes[0,1]
ax.plot(t_hard, A['F_env'][hard_phase_idx], '#EF5350', lw=2, label='A: Fixed Ks=300')
ax.plot(t_hard, D['F_env'][hard_phase_idx], '#2E7D32', lw=2, label='D: Adaptive Ks+Pred')
ax.plot(t_hard, E['F_env'][hard_phase_idx], '#1565C0', lw=2.5, ls='--', label='E: Oracle Ks=775')
ax.axhline(y=0, color='k', ls='-', alpha=0.3, linewidth=0.5)
ax.set_ylabel('Contact Force (N)', fontsize=11)
ax.set_title('(b) Contact Force (Phase 3, K=2000)', fontsize=11, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)

# (c) 环境刚度估计
ax = axes[0,2]
ax.plot(t_hard, D['K_est'][hard_phase_idx], '#2E7D32', lw=2.5, label='K_env Estimate')
ax.axhline(y=2000, color='red', ls='--', linewidth=2, alpha=0.8, label='True K_env = 2000')
ax.set_ylabel('Stiffness (N/m)', fontsize=11)
ax.set_title('(c) Environment Stiffness Learning', fontsize=11, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)
ax.set_ylim([500, 2500])

# (d) 自适应参数调节
ax = axes[1,0]
ax.plot(t_hard, A['Ks'][hard_phase_idx], '#EF5350', lw=2, label='A: Fixed Ks=300')
ax.plot(t_hard, D['Ks'][hard_phase_idx], '#2E7D32', lw=2, label='D: Adaptive Ks')
ax.plot(t_hard, E['Ks'][hard_phase_idx], '#1565C0', lw=2.5, ls='--', label='E: Oracle Ks=775')
ax.set_ylabel('Controller Stiffness (N/m)', fontsize=11)
ax.set_title('(d) Adaptive Stiffness Parameter', fontsize=11, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)

# (e) 力矩
ax = axes[1,1]
ax.plot(t_hard, A['tau_motor'][hard_phase_idx], '#EF5350', lw=1, alpha=0.8, label='A: Fixed Ks=300')
ax.plot(t_hard, D['tau_motor'][hard_phase_idx], '#2E7D32', lw=1, alpha=0.8, label='D: Adaptive Ks+Pred')
ax.plot(t_hard, E['tau_motor'][hard_phase_idx], '#1565C0', lw=1.2, alpha=0.8, ls='--', label='E: Oracle Ks=775')
ax.axhline(y=TAU_HOLD, color='orange', ls='--', alpha=0.5, linewidth=1)
ax.axhline(y=-TAU_HOLD, color='orange', ls='--', alpha=0.5, linewidth=1)
ax.set_ylabel('Motor Torque (Nm)', fontsize=11)
ax.set_title('(e) Motor Torque', fontsize=11, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)

# (f) 电流误差
ax = axes[1,2]
err_A = np.abs(A['iq_ideal'][hard_phase_idx] - A['iq_actual'][hard_phase_idx])
err_D = np.abs(D['iq_ideal'][hard_phase_idx] - D['iq_actual'][hard_phase_idx])
err_E = np.abs(E['iq_ideal'][hard_phase_idx] - E['iq_actual'][hard_phase_idx])
win = 50
err_A_s = np.convolve(err_A, np.ones(win)/win, mode='same')
err_D_s = np.convolve(err_D, np.ones(win)/win, mode='same')
err_E_s = np.convolve(err_E, np.ones(win)/win, mode='same')
ax.plot(t_hard, err_A_s, '#EF5350', lw=2, label='A: Fixed')
ax.plot(t_hard, err_D_s, '#2E7D32', lw=2, label='D: Adaptive+Pred')
ax.plot(t_hard, err_E_s, '#1565C0', lw=2.5, ls='--', label='E: Oracle+Pred')
ax.set_ylabel('Current Error |iq_ideal - iq_actual| (A)', fontsize=11)
ax.set_xlabel('Time (ms)', fontsize=11)
ax.set_title('(f) Current Tracking Error', fontsize=11, fontweight='bold')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)

fig.suptitle('PHASE 3: HARD CONTACT (35-50ms, K_env = 2000 N/m)\nAdaptive Learning in Hard Environment (25x Stiffer)',
             fontsize=13, fontweight='bold', y=0.995)
fig.tight_layout()
fig.savefig(f"{FIG}/改进实验_硬接触阶段_5面板.png", dpi=300, bbox_inches='tight')
plt.close()
print(">> Saved: 改进实验_硬接触阶段_5面板.png")

# ============================================================
# 图2：性能指标对比（NIST标准）
# ============================================================
print("生成改进图表2：性能指标对比...")

soft_phase = (t >= 22) & (t <= 35)
hard_phase = (t >= 35)

fig, axes = plt.subplots(1, 2, figsize=(12, 5))

# (a) 软接触阶段
ax = axes[0]
configs = ['Fixed\n(A)', 'Adaptive\n(D)', 'Oracle\n(E)']
f_peak_soft = [
    np.max(np.abs(A['F_env'][soft_phase])),
    np.max(np.abs(D['F_env'][soft_phase])),
    np.max(np.abs(E['F_env'][soft_phase]))
]
colors = ['#EF5350', '#2E7D32', '#1565C0']
bars = ax.bar(configs, f_peak_soft, color=colors, alpha=0.7, edgecolor='black', linewidth=1.5)
ax.set_ylabel('Force Peak (N)', fontsize=11)
ax.set_title('(a) Soft Contact Phase (22-35ms)\nK_env = 500 N/m', fontsize=11, fontweight='bold')
ax.grid(True, alpha=0.3, axis='y')
# 添加数值标签
for bar, val in zip(bars, f_peak_soft):
    ax.text(bar.get_x() + bar.get_width()/2, val + 0.1, f'{val:.2f}N',
            ha='center', va='bottom', fontsize=10, fontweight='bold')

# (b) 硬接触阶段
ax = axes[1]
f_peak_hard = [
    np.max(np.abs(A['F_env'][hard_phase])),
    np.max(np.abs(D['F_env'][hard_phase])),
    np.max(np.abs(E['F_env'][hard_phase]))
]
bars = ax.bar(configs, f_peak_hard, color=colors, alpha=0.7, edgecolor='black', linewidth=1.5)
ax.set_ylabel('Force Peak (N)', fontsize=11)
ax.set_title('(b) Hard Contact Phase (35-50ms)\nK_env = 2000 N/m', fontsize=11, fontweight='bold')
ax.grid(True, alpha=0.3, axis='y')
# 添加数值标签
for bar, val in zip(bars, f_peak_hard):
    ax.text(bar.get_x() + bar.get_width()/2, val + 0.1, f'{val:.2f}N',
            ha='center', va='bottom', fontsize=10, fontweight='bold')

fig.suptitle('Performance Metrics Comparison (NIST Standard)\nAdaptive vs Fixed vs Oracle',
             fontsize=12, fontweight='bold')
fig.tight_layout()
fig.savefig(f"{FIG}/改进实验_性能指标对比.png", dpi=300, bbox_inches='tight')
plt.close()
print(">> Saved: 改进实验_性能指标对比.png")

print("")
print("========================================")
print("All improved figures generated!")
print("Location: ", FIG)
