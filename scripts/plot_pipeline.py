import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch
import numpy as np

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

fig = plt.figure(figsize=(16, 10))
gs = fig.add_gridspec(2, 2, height_ratios=[3, 2.5], width_ratios=[3, 1.2],
                      hspace=0.35, wspace=0.25)
ax1 = fig.add_subplot(gs[0, 0])   # 甘特图
ax2 = fig.add_subplot(gs[0, 1])   # 条形图
ax3 = fig.add_subplot(gs[1, :])   # 7:1 频率关系

fig.suptitle('控制链流水线时序与频率冗余利用', fontsize=15, fontweight='bold', y=0.97)

# ============================================================
# (a) 流水线甘特图
# ============================================================
modules = [
    ('自适应 RLS\n(与FK并行)',   0, 48,  '#E8A0BF'),
    ('FK + 解析雅可比',          0, 300, '#4ECDC4'),
    ('速度估计',                300, 4,   '#FFD93D'),
    ('阻抗控制',                304, 3,   '#FF6B6B'),
    ('J^T 力映射',              307, 8,   '#6C5CE7'),
    ('τ→Iq 转换',              315, 3,   '#A8E6CF'),
]
y_positions = [5, 4, 3, 2, 1, 0]

for i, (name, start, duration, color) in enumerate(modules):
    y = y_positions[i]
    ax1.barh(y, duration, left=start, height=0.55, color=color,
             edgecolor='#333333', linewidth=1.2, zorder=3)
    cx = start + duration / 2
    if duration >= 20:
        ax1.text(cx, y, f'{duration} cyc', ha='center', va='center',
                fontsize=11, fontweight='bold', color='#222222')
    else:
        ax1.text(start + duration + 4, y, f'{duration} cyc', ha='left', va='center',
                fontsize=10, fontweight='bold', color='#444444')

rect = FancyBboxPatch((-2, 3.68), 304, 1.95, boxstyle="round,pad=0.08",
                       linewidth=2, edgecolor='#E74C3C', facecolor='#FDEDEC',
                       linestyle='--', alpha=0.5, zorder=2)
ax1.add_patch(rect)
ax1.text(152, 5.95, '并行执行 → 自适应零额外延迟', fontsize=10,
         color='#E74C3C', ha='center', fontweight='bold',
         bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='#E74C3C', alpha=0.9))

ax1.axvline(x=318, color='#2C3E50', linestyle='-.', linewidth=1.8, zorder=2)
ax1.text(318, -0.8, '总延迟 ≈ 320 cyc = 3.2 μs\n@ 100 MHz → 312 kHz',
         fontsize=10, fontweight='bold', color='#2C3E50', ha='center', va='top',
         bbox=dict(boxstyle='round,pad=0.3', facecolor='#EBF5FB', edgecolor='#2C3E50'))

ax1.set_yticks(y_positions)
ax1.set_yticklabels([m[0] for m in modules], fontsize=10)
ax1.set_xlabel('时钟周期 (cycles @ 100 MHz)', fontsize=11)
ax1.set_xlim(-15, 380)
ax1.set_ylim(-1.5, 6.8)
ax1.set_title('(a) 单次流水线时序', fontsize=13, pad=10)
ax1.grid(axis='x', alpha=0.25, zorder=0)
ax1.spines['top'].set_visible(False)
ax1.spines['right'].set_visible(False)

# ============================================================
# (b) 各模块延迟条形图
# ============================================================
bar_names = ['FK+\nJacobian', '速度\n估计', '阻抗\n控制', 'J^T\n映射', 'τ→Iq\n转换', '自适应\nRLS']
bar_cycles = [300, 4, 3, 8, 3, 48]
bar_colors = ['#4ECDC4', '#FFD93D', '#FF6B6B', '#6C5CE7', '#A8E6CF', '#E8A0BF']
total = 320

bars = ax2.bar(np.arange(len(bar_names)), bar_cycles, color=bar_colors,
               edgecolor='#333333', linewidth=1.2, width=0.65, zorder=3)
for i, (b, c) in enumerate(zip(bars, bar_cycles)):
    pct = c / total * 100
    ax2.text(b.get_x() + b.get_width()/2, b.get_height() + 5,
            f'{c}\n({pct:.1f}%)', ha='center', va='bottom',
            fontsize=9, fontweight='bold', color='#333333')

ax2.set_xticks(np.arange(len(bar_names)))
ax2.set_xticklabels(bar_names, fontsize=9)
ax2.set_ylabel('延迟 (周期)', fontsize=11)
ax2.set_title('(b) 各模块延迟', fontsize=13, pad=10)
ax2.set_ylim(0, 380)
ax2.grid(axis='y', alpha=0.25, zorder=0)
ax2.spines['top'].set_visible(False)
ax2.spines['right'].set_visible(False)
ax2.annotate('瓶颈\n(93.8%)', xy=(0, 300), xytext=(1.2, 340),
            fontsize=10, fontweight='bold', color='#E74C3C',
            arrowprops=dict(arrowstyle='->', color='#E74C3C', lw=2), ha='center')

# ============================================================
# (c) 7:1 频率冗余时序图
# ============================================================
foc_period = 27.78   # μs
imp_period = 3.95    # μs
n_imp = 7

# FOC 周期大条
ax3.barh(2, foc_period, left=0, height=0.7, color='#3498DB', alpha=0.25,
         edgecolor='#2980B9', linewidth=2, zorder=2)
ax3.text(foc_period/2, 2, f'1 个 FOC 周期 = {foc_period} μs  (36 kHz)',
         ha='center', va='center', fontsize=12, fontweight='bold', color='#2980B9')

# 7 次控制流水线迭代，每次输出一个 i_q*
pipe_colors_7 = ['#4ECDC4', '#45B7AA', '#3DA192', '#358B7A', '#2D7562', '#255F4A', '#1D4932']
for k in range(n_imp):
    x0 = k * imp_period
    ax3.barh(1, imp_period * 0.92, left=x0, height=0.55, color=pipe_colors_7[k],
             edgecolor='#333333', linewidth=1, zorder=3)
    # 标注每次输出的 i_q*
    ax3.text(x0 + imp_period/2, 1, f'i_q*[{k+1}]', ha='center', va='center',
            fontsize=9, fontweight='bold', color='white')

ax3.text(foc_period/2, 0.45, f'控制流水线 × 7 次  (每次 {imp_period} μs → 253 kHz)',
         ha='center', va='center', fontsize=11, fontweight='bold', color='#1D4932')

# 预测器标注：在 FOC 采样时刻，利用最近两个 i_q* 做线性外推
ax3.annotate('预测器：利用 i_q*[n] 与 i_q*[n-1]\n线性外推 → i_q,pred',
            xy=(foc_period, 1), xytext=(foc_period + 1.5, 0.3),
            fontsize=9, fontweight='bold', color='#E67E22',
            arrowprops=dict(arrowstyle='->', color='#E67E22', lw=2),
            bbox=dict(boxstyle='round,pad=0.3', facecolor='#FDF2E9', edgecolor='#E67E22'))

# 自适应标注：与 FK 并行，不额外占用
ax3.text(1 * imp_period + imp_period/2, 0.05, '自适应RLS\n与FK并行', ha='center', va='top',
        fontsize=8, color='#8E44AD', style='italic', fontweight='bold')

# 功能分配标注（简化）
annotations = []
for k, (idx, label) in enumerate(annotations):
    x0 = idx * imp_period
    ax3.text(x0 + imp_period/2, 0.05, label, ha='center', va='top',
            fontsize=8, color='#555555', style='italic')

# 7:1 标注箭头
ax3.annotate('', xy=(0, 1.55), xytext=(0, 1.7),
            arrowprops=dict(arrowstyle='-', color='#E74C3C', lw=1.5))
ax3.annotate('', xy=(foc_period, 1.55), xytext=(foc_period, 1.7),
            arrowprops=dict(arrowstyle='-', color='#E74C3C', lw=1.5))
ax3.annotate('', xy=(0, 1.65), xytext=(foc_period, 1.65),
            arrowprops=dict(arrowstyle='<->', color='#E74C3C', lw=2))
ax3.text(foc_period/2, 1.78, '7 : 1 频率比', ha='center', va='bottom',
         fontsize=12, fontweight='bold', color='#E74C3C',
         bbox=dict(boxstyle='round,pad=0.3', facecolor='#FDEDEC', edgecolor='#E74C3C'))

# FOC 采样点标注
ax3.axvline(x=0, color='#3498DB', linestyle=':', linewidth=1.5, ymin=0.1, ymax=0.95, zorder=1)
ax3.axvline(x=foc_period, color='#3498DB', linestyle=':', linewidth=1.5, ymin=0.1, ymax=0.95, zorder=1)
ax3.text(-0.3, 2.55, 'FOC\n采样', fontsize=8, color='#2980B9', ha='center', va='bottom')
ax3.text(foc_period+0.3, 2.55, 'FOC\n采样', fontsize=8, color='#2980B9', ha='center', va='bottom')

ax3.set_xlim(-2, 32)
ax3.set_ylim(-0.5, 3.0)
ax3.set_yticks([])
ax3.set_xlabel('时间 (μs)', fontsize=12)
ax3.set_title('(c) 7:1 频率冗余 — 单个 FOC 周期内控制流水线输出 7 次 i_q* 供预测器外推', fontsize=13, pad=10)
ax3.spines['top'].set_visible(False)
ax3.spines['right'].set_visible(False)
ax3.spines['left'].set_visible(False)
ax3.grid(axis='x', alpha=0.2, zorder=0)

plt.savefig('D:/papercode/figures/fig_4_2_pipeline.png', dpi=200, bbox_inches='tight',
            facecolor='white', edgecolor='none')
print('saved')
