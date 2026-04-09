#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Signal timing diagram for impedance control + FOC + prediction.
Style: dark-navy signals, orange dashed markers, blue arrows, green callouts.
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch

# ── Style ──────────────────────────────────────────────────────────────────
C_SIG   = '#0D3B7A'   # dark navy - signal lines
C_PRED  = '#0D3B7A'   # prediction signal (same navy)
C_NP    = '#0D3B7A'   # no-prediction signal
C_TRUE  = '#888888'   # thin grey - true Iq (background reference)
C_DASH  = '#FF8C00'   # orange dashed vertical markers
C_ARROW = '#1565C0'   # blue arrows
C_GREEN = '#1a7a3a'   # green callout edge
C_GBKG  = '#E8F5E9'   # green callout fill
C_RED   = '#C62828'   # red error bars
C_BLUE  = '#1565C0'   # blue prediction markers

plt.rcParams.update({
    'font.family': 'sans-serif',
    'font.size': 9,
})

fig, ax = plt.subplots(figsize=(14, 7.5))
ax.set_xlim(-4.5, 80)
ax.set_ylim(-0.8, 11.5)
ax.axis('off')

# ── Timing parameters (display units, 1 unit ≈ 1 μs) ─────────────────────
T_FOC = 28          # FOC period (27.78 μs)
T_IMP = T_FOC / 7  # Impedance period (3.97 μs), ratio=7
N_FOC = 2           # show 2 full FOC periods + small overhang
t_total = N_FOC * T_FOC + 6

# FOC period boundaries
foc_edges = [i * T_FOC for i in range(N_FOC + 1)]   # [0, 28, 56]

# Imp_done pulse times within each FOC period (7 per period)
imp_times = []
for f in range(N_FOC):
    for k in range(7):
        imp_times.append(f * T_FOC + (k + 1) * T_IMP)

# True Iq reference: cosine curve (0.10 → 0.90) — non-linear to show prediction benefit
def true_iq(t_us):
    return 0.50 - 0.40 * np.cos(np.pi * t_us / (N_FOC * T_FOC))

t_hires = np.linspace(0, t_total, 2000)
iq_hires = np.array([true_iq(t) for t in t_hires])   # 0–1 range

# Iq values at each imp_done event
imp_t_arr = np.array(imp_times)
iq_imp    = np.array([true_iq(t) for t in imp_t_arr])

# Iq (no prediction): steps only at FOC boundaries — holds last imp value
def iq_no_pred(t_us):
    # use value computed at the IMP just before or at the FOC boundary
    foc_idx = int(t_us // T_FOC)
    # last imp_done before this FOC start
    boundary = foc_idx * T_FOC
    before = [v for tt, v in zip(imp_times, iq_imp) if tt <= boundary]
    return before[-1] if before else iq_imp[0]

# Iq (with prediction): linear extrapolation from last two imp values to next FOC boundary
def iq_predict_at_foc(foc_boundary):
    before = [(tt, v) for tt, v in zip(imp_times, iq_imp) if tt <= foc_boundary]
    if len(before) < 2:
        return before[-1][1] if before else 0.5
    t1, v1 = before[-2]
    t2, v2 = before[-1]
    slope = (v2 - v1) / (t2 - t1)
    return v2 + slope * (foc_boundary - t2)

# ── Y positions for each signal row ───────────────────────────────────────
# (bottom to top)
ROW = {
    'iq_pred':  {'y': 0.4,  'h': 1.3},
    'iq_np':    {'y': 2.4,  'h': 1.3},
    'imp_done': {'y': 4.4,  'h': 0.7},
    'foc_per':  {'y': 5.8,  'h': 0.7},
    'sys_clk':  {'y': 7.2,  'h': 0.55},
}

def sig_low(row):  return ROW[row]['y']
def sig_high(row): return ROW[row]['y'] + ROW[row]['h']
def sig_mid(row):  return ROW[row]['y'] + ROW[row]['h'] / 2

# ── Helper: draw binary signal (list of (t, level) transitions) ───────────
def draw_signal(transitions, row, color=C_SIG, lw=1.8, zorder=3):
    y0, y1 = sig_low(row), sig_high(row)
    pts = list(transitions)
    xs, ys = [], []
    for i, (t, lvl) in enumerate(pts):
        y = y1 if lvl else y0
        if i == 0:
            xs.append(t); ys.append(y)
        else:
            # vertical then horizontal
            xs.append(t); ys.append(ys[-1])   # horizontal
            xs.append(t); ys.append(y)         # vertical
    ax.plot(xs, ys, color=color, lw=lw, zorder=zorder, solid_capstyle='butt')

# ── Helper: draw analog staircase ─────────────────────────────────────────
def draw_staircase(step_times, step_vals, row, color=C_SIG, lw=1.9,
                   t_end=t_total, zorder=3):
    y0   = sig_low(row)
    h    = ROW[row]['h']
    vmin, vmax = 0.0, 1.0
    xs, ys = [], []
    for i, (t, v) in enumerate(zip(step_times, step_vals)):
        yv = y0 + (v - vmin) / (vmax - vmin) * h
        if i == 0:
            xs += [0, t]; ys += [yv, yv]
        else:
            xs.append(t); ys.append(ys[-1])
            xs.append(t); ys.append(yv)
    # extend to t_end
    xs.append(t_end); ys.append(ys[-1])
    ax.plot(xs, ys, color=color, lw=lw, zorder=zorder, solid_capstyle='butt')

# ── Helper: thin reference curve ──────────────────────────────────────────
def draw_ref_curve(row, color=C_TRUE, lw=0.9, alpha=0.45):
    y0 = sig_low(row)
    h  = ROW[row]['h']
    ys = y0 + iq_hires * h
    ax.plot(t_hires, ys, color=color, lw=lw, alpha=alpha, zorder=2,
            linestyle='--')

# ══════════════════════════════════════════════════════════════════════════
# 1. sys_clk  (100 MHz → period=0.01 μs; show representative fast pulses)
# ══════════════════════════════════════════════════════════════════════════
CLK_P = 0.8   # display period for clock (just for visual, not to scale)
clk_trans = []
t = 0.0
lvl = 0
while t < t_total + 0.5:
    clk_trans.append((t, lvl))
    lvl = 1 - lvl
    t += CLK_P / 2
draw_signal(clk_trans, 'sys_clk', lw=1.3)

# ══════════════════════════════════════════════════════════════════════════
# 2. FOC period signal (square wave, alternates each FOC period)
# ══════════════════════════════════════════════════════════════════════════
foc_trans = [(0, 0)]
for i, edge in enumerate(foc_edges[1:]):
    foc_trans.append((edge, i % 2 + 0))
# make it cleaner: just show a flat high for each period with gaps
y0_foc = sig_low('foc_per')
y1_foc = sig_high('foc_per')
for i in range(N_FOC + 1):
    t0 = i * T_FOC
    t1 = min(t0 + T_FOC, t_total)
    if i < N_FOC:
        ax.plot([t0, t1], [y1_foc, y1_foc], color=C_SIG, lw=1.8, zorder=3,
                solid_capstyle='butt')
        ax.plot([t0, t0], [y0_foc, y1_foc], color=C_SIG, lw=1.8, zorder=3)
        ax.plot([t1, t1], [y0_foc, y1_foc], color=C_SIG, lw=1.8, zorder=3)
        ax.plot([t0, t0], [y0_foc, y0_foc], color=C_SIG, lw=1.8, zorder=3)
        ax.plot([t0, t1], [y0_foc, y0_foc], color=C_SIG, lw=1.8, zorder=3)
        # fill lightly
        rect = mpatches.Rectangle((t0, y0_foc), t1 - t0, y1_foc - y0_foc,
                                   fc='#D6E4F7', ec='none', zorder=2)
        ax.add_patch(rect)

# ══════════════════════════════════════════════════════════════════════════
# 3. imp_done  (narrow positive pulses)
# ══════════════════════════════════════════════════════════════════════════
PW = 0.35   # pulse width
y0_imp = sig_low('imp_done')
y1_imp = sig_high('imp_done')
# baseline
ax.plot([0, t_total], [y0_imp, y0_imp], color=C_SIG, lw=1.8, zorder=3)
for tt in imp_times:
    ax.plot([tt, tt],         [y0_imp, y1_imp], color=C_SIG, lw=1.8, zorder=3)
    ax.plot([tt, tt + PW],    [y1_imp, y1_imp], color=C_SIG, lw=1.8, zorder=3)
    ax.plot([tt + PW, tt + PW],[y1_imp, y0_imp], color=C_SIG, lw=1.8, zorder=3)

# ══════════════════════════════════════════════════════════════════════════
# 4. Iq (no prediction): staircase, updates at FOC boundaries
# ══════════════════════════════════════════════════════════════════════════
draw_ref_curve('iq_np')

np_step_times = foc_edges[:-1]   # update at each FOC boundary start
np_step_vals  = []
for tb in np_step_times:
    # last imp_done at or before this boundary
    before = [v for tt, v in zip(imp_times, iq_imp) if tt <= tb + 0.01]
    np_step_vals.append(before[-1] if before else true_iq(0))
# first step at t=0
np_step_times = [0] + list(np_step_times[1:])
np_step_vals  = [true_iq(0)] + np_step_vals[1:]

draw_staircase(np_step_times, np_step_vals, 'iq_np', color=C_NP)

# Error markers at FOC boundaries (red bar showing error vs true)
for tb in foc_edges[1:-1]:
    true_v = true_iq(tb)
    pred_v = iq_no_pred(tb)
    y0r = sig_low('iq_np')
    h   = ROW['iq_np']['h']
    yt = y0r + true_v * h
    yp = y0r + pred_v * h
    ax.annotate('', xy=(tb, yt), xytext=(tb, yp),
                arrowprops=dict(arrowstyle='<->', color=C_RED, lw=1.5))

# ══════════════════════════════════════════════════════════════════════════
# 5. Iq (with prediction): staircase updates at every imp_done
# ══════════════════════════════════════════════════════════════════════════
draw_ref_curve('iq_pred')

draw_staircase(imp_times, list(iq_imp), 'iq_pred', color=C_PRED)

# At each FOC boundary, show predicted point (circle marker) vs true
for tb in foc_edges[1:2]:   # show for first boundary only (cleaner)
    pred_v = iq_predict_at_foc(tb)
    true_v = true_iq(tb)
    y0p = sig_low('iq_pred')
    h   = ROW['iq_pred']['h']
    yt = y0p + true_v * h
    yp = y0p + pred_v * h
    ax.plot(tb, yp, 'o', color=C_BLUE, ms=6, zorder=6, mew=1.5,
            mfc='white')
    ax.plot(tb, yt, 'o', color=C_TRUE, ms=5, zorder=6, mfc=C_TRUE)
    ax.annotate('', xy=(tb, yt), xytext=(tb, yp),
                arrowprops=dict(arrowstyle='<->', color=C_BLUE, lw=1.2))

# ══════════════════════════════════════════════════════════════════════════
# Signal labels (left side)
# ══════════════════════════════════════════════════════════════════════════
label_x = -0.5
label_info = [
    ('sys_clk',   'sys_clk',       'sys_clk'),
    ('foc_per',   'FOC period',    'foc_per'),
    ('imp_done',  'imp_done',      'imp_done'),
    ('iq_np',     '$i_q$ (no pred.)', 'iq_np'),
    ('iq_pred',   '$i_q$ (predict)',  'iq_pred'),
]
for row, label, _ in label_info:
    ax.text(label_x, sig_mid(row), label,
            ha='right', va='center', fontsize=9.5, fontweight='bold',
            color=C_SIG)

# ══════════════════════════════════════════════════════════════════════════
# Orange dashed vertical markers
# ══════════════════════════════════════════════════════════════════════════
def vdash(t, ybot=0.0, ytop=9.5):
    ax.plot([t, t], [ybot, ytop], color=C_DASH, lw=1.2,
            linestyle='--', zorder=1, alpha=0.85)

# At FOC boundaries
for edge in foc_edges:
    vdash(edge, ybot=-0.3, ytop=sig_high('sys_clk') + 0.3)

# At a few imp_done events (within first FOC period) for illustration
for k in range(7):
    tt = imp_times[k]
    vdash(tt, ybot=sig_low('imp_done') - 0.15,
          ytop=sig_high('imp_done') + 0.15)

# ══════════════════════════════════════════════════════════════════════════
# Blue timing arrows
# ══════════════════════════════════════════════════════════════════════════
def time_arrow(t1, t2, y, label, above=True, color=C_ARROW, fs=8.5):
    ax.annotate('', xy=(t2, y), xytext=(t1, y),
                arrowprops=dict(arrowstyle='<->', color=color, lw=1.5),
                zorder=5)
    offset = 0.32 if above else -0.32
    ax.text((t1 + t2) / 2, y + offset, label,
            ha='center', va='center', fontsize=fs, color=color,
            fontweight='bold')

# T_FOC arrow (between first and second FOC boundary)
y_arrow_foc = sig_high('sys_clk') + 0.55
time_arrow(0, T_FOC, y_arrow_foc,
           r'$T_{FOC}$ = 27.78 μs', above=True)

# T_IMP arrow (first imp_done interval)
y_arrow_imp = sig_low('imp_done') - 0.45
time_arrow(imp_times[0], imp_times[1], y_arrow_imp,
           r'$T_{imp}$ = 3.95 μs', above=False, fs=7.5)

# ══════════════════════════════════════════════════════════════════════════
# Green callout bubbles
# ══════════════════════════════════════════════════════════════════════════
def callout(x, y, text, width=7.0, height=0.85):
    box = FancyBboxPatch((x - width/2, y - height/2), width, height,
                         boxstyle='round,pad=0.15',
                         facecolor=C_GBKG, edgecolor=C_GREEN,
                         linewidth=1.4, zorder=7)
    ax.add_patch(box)
    ax.text(x, y, text, ha='center', va='center',
            fontsize=8.5, color=C_GREEN, fontweight='bold', zorder=8)

# ══════════════════════════════════════════════════════════════════════════
# Comparison snapshot at t_cmp (midpoint of second FOC period)
# ══════════════════════════════════════════════════════════════════════════
t_cmp = T_FOC + T_FOC * 0.5   # t = 42 (midpoint of 2nd FOC period)

v_true  = true_iq(t_cmp)           # true iq at that moment

# no-pred: holds iq from the last FOC boundary (t=28)
np_val_at_28 = [v for tt, v in zip(imp_times, iq_imp) if tt <= T_FOC + 0.01]
v_np    = np_val_at_28[-1]          # still holding this value at t_cmp

# prediction: last imp_done before t_cmp, then linear extrapolation
before_cmp = [(tt, v) for tt, v in zip(imp_times, iq_imp) if tt <= t_cmp]
t1c, v1c = before_cmp[-2]
t2c, v2c = before_cmp[-1]
slope_c  = (v2c - v1c) / (t2c - t1c)
v_pred   = v2c + slope_c * (t_cmp - t2c)

# ── Draw vertical comparison marker ──────────────────────────────────────
vdash(t_cmp, ybot=sig_low('iq_pred') - 0.2,
      ytop=sig_high('iq_np') + 0.2)

# Dot helper
def snap_dot(row, val, color, marker, ms=7):
    y0 = sig_low(row)
    h  = ROW[row]['h']
    yv = y0 + val * h
    ax.plot(t_cmp, yv, marker, color=color, ms=ms, zorder=8,
            mfc='white' if marker == 'o' else color, mew=2.0)
    return yv

# On iq_np row: true (grey) and no-pred (red)
y_np_true = snap_dot('iq_np', v_true, C_TRUE,  's', ms=6)
y_np_held = snap_dot('iq_np', v_np,   C_RED,   'x', ms=8)

# On iq_pred row: true (grey) and predicted (blue)
y_pr_true = snap_dot('iq_pred', v_true, C_TRUE, 's', ms=6)
y_pr_pred = snap_dot('iq_pred', v_pred, C_BLUE, 'o', ms=7)

# ── Value labels (right of t_cmp) ─────────────────────────────────────────
lx = t_cmp + 1.2
fs_val = 8.0

# iq_np row labels
ax.text(lx, y_np_true, f'True = {v_true:.3f}',
        va='center', fontsize=fs_val, color='#555', fontweight='bold')
ax.text(lx, y_np_held, f'No-pred = {v_np:.3f}',
        va='center', fontsize=fs_val, color=C_RED,  fontweight='bold')

# iq_pred row labels
ax.text(lx, y_pr_true, f'True = {v_true:.3f}',
        va='center', fontsize=fs_val, color='#555', fontweight='bold')
ax.text(lx, y_pr_pred, f'Predict = {v_pred:.3f}',
        va='center', fontsize=fs_val, color=C_BLUE, fontweight='bold')

# ── Error arrows ───────────────────────────────────────────────────────────
ax.annotate('', xy=(t_cmp - 0.5, y_np_true), xytext=(t_cmp - 0.5, y_np_held),
            arrowprops=dict(arrowstyle='<->', color=C_RED, lw=1.6), zorder=9)
err_np = abs(v_true - v_np)
ax.text(t_cmp - 2.2, (y_np_true + y_np_held) / 2,
        f'Δ={err_np:.3f}', ha='center', va='center',
        fontsize=7.5, color=C_RED, fontweight='bold')

ax.annotate('', xy=(t_cmp - 0.5, y_pr_true), xytext=(t_cmp - 0.5, y_pr_pred),
            arrowprops=dict(arrowstyle='<->', color=C_BLUE, lw=1.6), zorder=9)
err_pred = abs(v_true - v_pred)
ax.text(t_cmp - 2.2, (y_pr_true + y_pr_pred) / 2,
        f'Δ={err_pred:.4f}', ha='center', va='center',
        fontsize=7.5, color=C_BLUE, fontweight='bold')

# ══════════════════════════════════════════════════════════════════════════
# Axis baseline
# ══════════════════════════════════════════════════════════════════════════
ax.axhline(-0.1, color='#CCCCCC', lw=0.5, zorder=0)

# ══════════════════════════════════════════════════════════════════════════
# Title
# ══════════════════════════════════════════════════════════════════════════
ax.text(31, 11.0,
        'Signal Timing: Impedance Control, FOC, and Prediction',
        ha='center', va='center', fontsize=13, fontweight='bold',
        color='#111')
ax.text(31, 10.5,
        r'$f_{imp}$ = 253.2 kHz,   $f_{FOC}$ = 36 kHz,   ratio = 7',
        ha='center', va='center', fontsize=10, color='#555')

# ══════════════════════════════════════════════════════════════════════════
# Legend (bottom)
# ══════════════════════════════════════════════════════════════════════════
legend_y = -0.55
ax.plot([20, 22.5], [legend_y, legend_y], color=C_TRUE, lw=1.2,
        linestyle='--', alpha=0.6)
ax.text(23, legend_y, 'True $i_q$ reference',
        va='center', fontsize=8.5, color='#555')
ax.plot([33, 35.5], [legend_y, legend_y], color=C_RED, lw=1.5)
ax.annotate('', xy=(35.5, legend_y + 0.15), xytext=(35.5, legend_y - 0.15),
            arrowprops=dict(arrowstyle='->', color=C_RED, lw=1.2))
ax.text(36, legend_y, 'Error (no pred.)',
        va='center', fontsize=8.5, color=C_RED)
ax.plot(46, legend_y, 'o', color=C_BLUE, ms=6, mfc='white', mew=1.5)
ax.text(47.2, legend_y, 'Predicted point',
        va='center', fontsize=8.5, color=C_BLUE)

plt.tight_layout(pad=0.3)
out = 'D:/robot/impedance_control/figures/fig.png'
fig.savefig(out, dpi=300, bbox_inches='tight', facecolor='white')
print(f'Saved: {out}')
plt.close()
