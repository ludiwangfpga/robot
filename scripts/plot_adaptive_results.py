#!/usr/bin/env python3
"""plot_adaptive_results.py - Parse VCD and plot adaptive impedance control results.

Reads tb_adaptive_impedance.vcd and generates:
  Row 1: K_env estimated vs true (stiffness tracking)
  Row 2: Ks_out adaptive stiffness
  Row 3: Ds_out adaptive damping

Output: D:/robot/impedance_control/figures/fig_adaptive_impedance.png
"""

import struct, re, sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

# ── VCD Parser (reused from plot_vcd.py) ────────────────────────────────────
VCD_FILE = Path(r"D:/robot/impedance_control/sim_prediction/tb_adaptive_impedance.vcd")
OUT_FILE = Path(r"D:/robot/impedance_control/figures/fig_adaptive_impedance.png")

signals = {}   # name -> token
tokens  = {}   # token -> name
changes = {}   # token -> [(time, value), ...]

def parse_vcd(path):
    scope_stack = []
    with open(path, 'r') as f:
        cur_time = 0
        in_defs = True
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith('$scope'):
                parts = line.split()
                if len(parts) >= 3:
                    scope_stack.append(parts[2])
            elif line.startswith('$upscope'):
                if scope_stack:
                    scope_stack.pop()
            elif line.startswith('$var'):
                parts = line.split()
                tok = parts[3]
                name = parts[4]
                full = '.'.join(scope_stack + [name]) if scope_stack else name
                # Use short name for convenience
                signals[name] = tok
                tokens[tok] = name
                changes[tok] = []
            elif line.startswith('$enddefinitions'):
                in_defs = False
            elif line.startswith('#'):
                cur_time = int(line[1:])
            elif not in_defs:
                if line.startswith('b'):
                    parts = line.split()
                    bval = parts[0][1:]
                    tok = parts[1]
                    if tok in changes:
                        # Convert binary string to signed 32-bit int
                        if 'x' in bval or 'z' in bval or 'X' in bval or 'Z' in bval:
                            val = 0
                        else:
                            val = int(bval, 2)
                            if len(bval) >= 32 and bval[0] == '1':
                                val -= (1 << len(bval))
                        changes[tok].append((cur_time, val))
                elif line[0] in '01xzXZ':
                    val = 1 if line[0] == '1' else 0
                    tok = line[1:]
                    if tok in changes:
                        changes[tok].append((cur_time, val))

parse_vcd(VCD_FILE)

Q16 = 65536.0

def get_signal_trace(name, q=Q16):
    """Get time (μs) and value arrays for a signal."""
    tok = signals.get(name)
    if not tok or not changes[tok]:
        return [], []
    ts, vs = [], []
    for t, v in changes[tok]:
        ts.append(t / 1e6)  # ps → μs
        vs.append(v / q)
    return ts, vs

def step_plot(ax, ts, vs, **kwargs):
    """Step plot from change list."""
    if not ts:
        return
    ax.step(ts, vs, where='post', **kwargs)

# ── Extract signals ─────────────────────────────────────────────────────────
# Signal names changed to joint-space: _x → _0
t_kenv, v_kenv = get_signal_trace('K_env_est_0')
t_ks, v_ks     = get_signal_trace('Ks_out_0')
t_ds, v_ds     = get_signal_trace('Ds_out_0')
t_done, v_done = get_signal_trace('done', q=1)

# ── Iteration-based resampling ──────────────────────────────────────────────
# Each iteration: start pulse → done pulse. We want values at done edges.
# Find done rising edges
done_times = []
for i in range(1, len(changes[signals['done']])):
    t_prev, v_prev = changes[signals['done']][i-1]
    t_cur, v_cur   = changes[signals['done']][i]
    if v_prev == 0 and v_cur == 1:
        done_times.append(t_cur)

def sample_at_done(name, q=Q16):
    """Sample signal value at each done pulse."""
    tok = signals.get(name)
    if not tok:
        return []
    ch = changes[tok]
    vals = []
    ci = 0
    for dt in done_times:
        while ci < len(ch) - 1 and ch[ci+1][0] <= dt:
            ci += 1
        vals.append(ch[ci][1] / q)
    return vals

kenv_vals = sample_at_done('K_env_est_0')
ks_vals   = sample_at_done('Ks_out_0')
ds_vals   = sample_at_done('Ds_out_0')

n_iter = len(kenv_vals)
iters = list(range(n_iter))

# ── True K_env reference ────────────────────────────────────────────────────
kenv_true = []
for i in range(n_iter):
    if i < 200:
        kenv_true.append(0.0)  # free space (but dead-zone → frozen)
    elif i < 500:
        kenv_true.append(5.0)  # hard wall
    else:
        kenv_true.append(0.2)  # soft sponge

# ── Plot ────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
fig.suptitle('Adaptive Impedance Control: Online RLS Stiffness Estimation\n'
             '$K_{s,nom}=1.0$ N/m,  $D_{s,nom}=0.2$ Ns/m,  $\\lambda=0.98$',
             fontsize=13, fontweight='bold')

C_TRUE = '#888888'
C_EST  = '#1f77b4'
C_KS   = '#d62728'
C_DS   = '#2ca02c'

# Phase shading
for ax in axes:
    ax.axvspan(0, 200, alpha=0.08, color='green', label='_nolegend_')
    ax.axvspan(200, 500, alpha=0.08, color='red', label='_nolegend_')
    ax.axvspan(500, n_iter, alpha=0.08, color='blue', label='_nolegend_')
    ax.grid(True, alpha=0.3)

# Row 0: K_env estimation tracking
axes[0].plot(iters, kenv_true, '--', color=C_TRUE, lw=2, label='$K_{env}$ true')
axes[0].plot(iters, kenv_vals, '-', color=C_EST, lw=1.5, label='$K_{env}$ estimated (RLS)')
axes[0].set_ylabel('$K_{env}$ (N/m)', fontsize=11)
axes[0].legend(fontsize=10, loc='upper right')
axes[0].set_ylim([-0.5, 6])
# Phase labels
axes[0].text(100, 5.3, 'Free Space\n(dead-zone)', ha='center', fontsize=9, color='green')
axes[0].text(350, 5.3, 'Hard Wall\n$K_{env}=5.0$', ha='center', fontsize=9, color='red')
axes[0].text(650, 5.3, 'Soft Sponge\n$K_{env}=0.2$', ha='center', fontsize=9, color='blue')

# Row 1: Adaptive Ks
axes[1].axhline(y=1.0, color=C_TRUE, ls='--', lw=1.5, label='$K_{s,nom}=1.0$')
axes[1].plot(iters, ks_vals, '-', color=C_KS, lw=1.5, label='$K_{s,out}$ (adaptive)')
axes[1].set_ylabel('$K_s$ (N/m)', fontsize=11)
axes[1].legend(fontsize=10, loc='upper right')
axes[1].set_ylim([-0.2, 5.5])

# Row 2: Adaptive Ds
axes[2].axhline(y=0.2, color=C_TRUE, ls='--', lw=1.5, label='$D_{s,nom}=0.2$')
axes[2].plot(iters, ds_vals, '-', color=C_DS, lw=1.5, label='$D_{s,out}$ (adaptive)')
axes[2].set_ylabel('$D_s$ (Ns/m)', fontsize=11)
axes[2].set_xlabel('Iteration (253 kHz update rate)', fontsize=11)
axes[2].legend(fontsize=10, loc='upper right')
axes[2].set_ylim([-0.05, 1.1])

plt.tight_layout()
OUT_FILE.parent.mkdir(parents=True, exist_ok=True)
plt.savefig(str(OUT_FILE), dpi=200, bbox_inches='tight')
print(f"Saved: {OUT_FILE}")
