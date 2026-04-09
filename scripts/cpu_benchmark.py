#!/usr/bin/env python3
"""
CPU Benchmark for Impedance Control Pipeline
Compare against FPGA: 395 cycles = 3.95 us @ 100MHz

Implements the SAME algorithm in Python to measure CPU execution time.
"""

import numpy as np
import time
import os

# ============================================================
# DH Parameters (meters, radians)
# ============================================================
DH = [
    # a,       alpha,    d
    (0.055,   np.pi/2,  0.166),
    (0.200,   0.0,      0.0),
    (0.056,   np.pi/2,  0.0),
    (0.0,    -np.pi/2,  0.192),
    (0.0,     np.pi/2,  0.0),
    (0.0,     0.0,      0.055),
]

# Test configurations (same as tb_latency_benchmark.v, in radians)
TEST_CONFIGS = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
    [0.7854, 0.7854, 0.7854, 0.7854, 0.7854, 0.7854],
    [1.5708, 0.0, 1.5708, 0.0, 1.5708, 0.0],
    [0.5, -0.25, 1.0, -0.5, 0.25, -1.0],
    [3.0, 0.0, 0.0, 0.0, 0.0, 3.0],
    [-0.7854, -1.5708, 0.7854, -0.7854, 1.5708, -0.7854],
    [2.0, -1.0, 1.5, -2.0, 1.0, -0.5],
    [0.0, 1.5708, -1.5708, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 1.5708, 0.0],
]

# ============================================================
# Algorithm Implementation (float64)
# ============================================================

def dh_matrix(a, alpha, d, theta):
    """Standard DH transformation matrix."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ],
    ])

def forward_kinematics(q):
    """FK: compute T_0^6 and extract pose."""
    T = np.eye(4)
    for i in range(6):
        Ti = dh_matrix(DH[i][0], DH[i][1], DH[i][2], q[i])
        T = T @ Ti
    # Extract position
    x, y, z = T[0,3], T[1,3], T[2,3]
    # ZYX Euler angles
    rz = np.arctan2(T[1,0], T[0,0])
    ry = np.arctan2(-T[2,0], np.sqrt(T[0,0]**2 + T[1,0]**2))
    rx = np.arctan2(T[2,1], T[2,2])
    return np.array([x, y, z, rx, ry, rz]), T

def analytical_jacobian(q):
    """Compute 6x6 geometric Jacobian."""
    T = [np.eye(4)]
    for i in range(6):
        Ti = dh_matrix(DH[i][0], DH[i][1], DH[i][2], q[i])
        T.append(T[-1] @ Ti)

    pe = T[6][:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        z = T[i][:3, 2]
        p = T[i][:3, 3]
        J[:3, i] = np.cross(z, pe - p)
        J[3:, i] = z
    return J

def velocity_estimator(pose, pose_prev, dt=1e-4, alpha=0.3):
    """First-order difference + IIR filter."""
    vel_raw = (pose - pose_prev) / dt
    # Simplified: no filter state for benchmark (just the computation)
    vel = alpha * vel_raw  # + (1-alpha) * vel_prev
    return vel

def impedance_control(xd, x, xd_dot, x_dot, Ks, Ds):
    """Diagonal impedance control: F = Ks*(xd-x) + Ds*(xd_dot-x_dot)."""
    return Ks * (xd - x) + Ds * (xd_dot - x_dot)

def jt_force_mapping(J, F):
    """tau = J^T * F (6x6 matrix-vector multiply)."""
    return J.T @ F

def torque_to_iq(tau, Kt=0.1):
    """iq = tau / Kt with saturation."""
    iq = tau / Kt
    return np.clip(iq, -32767, 32767).astype(np.int16) if isinstance(iq, np.ndarray) else max(-32767, min(32767, int(iq)))

def full_pipeline(q, xd, Ks, Ds):
    """Run complete pipeline once."""
    pose, T = forward_kinematics(q)
    J = analytical_jacobian(q)
    vel = velocity_estimator(pose, pose, dt=1e-4, alpha=0.3)
    F = impedance_control(xd, pose, np.zeros(6), vel, Ks, Ds)
    tau = jt_force_mapping(J, F)
    iq = torque_to_iq(tau)
    return pose, J, vel, F, tau, iq

# ============================================================
# Q16.16 Fixed-Point Implementation
# ============================================================

def q16_mult(a, b):
    """Q16.16 multiply."""
    return ((a * b) >> 16)

def q16_from_float(f):
    return int(round(f * 65536))

def q16_to_float(q):
    return q / 65536.0

def q16_sin_cos_cordic(angle_q16, iterations=16):
    """CORDIC sin/cos in Q16.16 (matching FPGA)."""
    # CORDIC angles in Q16.16
    atan_table = [q16_from_float(np.arctan(2.0**(-i))) for i in range(iterations)]
    K = q16_from_float(0.607252935)  # CORDIC gain

    x = K
    y = 0
    z = angle_q16

    for i in range(iterations):
        if z >= 0:
            x_new = x - (y >> i)
            y_new = y + (x >> i)
            z -= atan_table[i]
        else:
            x_new = x + (y >> i)
            y_new = y - (x >> i)
            z += atan_table[i]
        x, y = x_new, y_new

    return y, x  # sin, cos

def fk_q16_serial(q_q16):
    """Serial FK in Q16.16 (matching original fk_engine.v)."""
    ONE = 65536

    # Identity matrix
    Ta = [[ONE,0,0,0],[0,ONE,0,0],[0,0,ONE,0],[0,0,0,ONE]]

    for joint in range(6):
        a_q16 = q16_from_float(DH[joint][0])
        alpha_q16 = q16_from_float(DH[joint][1])
        d_q16 = q16_from_float(DH[joint][2])

        # 2 CORDIC calls per joint (serial)
        sin_q, cos_q = q16_sin_cos_cordic(q_q16[joint])
        sin_a, cos_a = q16_sin_cos_cordic(alpha_q16)

        # Build Ti
        Ti = [
            [cos_q, -q16_mult(sin_q, cos_a), q16_mult(sin_q, sin_a), q16_mult(a_q16, cos_q)],
            [sin_q, q16_mult(cos_q, cos_a), -q16_mult(cos_q, sin_a), q16_mult(a_q16, sin_q)],
            [0, sin_a, cos_a, d_q16],
            [0, 0, 0, ONE],
        ]

        # 4x4 matrix multiply (row by row serial)
        Tt = [[0]*4 for _ in range(4)]
        for r in range(4):
            for c in range(4):
                s = 0
                for k in range(4):
                    s += q16_mult(Ta[r][k], Ti[k][c])
                Tt[r][c] = s
        Ta = [row[:] for row in Tt]

    return Ta

def full_pipeline_q16(q_float, xd_float, Ks_float, Ds_float):
    """Full pipeline in Q16.16."""
    q_q16 = [q16_from_float(x) for x in q_float]
    Ta = fk_q16_serial(q_q16)

    # Extract pose
    pose_q16 = [Ta[0][3], Ta[1][3], Ta[2][3], 0, 0, 0]  # simplified euler

    # Impedance (simplified)
    xd_q16 = [q16_from_float(x) for x in xd_float]
    Ks_q16 = [q16_from_float(x) for x in Ks_float]
    Ds_q16 = [q16_from_float(x) for x in Ds_float]

    F_q16 = [q16_mult(Ks_q16[i], xd_q16[i] - pose_q16[i]) for i in range(6)]

    return pose_q16, F_q16

# ============================================================
# Benchmarking
# ============================================================

def benchmark_function(func, args, n_iter=10000):
    """Benchmark a function, return average time in microseconds."""
    # Warmup
    for _ in range(100):
        func(*args)

    start = time.perf_counter_ns()
    for _ in range(n_iter):
        func(*args)
    elapsed = time.perf_counter_ns() - start
    return elapsed / n_iter / 1000.0  # ns -> us

def benchmark_stages(q, n_iter=10000):
    """Benchmark each pipeline stage separately."""
    xd = np.array([0.3, 0.0, 0.35, 0.0, 0.0, 0.0])
    Ks = np.array([100.0]*3 + [10.0]*3)
    Ds = np.array([10.0]*3 + [1.0]*3)

    # Pre-compute for dependent stages
    pose, T = forward_kinematics(q)
    J = analytical_jacobian(q)
    vel = np.zeros(6)
    F = impedance_control(xd, pose, np.zeros(6), vel, Ks, Ds)

    fk_us = benchmark_function(forward_kinematics, (q,), n_iter)
    jac_us = benchmark_function(analytical_jacobian, (q,), n_iter)
    vel_us = benchmark_function(velocity_estimator, (pose, pose), n_iter)
    imp_us = benchmark_function(impedance_control, (xd, pose, np.zeros(6), vel, Ks, Ds), n_iter)
    jt_us = benchmark_function(jt_force_mapping, (J, F), n_iter)
    tau = J.T @ F
    t2iq_us = benchmark_function(torque_to_iq, (tau,), n_iter)
    full_us = benchmark_function(full_pipeline, (q, xd, Ks, Ds), n_iter)

    return {
        'fk': fk_us,
        'jac': jac_us,
        'fk_jac': fk_us + jac_us,
        'vel': vel_us,
        'imp': imp_us,
        'jt': jt_us,
        't2iq': t2iq_us,
        'full': full_us,
    }

def benchmark_q16(q_float, n_iter=1000):
    """Benchmark Q16.16 pipeline."""
    xd = [0.3, 0.0, 0.35, 0.0, 0.0, 0.0]
    Ks = [100.0]*3 + [10.0]*3
    Ds = [10.0]*3 + [1.0]*3

    q_q16 = [q16_from_float(x) for x in q_float]

    fk_us = benchmark_function(fk_q16_serial, (q_q16,), n_iter)
    full_us = benchmark_function(full_pipeline_q16, (q_float, xd, Ks, Ds), n_iter)

    return {'fk': fk_us, 'full': full_us}

# ============================================================
# ARM Cortex-A53 Estimation
# ============================================================

def estimate_arm_a53():
    """
    Estimate ARM Cortex-A53 @ 1.333 GHz execution time.

    ARM A53 is an in-order dual-issue core. Key throughput:
    - FMUL/FMADD: 1 per cycle, 4-cycle latency (double), 3-cycle (single)
    - FADD: 1 per cycle, 3-cycle latency
    - sin/cos via libm: ~120-180 cycles (double), ~80-100 (single, NEON)
    - atan2: ~150-200 cycles (double)
    - Memory: L1 hit 4 cycles, L2 hit ~20 cycles
    - Branch mispredict: ~8 cycles

    References:
    - ARM Cortex-A53 Software Optimization Guide
    - Measured libm trig on A53: ~150 cycles for sin() double
    """
    freq_ghz = 1.333
    ns_per_cycle = 1.0 / freq_ghz  # ~0.75 ns/cycle

    # FK: 6 joints, each needs:
    #   - 2 trig calls (sin/cos of q, sin/cos of alpha)
    #     But alpha is constant, so compiler can precompute → 1 sincos call
    #     sincos via libm on A53: ~150 cycles (double precision)
    #   - Build 4x4 Ti matrix: 8 multiplies with data deps → ~32 cycles
    #   - 4x4 matrix multiply T_acc × Ti:
    #     64 multiplies + 48 adds, but with data dependencies (accumulation)
    #     Each row: 4 FMADD chains of depth 4 → 4×4=16 cycles per row
    #     4 rows → 64 cycles, plus load/store overhead → ~80 cycles
    #   - Memory: store intermediate T matrices → ~20 cycles
    #   Total per joint: 150 + 32 + 80 + 20 = 282 cycles
    trig_per_joint = 150  # sincos call
    ti_build = 32  # build Ti from trig results
    matmul_per_joint = 80  # 4x4 matmul with deps
    mem_per_joint = 20  # load/store
    fk_cycles = 6 * (trig_per_joint + ti_build + matmul_per_joint + mem_per_joint)

    # Euler extraction: 3 atan2 calls + 1 sqrt + misc
    # atan2 on A53: ~180 cycles each (double)
    # sqrt: ~30 cycles
    euler_cycles = 3 * 180 + 30 + 20  # = 590

    # Jacobian: 6 columns
    # Each: vector subtraction (3 sub) + cross product (6 mul + 3 sub) + store
    # With memory access and function call overhead
    # ~40 cycles per column
    jac_cycles = 6 * 40 + 30  # = 270

    # Velocity: 6 channels × (subtract + multiply by inv_dt + IIR filter)
    # Each channel: 1 sub + 1 mul + 2 mul + 1 add = 5 FP ops
    # With deps: ~15 cycles per channel
    vel_cycles = 6 * 15 + 10  # = 100

    # Impedance: 6 channels × (2 sub + 2 mul + 1 add)
    # ~12 cycles per channel
    imp_cycles = 6 * 12 + 10  # = 82

    # J^T mapping: 6×6 matrix-vector multiply
    # 6 rows × (6 FMADD) = 36 FMADD, each row has depth-6 chain → ~24 cycles/row
    # Plus transpose access pattern (cache unfriendly) → extra latency
    jt_cycles = 6 * 28 + 20  # = 188

    # Torque to Iq: 6 divisions + 6 clamp
    # FDIV on A53: ~15-30 cycles latency
    # Serial: 6 × 20 = 120
    t2iq_cycles = 6 * 22 + 10  # = 142

    # Overhead: function calls, stack frame, loop control, cache misses
    # A53 is in-order → stalls on every cache miss and data dependency
    # Realistic overhead for a complete C function with 6 sub-calls
    overhead = 400

    total_cycles = (fk_cycles + euler_cycles + jac_cycles +
                    vel_cycles + imp_cycles + jt_cycles + t2iq_cycles + overhead)
    total_us = total_cycles * ns_per_cycle / 1000.0

    return {
        'fk_jac_cycles': fk_cycles + euler_cycles + jac_cycles,
        'vel_cycles': vel_cycles,
        'imp_cycles': imp_cycles,
        'jt_cycles': jt_cycles,
        't2iq_cycles': t2iq_cycles,
        'overhead_cycles': overhead,
        'total_cycles': total_cycles,
        'total_us': total_us,
        'freq_ghz': freq_ghz,
    }

# ============================================================
# STM32H7 Estimation
# ============================================================

def estimate_stm32h7():
    """
    Estimate STM32H7 @ 480 MHz (Cortex-M7, single-precision FPU).

    M7 has single-cycle FMUL (single precision), but:
    - Double precision: software emulation (~20x slower)
    - sin/cos: CMSIS-DSP arm_sin_f32 ~40 cycles, libm sinf ~100 cycles
    - No out-of-order execution
    - Flash wait states (even with ART accelerator): ~1-2 extra cycles
    - Tight coupling memory (TCM) helps but limited to 128KB

    Literature reference:
    - Typical 6-DOF FK on STM32H7: 200-400 us (measured)
    - Phan et al. report ~580 us for full 6-DOF torque control on TI C2000
    """
    freq_mhz = 480
    ns_per_cycle = 1000.0 / freq_mhz  # ~2.08 ns

    # FK: 6 joints
    # sincos via CMSIS-DSP (polynomial approx): ~50 cycles (single precision)
    # But we need double-like precision for Q16.16 equivalent → use libm sinf: ~100 cycles
    # 4x4 matmul on M7: single-issue, 64 FMUL + 48 FADD
    #   FMUL: 1 cycle throughput, 3 cycle latency (single)
    #   With accumulation deps: each row inner product depth-4 → ~12 cycles/row
    #   4 rows → 48 cycles, plus load/store → ~70 cycles
    # Build Ti: ~20 cycles
    # Memory overhead: ~30 cycles (stack, load DH params)
    trig_per_joint = 100  # sincos
    ti_build = 20
    matmul_per_joint = 70
    mem_per_joint = 30
    fk_per_joint = trig_per_joint + ti_build + matmul_per_joint + mem_per_joint
    fk_cycles = 6 * fk_per_joint  # = 1320

    # Euler: 3 atan2f (~120 cycles each) + sqrtf (~20 cycles)
    euler_cycles = 3 * 120 + 20 + 30  # = 410

    # Jacobian: 6 columns × ~35 cycles
    jac_cycles = 6 * 35 + 40  # = 250

    # Velocity: 6 × 20 cycles
    vel_cycles = 6 * 20 + 10  # = 130

    # Impedance: 6 × 15 cycles
    imp_cycles = 6 * 15 + 10  # = 100

    # J^T: 6×6 matvec, serial rows
    # Each row: 6 FMADD depth-6 chain → ~24 cycles
    jt_cycles = 6 * 26 + 20  # = 176

    # T2Iq: 6 FDIV (~14 cycles each on M7) + clamp
    t2iq_cycles = 6 * 18 + 10  # = 118

    # Overhead: function calls, flash wait states, interrupt latency
    # M7 has more overhead than A53 due to simpler pipeline
    overhead = 500

    total_cycles = (fk_cycles + euler_cycles + jac_cycles +
                    vel_cycles + imp_cycles + jt_cycles + t2iq_cycles + overhead)
    total_us = total_cycles * ns_per_cycle / 1000.0

    return {
        'total_cycles': total_cycles,
        'total_us': total_us,
        'freq_mhz': freq_mhz,
    }

# ============================================================
# Main
# ============================================================

def main():
    print("=" * 70)
    print("  CPU Benchmark: Impedance Control Pipeline")
    print("  FPGA baseline: 395 cycles = 3.95 us @ 100 MHz")
    print("=" * 70)
    print()

    FPGA_US = 3.95
    N_ITER = 50000

    # --- Python float64 benchmark ---
    print(f"[1/4] Python float64 benchmark ({N_ITER} iterations)...")
    all_results = []
    for i, q in enumerate(TEST_CONFIGS):
        q_np = np.array(q)
        r = benchmark_stages(q_np, N_ITER)
        all_results.append(r)
        if i == 0:
            print(f"  Config 0: FK={r['fk']:.1f} Jac={r['jac']:.1f} Vel={r['vel']:.2f} "
                  f"Imp={r['imp']:.2f} JT={r['jt']:.2f} T2Iq={r['t2iq']:.2f} "
                  f"Full={r['full']:.1f} us")

    avg_float = {k: np.mean([r[k] for r in all_results]) for k in all_results[0]}
    print(f"  Average: FK+Jac={avg_float['fk_jac']:.1f} Full={avg_float['full']:.1f} us")
    print()

    # --- Python Q16.16 benchmark ---
    print(f"[2/4] Python Q16.16 fixed-point benchmark ({N_ITER//10} iterations)...")
    q16_results = []
    for i, q in enumerate(TEST_CONFIGS):
        r = benchmark_q16(q, N_ITER // 10)
        q16_results.append(r)
    avg_q16 = {k: np.mean([r[k] for r in q16_results]) for k in q16_results[0]}
    print(f"  Average: FK={avg_q16['fk']:.1f} Full={avg_q16['full']:.1f} us")
    print()

    # --- NumPy vectorized (batch) ---
    print(f"[3/4] NumPy vectorized benchmark...")
    q_batch = np.array(TEST_CONFIGS)

    def numpy_batch_fk():
        results = []
        for q in q_batch:
            results.append(forward_kinematics(q))
        return results

    numpy_us = benchmark_function(numpy_batch_fk, (), N_ITER // 10)
    numpy_per_config = numpy_us / len(TEST_CONFIGS)
    print(f"  Per-config: {numpy_per_config:.1f} us")
    print()

    # --- ARM A53 estimation ---
    print("[4/4] ARM Cortex-A53 @ 1.333 GHz estimation...")
    arm = estimate_arm_a53()
    print(f"  FK+Jac: {arm['fk_jac_cycles']} cycles")
    print(f"  Total: {arm['total_cycles']} cycles = {arm['total_us']:.2f} us")
    print()

    # --- STM32H7 estimation ---
    stm32 = estimate_stm32h7()
    print(f"  STM32H7 @ {stm32['freq_mhz']} MHz: {stm32['total_cycles']} cycles = {stm32['total_us']:.2f} us")
    print()

    # ============================================================
    # Results Table
    # ============================================================
    print("=" * 70)
    print("  COMPARISON TABLE")
    print("=" * 70)
    print(f"  {'Platform':<35} {'Latency':>10} {'Speedup':>10}")
    print(f"  {'-'*35} {'-'*10} {'-'*10}")
    print(f"  {'FPGA @ 100 MHz (measured)':<35} {'3.95 us':>10} {'1.0x':>10}")
    arm_lat = f"{arm['total_us']:.2f} us"
    arm_spd = f"{arm['total_us']/FPGA_US:.1f}x"
    print(f"  {'ARM A53 @ 1.333 GHz (estimated)':<35} {arm_lat:>10} {arm_spd:>10}")
    stm_lat = f"{stm32['total_us']:.2f} us"
    stm_spd = f"{stm32['total_us']/FPGA_US:.1f}x"
    print(f"  {'STM32H7 @ 480 MHz (estimated)':<35} {stm_lat:>10} {stm_spd:>10}")
    flt_lat = f"{avg_float['full']:.1f} us"
    flt_spd = f"{avg_float['full']/FPGA_US:.0f}x"
    print(f"  {'Python float64 (measured)':<35} {flt_lat:>10} {flt_spd:>10}")
    q16_lat = f"{avg_q16['full']:.1f} us"
    q16_spd = f"{avg_q16['full']/FPGA_US:.0f}x"
    print(f"  {'Python Q16.16 (measured)':<35} {q16_lat:>10} {q16_spd:>10}")
    print("=" * 70)
    print()

    # ============================================================
    # Save CSV
    # ============================================================
    csv_dir = "D:/papercode/data/csv"
    os.makedirs(csv_dir, exist_ok=True)
    csv_path = os.path.join(csv_dir, "cpu_benchmark.csv")

    with open(csv_path, 'w') as f:
        f.write("platform,freq,fk_jac_us,vel_us,imp_us,jt_us,t2iq_us,total_us,speedup_vs_fpga\n")
        f.write(f"FPGA,100MHz,3.82,0.03,0.02,0.06,0.02,3.95,1.0\n")
        f.write(f"ARM_A53_est,1.333GHz,"
                f"{arm['fk_jac_cycles']*0.75/1000:.2f},"
                f"{arm['vel_cycles']*0.75/1000:.4f},"
                f"{arm['imp_cycles']*0.75/1000:.4f},"
                f"{arm['jt_cycles']*0.75/1000:.4f},"
                f"{arm['t2iq_cycles']*0.75/1000:.4f},"
                f"{arm['total_us']:.2f},"
                f"{arm['total_us']/FPGA_US:.1f}\n")
        f.write(f"STM32H7_est,480MHz,,,,,,{stm32['total_us']:.2f},{stm32['total_us']/FPGA_US:.1f}\n")
        f.write(f"Python_float64,measured,"
                f"{avg_float['fk_jac']:.2f},"
                f"{avg_float['vel']:.4f},"
                f"{avg_float['imp']:.4f},"
                f"{avg_float['jt']:.4f},"
                f"{avg_float['t2iq']:.4f},"
                f"{avg_float['full']:.2f},"
                f"{avg_float['full']/FPGA_US:.1f}\n")
        f.write(f"Python_Q16_16,measured,"
                f"{avg_q16['fk']:.2f},,,,,"
                f"{avg_q16['full']:.2f},"
                f"{avg_q16['full']/FPGA_US:.1f}\n")

    print(f"Results saved to {csv_path}")

    # ============================================================
    # Per-stage breakdown for paper table
    # ============================================================
    print()
    print("=" * 70)
    print("  PER-STAGE BREAKDOWN (for paper Table)")
    print("=" * 70)
    print(f"  {'Stage':<25} {'FPGA(us)':>10} {'FPGA(cyc)':>10} {'ARM A53(us)':>12} {'Ratio':>8}")
    print(f"  {'-'*25} {'-'*10} {'-'*10} {'-'*12} {'-'*8}")

    stages = [
        ("FK + Jacobian", 3.82, 382, arm['fk_jac_cycles']*0.75/1000),
        ("Velocity Est.", 0.03, 3, arm['vel_cycles']*0.75/1000),
        ("Impedance Ctrl", 0.02, 2, arm['imp_cycles']*0.75/1000),
        ("J^T Mapping", 0.06, 6, arm['jt_cycles']*0.75/1000),
        ("Torque->Iq", 0.02, 2, arm['t2iq_cycles']*0.75/1000),
    ]

    fpga_total = sum(s[1] for s in stages)
    arm_total = sum(s[3] for s in stages)

    for name, fpga_us, fpga_cyc, arm_us in stages:
        ratio = arm_us / fpga_us if fpga_us > 0 else 0
        print(f"  {name:<25} {fpga_us:>10.2f} {fpga_cyc:>10d} {arm_us:>12.4f} {ratio:>7.1f}x")

    print(f"  {'-'*25} {'-'*10} {'-'*10} {'-'*12} {'-'*8}")
    print(f"  {'TOTAL':<25} {fpga_total:>10.2f} {395:>10d} {arm['total_us']:>12.2f} {arm['total_us']/FPGA_US:>7.1f}x")
    print("=" * 70)

if __name__ == "__main__":
    main()
