#!/usr/bin/env python3
"""
jacobian_reference.py - Analytical Jacobian Reference Generator

Generates reference Jacobian matrices using numerical differentiation
for validating the FPGA analytical Jacobian implementation.

For each test configuration:
  1. Compute FK at q
  2. Compute numerical Jacobian via finite differences:
     J[:,i] = (FK(q+dq_i) - FK(q-dq_i)) / (2*dq)
  3. Output both FK pose and 6x6 Jacobian matrix

Uses the same DH parameters as the FPGA EPDobot model.
"""

import numpy as np
import os
import struct

# DH Parameters for EPDobot (matching part1_config.v exactly)
# Format: [d, a, alpha] in meters and radians
# Verified against FPGA Q16.16 values:
#   d_rom:     [10879, 0, 0, 12583, 0, 3604]  / 65536
#   a_rom:     [3604, 13107, 3670, 0, 0, 0]    / 65536
#   alpha_rom: [π/2, 0, π/2, -π/2, π/2, 0]
DH_PARAMS = [
    [0.166,   0.055,   np.pi/2],   # Joint 0: d=166mm, a=55mm, alpha=π/2
    [0.0,     0.200,   0.0],       # Joint 1: d=0, a=200mm, alpha=0
    [0.0,     0.056,   np.pi/2],   # Joint 2: d=0, a=56mm, alpha=π/2
    [0.192,   0.0,    -np.pi/2],   # Joint 3: d=192mm, a=0, alpha=-π/2
    [0.0,     0.0,     np.pi/2],   # Joint 4: d=0, a=0, alpha=π/2
    [0.055,   0.0,     0.0],       # Joint 5: d=55mm, a=0, alpha=0
]


def dh_matrix(q, d, a, alpha):
    """Compute 4x4 DH transformation matrix."""
    cq = np.cos(q)
    sq = np.sin(q)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [cq, -sq*ca,  sq*sa, a*cq],
        [sq,  cq*ca, -cq*sa, a*sq],
        [0,   sa,     ca,    d],
        [0,   0,      0,     1]
    ])


def forward_kinematics(q):
    """Compute FK: returns 4x4 transformation matrix and intermediate frames."""
    T_acc = np.eye(4)
    frames = []  # Store T_0^i for each joint

    for i in range(6):
        d, a, alpha = DH_PARAMS[i]
        Ti = dh_matrix(q[i], d, a, alpha)
        T_acc = T_acc @ Ti
        frames.append(T_acc.copy())

    return T_acc, frames


def rotation_to_euler_zyx(R):
    """Extract ZYX Euler angles from rotation matrix (same as FPGA)."""
    # atan2(R10, R00) → rz
    rz = np.arctan2(R[1, 0], R[0, 0])
    # atan2(-R20, sqrt(R00^2 + R10^2)) → ry
    ry = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
    # atan2(R21, R22) → rx
    rx = np.arctan2(R[2, 1], R[2, 2])
    return np.array([rx, ry, rz])


def pose_from_transform(T):
    """Extract 6-DOF pose from 4x4 transform."""
    pos = T[:3, 3]
    euler = rotation_to_euler_zyx(T[:3, :3])
    return np.concatenate([pos, euler])


def analytical_jacobian(q):
    """Compute 6x6 geometric Jacobian analytically."""
    T_final, frames = forward_kinematics(q)
    pe = T_final[:3, 3]  # End-effector position

    J = np.zeros((6, 6))

    for i in range(6):
        if i == 0:
            z = np.array([0, 0, 1])  # Base z-axis
            p = np.array([0, 0, 0])  # Base origin
        else:
            T_prev = frames[i - 1]
            z = T_prev[:3, 2]  # z-axis of frame i-1
            p = T_prev[:3, 3]  # origin of frame i-1

        # Linear velocity: z × (pe - p)
        J[:3, i] = np.cross(z, pe - p)
        # Angular velocity: z
        J[3:, i] = z

    return J


def numerical_jacobian(q, dq=1e-6):
    """Compute Jacobian using central finite differences."""
    J = np.zeros((6, 6))
    pose0 = pose_from_transform(forward_kinematics(q)[0])

    for i in range(6):
        q_plus = q.copy()
        q_minus = q.copy()
        q_plus[i] += dq
        q_minus[i] -= dq

        pose_plus = pose_from_transform(forward_kinematics(q_plus)[0])
        pose_minus = pose_from_transform(forward_kinematics(q_minus)[0])

        J[:, i] = (pose_plus - pose_minus) / (2 * dq)

    return J


def float_to_q16_16(val):
    """Convert float to Q16.16 signed 32-bit integer."""
    result = int(round(val * 65536))
    # Clamp to 32-bit signed range
    result = max(-2147483648, min(2147483647, result))
    return result


def q16_16_to_hex(val):
    """Convert Q16.16 integer to 8-digit hex string (2's complement)."""
    if val < 0:
        val = val + (1 << 32)
    return f"{val:08x}"


def main():
    output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              '..', 'sim', 'test_vectors')
    os.makedirs(output_dir, exist_ok=True)

    # Generate test configurations
    np.random.seed(42)
    configs = []

    # Special configurations
    configs.append(np.zeros(6))                          # Zero
    configs.append(np.ones(6) * 0.1)                     # Small uniform
    configs.append(np.array([np.pi/4]*6))                # pi/4
    configs.append(np.array([np.pi/2, 0, np.pi/2, 0, np.pi/2, 0]))  # Alternating
    configs.append(np.array([0.5, -0.25, 1.0, -0.5, 0.25, -1.0]))   # Mixed
    configs.append(np.array([0, np.pi/2, -np.pi/2, 0, 0, 0]))       # Boundary

    # Random configurations
    for _ in range(94):
        q = np.random.uniform(-np.pi * 0.8, np.pi * 0.8, 6)
        configs.append(q)

    num_tests = len(configs)

    # Output files
    f_q = open(os.path.join(output_dir, 'jacobian_input_q.txt'), 'w')
    f_pose = open(os.path.join(output_dir, 'jacobian_output_pose.txt'), 'w')
    f_jac = open(os.path.join(output_dir, 'jacobian_output_matrix.txt'), 'w')
    f_readable = open(os.path.join(output_dir, 'jacobian_reference_readable.txt'), 'w')

    print(f"Generating {num_tests} Jacobian reference test vectors...")

    max_jac_diff = 0  # Track max difference between analytical and numerical

    for idx, q in enumerate(configs):
        # Compute FK
        T, frames = forward_kinematics(q)
        pose = pose_from_transform(T)

        # Compute analytical Jacobian
        J_analytical = analytical_jacobian(q)

        # Compute numerical Jacobian for cross-validation
        J_numerical = numerical_jacobian(q)

        # Check consistency
        jac_diff = np.max(np.abs(J_analytical[:3, :] - J_numerical[:3, :]))
        max_jac_diff = max(max_jac_diff, jac_diff)

        # Write joint angles (Q16.16 hex)
        q_hex = ' '.join([q16_16_to_hex(float_to_q16_16(qi)) for qi in q])
        f_q.write(q_hex + '\n')

        # Write pose (Q16.16 hex)
        pose_hex = ' '.join([q16_16_to_hex(float_to_q16_16(pi)) for pi in pose])
        f_pose.write(pose_hex + '\n')

        # Write Jacobian matrix (6 rows × 6 cols, Q16.16 hex)
        for row in range(6):
            row_hex = ' '.join([q16_16_to_hex(float_to_q16_16(J_analytical[row, col]))
                               for col in range(6)])
            f_jac.write(row_hex + '\n')

        # Readable output
        f_readable.write(f"=== Test {idx} ===\n")
        f_readable.write(f"q = [{', '.join([f'{qi:.6f}' for qi in q])}] rad\n")
        f_readable.write(f"pose = [{', '.join([f'{pi:.6f}' for pi in pose])}]\n")
        f_readable.write("Jacobian (analytical):\n")
        for row in range(6):
            f_readable.write("  [" + ', '.join([f'{J_analytical[row,col]:10.6f}' for col in range(6)]) + "]\n")
        f_readable.write(f"Max diff (analytical vs numerical, linear): {jac_diff:.2e}\n\n")

    # Write test count
    with open(os.path.join(output_dir, 'jacobian_test_count.txt'), 'w') as f:
        f.write(str(num_tests))

    f_q.close()
    f_pose.close()
    f_jac.close()
    f_readable.close()

    print(f"Generated {num_tests} test vectors")
    print(f"Max analytical vs numerical Jacobian difference (linear part): {max_jac_diff:.2e}")
    print(f"Output directory: {output_dir}")

    # Summary statistics
    print("\nVerification of first test case (q=0):")
    q0 = np.zeros(6)
    J0 = analytical_jacobian(q0)
    T0, _ = forward_kinematics(q0)
    print(f"  FK pose: {pose_from_transform(T0)}")
    print(f"  J[0,:] = {J0[0,:]}")
    print(f"  J[1,:] = {J0[1,:]}")
    print(f"  J[2,:] = {J0[2,:]}")


if __name__ == '__main__':
    main()
