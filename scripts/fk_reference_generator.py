#!/usr/bin/env python3
"""
fk_reference_generator.py
生成 EPDobot 6-DOF 正向运动学参考测试向量
输出 Q16.16 格式的关节角和对应的 FK 结果，供 Verilog testbench 读取
"""

import numpy as np
import os

# Q16.16 缩放因子
SCALE = 65536

# EPDobot DH 参数 (d, a, alpha) 单位: m, rad
DH_PARAMS = [
    (0.166, 0.055, np.pi / 2),
    (0.0,   0.200, 0.0),
    (0.0,   0.056, np.pi / 2),
    (0.192, 0.0,  -np.pi / 2),
    (0.0,   0.0,   np.pi / 2),
    (0.055, 0.0,   0.0),
]

# 关节角范围 (rad)
JOINT_LIMITS = [
    (-np.pi,       np.pi),
    (-1.0472,      3.6652),   # -60° to 210°
    (-2.3562,      2.3562),   # -135° to 135°
    (-2.6180,      2.6180),   # -150° to 150°
    (-1.9199,      1.9199),   # -110° to 110°
    (-2.6180,      2.6180),   # -150° to 150°
]


def dh_transform(theta, d, a, alpha):
    """计算单个关节的 4x4 DH 变换矩阵"""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1],
    ])


def forward_kinematics(q):
    """计算正向运动学，返回 4x4 齐次变换矩阵"""
    T = np.eye(4)
    for i in range(6):
        d, a, alpha = DH_PARAMS[i]
        Ti = dh_transform(q[i], d, a, alpha)
        T = T @ Ti
    return T


def matrix_to_pose(T):
    """从 4x4 矩阵提取位姿 [x, y, z, rx, ry, rz]"""
    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]

    # 欧拉角提取 (ZYX convention)
    if abs(T[2, 0]) < 0.9999:
        ry = np.arctan2(-T[2, 0], np.sqrt(T[0, 0]**2 + T[1, 0]**2))
        rz = np.arctan2(T[1, 0], T[0, 0])
        rx = np.arctan2(T[2, 1], T[2, 2])
    else:
        # 万向锁情况
        ry = np.arctan2(-T[2, 0], np.sqrt(T[0, 0]**2 + T[1, 0]**2))
        rz = np.arctan2(-T[0, 1], T[1, 1])
        rx = 0.0

    return np.array([x, y, z, rx, ry, rz])


def to_q16_16(value):
    """浮点数转 Q16.16 定点整数"""
    return int(round(value * SCALE))


def from_q16_16(value):
    """Q16.16 定点整数转浮点数"""
    return value / SCALE


def generate_test_vectors(num_vectors=1000, seed=42):
    """生成随机测试向量"""
    np.random.seed(seed)
    vectors = []

    for i in range(num_vectors):
        # 在关节限位范围内随机生成关节角
        q = np.array([
            np.random.uniform(JOINT_LIMITS[j][0] * 0.9, JOINT_LIMITS[j][1] * 0.9)
            for j in range(6)
        ])

        # 计算 FK
        T = forward_kinematics(q)
        pose = matrix_to_pose(T)

        vectors.append((q, T, pose))

    # 添加特殊测试用例
    special_cases = [
        np.zeros(6),                                        # 零位
        np.array([0, np.pi/2, 0, 0, 0, 0]),               # 关节1 90°
        np.array([np.pi/4, np.pi/4, -np.pi/4, 0, 0, 0]),  # 混合角度
        np.array([0, 1.5648, -0.6141, np.pi, -0.6201, np.pi - 0.0175]),  # testbench 用例
    ]
    for q in special_cases:
        T = forward_kinematics(q)
        pose = matrix_to_pose(T)
        vectors.append((q, T, pose))

    return vectors


def write_test_vectors(vectors, output_dir):
    """写入测试向量文件"""
    os.makedirs(output_dir, exist_ok=True)

    # 关节角输入文件 (每行6个Q16.16值，空格分隔)
    with open(os.path.join(output_dir, "fk_input_q.txt"), "w") as f:
        for q, T, pose in vectors:
            vals = [to_q16_16(q[i]) for i in range(6)]
            f.write(" ".join(str(v) for v in vals) + "\n")

    # FK 输出: 4x4 变换矩阵 (每行16个Q16.16值)
    with open(os.path.join(output_dir, "fk_output_matrix.txt"), "w") as f:
        for q, T, pose in vectors:
            vals = []
            for r in range(4):
                for c in range(4):
                    vals.append(to_q16_16(T[r, c]))
            f.write(" ".join(str(v) for v in vals) + "\n")

    # FK 输出: 位姿 (每行6个Q16.16值: x,y,z,rx,ry,rz)
    with open(os.path.join(output_dir, "fk_output_pose.txt"), "w") as f:
        for q, T, pose in vectors:
            vals = [to_q16_16(pose[i]) for i in range(6)]
            f.write(" ".join(str(v) for v in vals) + "\n")

    # 人类可读的参考文件
    with open(os.path.join(output_dir, "fk_reference_readable.txt"), "w") as f:
        f.write(f"# EPDobot FK Reference Test Vectors\n")
        f.write(f"# Total: {len(vectors)} test cases\n")
        f.write(f"# Format: Q16.16 (scale = {SCALE})\n\n")
        for idx, (q, T, pose) in enumerate(vectors):
            f.write(f"--- Test {idx} ---\n")
            f.write(f"  q (deg): {np.degrees(q).round(2)}\n")
            f.write(f"  q (rad): {q.round(6)}\n")
            f.write(f"  pose: x={pose[0]:.6f}m y={pose[1]:.6f}m z={pose[2]:.6f}m\n")
            f.write(f"        rx={np.degrees(pose[3]):.2f}° ry={np.degrees(pose[4]):.2f}° rz={np.degrees(pose[5]):.2f}°\n")
            f.write(f"  T matrix:\n")
            for r in range(4):
                f.write(f"    [{T[r,0]:10.6f} {T[r,1]:10.6f} {T[r,2]:10.6f} {T[r,3]:10.6f}]\n")
            f.write("\n")

    # 测试数量文件
    with open(os.path.join(output_dir, "fk_test_count.txt"), "w") as f:
        f.write(f"{len(vectors)}\n")

    print(f"Generated {len(vectors)} test vectors in {output_dir}")
    print(f"Files: fk_input_q.txt, fk_output_matrix.txt, fk_output_pose.txt, fk_reference_readable.txt")


def verify_consistency():
    """验证 Python FK 与已知结果一致"""
    # 使用 testbench 中的测试用例验证
    q = np.array([0, 1.5648, -0.6141, np.pi, -0.6201, np.pi - 0.0175])
    T = forward_kinematics(q)
    pose = matrix_to_pose(T)

    print("Verification with testbench case:")
    print(f"  Target: xyz(0.3, 0, 0.3) rxryrz(90, -89, 90)")
    print(f"  FK result: x={pose[0]:.4f}m y={pose[1]:.4f}m z={pose[2]:.4f}m")
    print(f"             rx={np.degrees(pose[3]):.2f}° ry={np.degrees(pose[4]):.2f}° rz={np.degrees(pose[5]):.2f}°")
    print(f"  Position error to target: "
          f"dx={abs(pose[0]-0.3)*1000:.2f}mm "
          f"dy={abs(pose[1]-0.0)*1000:.2f}mm "
          f"dz={abs(pose[2]-0.3)*1000:.2f}mm")
    print()


if __name__ == "__main__":
    output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "..", "sim", "test_vectors")

    verify_consistency()
    vectors = generate_test_vectors(num_vectors=1000)
    write_test_vectors(vectors, output_dir)
