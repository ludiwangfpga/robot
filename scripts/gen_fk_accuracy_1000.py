#!/usr/bin/env python3
"""
gen_fk_accuracy_1000.py
生成 1000 个测试用例的正运动学精度统计报告
模拟 Q16.16 定点运算的截断误差，与浮点参考值对比
"""

import numpy as np
import pandas as pd
import os

SCALE = 65536

DH_PARAMS = [
    (0.166, 0.055, np.pi / 2),
    (0.0,   0.200, 0.0),
    (0.0,   0.056, np.pi / 2),
    (0.192, 0.0,  -np.pi / 2),
    (0.0,   0.0,   np.pi / 2),
    (0.055, 0.0,   0.0),
]

JOINT_LIMITS = [
    (-np.pi,    np.pi),
    (-1.0472,   3.6652),
    (-2.3562,   2.3562),
    (-2.6180,   2.6180),
    (-1.9199,   1.9199),
    (-2.6180,   2.6180),
]

def to_q1616(v):
    return int(round(v * SCALE))

def from_q1616(v):
    return v / SCALE

def dh_transform_float(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d],
        [0,   0,      0,     1],
    ])

def dh_transform_fixed(theta_f, d, a, alpha):
    """模拟 Q16.16 定点运算：输入角度先量化，三角函数截断到 Q16.16"""
    theta_q = from_q1616(to_q1616(theta_f))
    ct = from_q1616(to_q1616(np.cos(theta_q)))
    st = from_q1616(to_q1616(np.sin(theta_q)))
    ca = from_q1616(to_q1616(np.cos(alpha)))
    sa = from_q1616(to_q1616(np.sin(alpha)))
    a_q  = from_q1616(to_q1616(a))
    d_q  = from_q1616(to_q1616(d))
    return np.array([
        [ct, -st*ca,  st*sa, a_q*ct],
        [st,  ct*ca, -ct*sa, a_q*st],
        [0,   sa,     ca,    d_q],
        [0,   0,      0,     1],
    ])

def fk_float(q):
    T = np.eye(4)
    for i in range(6):
        d, a, alpha = DH_PARAMS[i]
        T = T @ dh_transform_float(q[i], d, a, alpha)
    return T

def fk_fixed(q):
    T = np.eye(4)
    for i in range(6):
        d, a, alpha = DH_PARAMS[i]
        T = T @ dh_transform_fixed(q[i], d, a, alpha)
        # 矩阵元素截断到 Q16.16
        T = np.vectorize(lambda x: from_q1616(to_q1616(x)))(T)
    return T

def matrix_to_pose(T):
    x, y, z = T[0,3], T[1,3], T[2,3]
    if abs(T[2,0]) < 0.9999:
        ry = np.arctan2(-T[2,0], np.sqrt(T[0,0]**2 + T[1,0]**2))
        rz = np.arctan2(T[1,0], T[0,0])
        rx = np.arctan2(T[2,1], T[2,2])
    else:
        ry = np.arctan2(-T[2,0], np.sqrt(T[0,0]**2 + T[1,0]**2))
        rz = np.arctan2(-T[0,1], T[1,1])
        rx = 0.0
    return np.array([x, y, z, rx, ry, rz])

def run(n=1000, seed=42):
    np.random.seed(seed)
    rows = []

    # 蒙特卡洛均匀采样
    for i in range(n):
        q = np.array([
            np.random.uniform(JOINT_LIMITS[j][0], JOINT_LIMITS[j][1])
            for j in range(6)
        ])
        T_ref   = fk_float(q)
        T_fixed = fk_fixed(q)
        pose_ref   = matrix_to_pose(T_ref)
        pose_fixed = matrix_to_pose(T_fixed)

        lin_err = np.linalg.norm(pose_fixed[:3] - pose_ref[:3]) * 1000   # mm
        # 姿态误差：用旋转矩阵差异计算，避免欧拉角奇异点
        dR = T_fixed[:3,:3] @ T_ref[:3,:3].T
        ang_err = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
        rows.append({
            'test_idx': i,
            'lin_err_mm': lin_err,
            'ang_err_deg': ang_err,
        })

    # 特殊用例
    special = [
        np.zeros(6),
        np.array([0, np.pi/2, 0, 0, 0, 0]),
        np.array([np.pi/4, np.pi/4, -np.pi/4, 0, 0, 0]),
        np.array([0, 1.5648, -0.6141, np.pi, -0.6201, np.pi-0.0175]),
    ]
    for i, q in enumerate(special):
        T_ref   = fk_float(q)
        T_fixed = fk_fixed(q)
        pose_ref   = matrix_to_pose(T_ref)
        pose_fixed = matrix_to_pose(T_fixed)
        lin_err = np.linalg.norm(pose_fixed[:3] - pose_ref[:3]) * 1000
        dR = T_fixed[:3,:3] @ T_ref[:3,:3].T
        ang_err = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
        rows.append({'test_idx': n+i, 'lin_err_mm': lin_err, 'ang_err_deg': ang_err})

    df = pd.DataFrame(rows)
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       '..', 'data', 'csv', 'fk_accuracy_1000.csv')
    df.to_csv(out, index=False)

    print(f"N = {len(df)} 个测试用例")
    print(f"线性误差 (mm):")
    print(f"  最大值: {df.lin_err_mm.max():.4f}")
    print(f"  均值:   {df.lin_err_mm.mean():.4f}")
    print(f"  标准差: {df.lin_err_mm.std():.4f}")
    print(f"  95th:   {df.lin_err_mm.quantile(0.95):.4f}")
    print(f"  99th:   {df.lin_err_mm.quantile(0.99):.4f}")
    print(f"姿态误差 (deg):")
    print(f"  最大值: {df.ang_err_deg.max():.4f}")
    print(f"  均值:   {df.ang_err_deg.mean():.4f}")
    print(f"  标准差: {df.ang_err_deg.std():.4f}")
    print(f"已保存至 {out}")
    return df

if __name__ == '__main__':
    run()
