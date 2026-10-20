#!/usr/bin/env python3
"""
reBotArm 正运动学数据测试。
输入: 6 个机械臂关节角度，单位：度，空格分隔；URDF 夹指位移保持 0
输出: 末端位置 (x, y, z)，单位：米
      + 旋转矩阵 (3x3)
      + 欧拉角 (roll, pitch, yaw)，单位：度

Forward kinematics data test.
Input: Six arm joint angles in degrees; URDF finger displacements stay at zero
Output: End-effector position (x, y, z) in meters
        + Rotation matrix (3x3)
        + Euler angles (roll, pitch, yaw) in degrees

用法 / Usage:
    python example/5_fk_test.py

配置 / Config: config/rebotarm.yaml
"""

import sys
import numpy as np
import pinocchio as pin

sys.path.insert(0, str(__file__).rsplit("/", 2)[0])

from reBotArm_control_py.kinematics import (
    load_robot_model,
    compute_fk,
    get_joint_names,
    get_arm_joint_count,
    pad_q_for_model,
)

# ----------------------------------------------------------------------
# 打印 / Print
# ----------------------------------------------------------------------
def print_welcome(model, joint_names) -> None:
    n = get_arm_joint_count(model)
    print("=" * 52)
    print("  reBotArm 正运动学测试 / Forward Kinematics Test")
    print("=" * 52)
    print(f"  机器人 / Robot  : {model.name}")
    print(f"  关节    / Joints: {joint_names[:n]}")
    print(f"  nq = {model.nq} (URDF), 输入 {n} 个 arm 角度 / {n} arm angles as input")
    print("  URDF 夹指位移保持 0 米 / URDF finger displacements stay at 0 m")
    print()
    print(f"  输入 {n} 个关节角度（度），空格分隔 / Enter {n} joint angles (deg), space-separated")
    print("-" * 52)
    print("> ", end="", flush=True)

def print_result(q_deg, position, rotation, euler_deg) -> None:
    print()
    print("=" * 52)
    print("  结果 / Result")
    print("=" * 52)
    print(f"  关节角度 (度) / Joint angles (deg): {q_deg}")
    print()
    print(f"  末端位置 (m) / End-effector position (m):")
    print(f"    X = {position[0]:+.6f}")
    print(f"    Y = {position[1]:+.6f}")
    print(f"    Z = {position[2]:+.6f}")
    print()
    print(f"  旋转矩阵 (R_world^end) / Rotation matrix (R_world^end):")
    for row in rotation:
        print(f"    [{row[0]:+.6f}  {row[1]:+.6f}  {row[2]:+.6f}]")
    print()
    print(f"  欧拉角 XYZ (横滚, 俯仰, 偏航) [度] / Euler XYZ (roll, pitch, yaw) [deg]:")
    print(f"    横滚/roll  = {euler_deg[0]:+.4f}")
    print(f"    俯仰/pitch = {euler_deg[1]:+.4f}")
    print(f"    偏航/yaw   = {euler_deg[2]:+.4f}")

def parse_joint_input(line: str, n: int) -> np.ndarray:
    tokens = line.split("#", 1)[0].split()
    if len(tokens) != n:
        print(f"错误: 需要 {n} 个值，输入了 {len(tokens)} 个")
        print(f"Error: need {n} values, got {len(tokens)}")
        sys.exit(1)
    try:
        q_deg = [float(x) for x in tokens]
    except ValueError as e:
        print(f"错误: 无法解析数字 — {e}")
        print(f"Error: cannot parse number — {e}")
        sys.exit(1)
    if not np.all(np.isfinite(q_deg)):
        print("错误: 输入必须是有限数字 / Error: values must be finite numbers")
        sys.exit(1)
    return np.radians(q_deg)


# ----------------------------------------------------------------------
# 核心算法 / Core algorithm
# ----------------------------------------------------------------------
def compute_fk_from_deg(model, q_deg: list) -> tuple:
    q_rad = np.radians(q_deg)
    full_q = pad_q_for_model(model, q_rad, controlled_joints=get_arm_joint_count(model))
    position, rotation, homogeneous = compute_fk(model, full_q)
    euler_deg = np.degrees(pin.rpy.matrixToRpy(rotation))
    return position, rotation, homogeneous, euler_deg

# ----------------------------------------------------------------------
# main
# ----------------------------------------------------------------------
def main() -> None:
    model = load_robot_model()
    joint_names = get_joint_names(model)
    n_joints = get_arm_joint_count(model)

    print_welcome(model, joint_names)

    try:
        line = input().strip()
    except EOFError:
        print("无输入，退出。/ No input, exit.")
        return

    q_rad = parse_joint_input(line, n_joints)
    q_deg = np.degrees(q_rad)

    position, rotation, homogeneous, euler_deg = compute_fk_from_deg(model, q_deg)

    print_result(q_deg, position, rotation, euler_deg)

if __name__ == "__main__":
    main()
