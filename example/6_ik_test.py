#!/usr/bin/env python3
"""
reBotArm 逆运动学数据测试。
输入: 末端期望位置 (x y z)，单位：米
     可选: 跟随姿态 (roll pitch yaw)，单位：度
输出: 求得的关节角度（度）
      + 收敛信息

Inverse kinematics data test.
Input: End-effector desired position (x y z) in meters
       Optional: Follow orientation (roll pitch yaw) in degrees
Output: Computed joint angles in degrees
        + Convergence info

用法 / Usage:
    python example/6_ik_test.py

配置 / Config: config/rebotarm.yaml
"""

import sys
import numpy as np
import pinocchio as pin

sys.path.insert(0, str(__file__).rsplit("/", 2)[0])

from reBotArm_control_py.kinematics import (
    get_arm_joint_count,
    get_end_effector_frame_id,
    load_robot_model,
    get_joint_names,
)
from reBotArm_control_py.kinematics.inverse_kinematics import IKParams, pos_rot_to_se3, solve_ik


# ----------------------------------------------------------------------
# 打印 / UI 相关
# ----------------------------------------------------------------------

def print_welcome(model, joint_names) -> None:
    print("=" * 52)
    print("  reBotArm 逆运动学测试 / Inverse Kinematics Test")
    print("=" * 52)
    print(f"  机器人 / Robot: {model.name}")
    n = get_arm_joint_count(model)
    print(f"  机械臂关节 / Arm joints: {joint_names[:n]}")
    print(f"  nq={model.nq} (URDF), arm={n}; 夹指位移保持 0 米 / fingers stay at 0 m")
    print()
    print("  输入末端期望位姿 / Enter desired end-effector pose:")
    print("    <x> <y> <z>                       (仅位置，米 / position only, meters)")
    print("    <x> <y> <z> <roll> <pitch> <yaw>    (位置+姿态，度 / position+orientation, degrees)")
    print()
    print("  示例 / Examples:")
    print("    0.25 0.0 0.15                      (仅位置 / position only)")
    print("    0.25 0.0 0.15 0 0 0                (位置+姿态 / position+orientation)")
    print("-" * 52)
    print("> ", end="", flush=True)


def print_result(result, target_pos, target_rot, joint_names, n_joints: int) -> None:
    print()
    print("=" * 52)
    print("  结果 / Result")
    print("=" * 52)
    print(f"  目标末端位置 / Target position   : [{target_pos[0]:+.4f}, {target_pos[1]:+.4f}, {target_pos[2]:+.4f}] m")
    if target_rot is not None:
        euler_in = np.degrees(pin.rpy.matrixToRpy(target_rot))
        print(f"  目标末端姿态 / Target orientation: [{euler_in[0]:+.2f}, {euler_in[1]:+.2f}, {euler_in[2]:+.2f}] deg")
    print()
    print(f"  收敛 / Converged  : {'是 / Yes' if result.success else '否 / No'}")
    print(f"  迭代次数 / Iterations: {result.iterations}")
    print(f"  位置误差 / Position error: {result.error:.2e} m")
    print()
    print(f"  关节角度 (度) [前 {n_joints} 个控制关节] / Joint angles (deg) [first {n_joints} control joints]:")
    for name, deg, rad in zip(joint_names[:n_joints], np.degrees(result.q[:n_joints]), result.q[:n_joints]):
        print(f"    {name:10s} = {deg:+8.4f} deg  ({rad:+.4f} rad)")


def parse_pose_input(line: str) -> tuple:
    tokens = line.split("#", 1)[0].split()
    if len(tokens) not in (3, 6):
        print(f"错误: 需要 3 个值（仅位置）或 6 个值（位置+姿态），输入了 {len(tokens)} 个")
        print(f"Error: need 3 values (pos only) or 6 values (pos+ori), got {len(tokens)}")
        sys.exit(1)
    try:
        vals = [float(x) for x in tokens]
    except ValueError as e:
        print(f"错误: 无法解析数字 — {e}")
        print(f"Error: cannot parse number — {e}")
        sys.exit(1)

    if not np.all(np.isfinite(vals)):
        print("错误: 输入必须是有限数字 / Error: values must be finite numbers")
        sys.exit(1)
    target_pos = np.array(vals[:3])
    target_rot = None
    if len(vals) == 6:
        roll, pitch, yaw = np.radians(vals[3:6])
        target_rot = pin.rpy.rpyToMatrix(roll, pitch, yaw)
    return target_pos, target_rot


# ----------------------------------------------------------------------
# main
# ----------------------------------------------------------------------

def main() -> None:
    model = load_robot_model()
    joint_names = get_joint_names(model)
    frame_id = get_end_effector_frame_id(model)
    n_joints = get_arm_joint_count(model, frame_id)

    print_welcome(model, joint_names)

    try:
        line = input().strip()
    except EOFError:
        print("无输入，退出。/ No input, exit.")
        return

    target_pos, target_rot = parse_pose_input(line)

    q_init = pin.neutral(model)

    ik_params = IKParams(max_iter=2000, damping=0.01)

    result = solve_ik(
        model, model.createData(), frame_id,
        pos_rot_to_se3(target_pos, target_rot), q_init, ik_params,
        controlled_joints=n_joints, position_only=target_rot is None,
    )

    print_result(result, target_pos, target_rot, joint_names, n_joints)


if __name__ == "__main__":
    main()
