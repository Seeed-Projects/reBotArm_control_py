#!/usr/bin/env python3
"""
reBotArm POS_VEL 控制（全部关节，测试模式）。
输入: arm 关节角度（度）；有夹爪时可额外输入夹爪角度
所有关节统一 POS_VEL 模式，每周期同步发送。

POS_VEL control for all joints (test mode).
Input: Arm joint angles (degrees), optionally followed by gripper angles
All joints use POS_VEL mode, synchronized sending every cycle.

用法 / Usage:
    python example/4_pos_vel_control.py

示例 / Examples:
    30 0 0 0 0 0        # 仅更新 arm；夹爪保持当前目标 / arm only; gripper target unchanged
    30 0 0 0 0 0 2.0    # arm + 夹爪（如果配置了 gripper）/ arm + gripper (if configured)
"""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reBotArm_control_py.actuator import RebotArm

rebotarm = RebotArm()
rebotarm.connect()
rebotarm.arm.mode_pos_vel()
if rebotarm.has_gripper:
    rebotarm.gripper.mode_pos_vel()
rebotarm.enable_all()

n_arm = rebotarm.arm.num_joints
n_gripper = rebotarm.gripper.num_joints
n_total = n_arm + n_gripper
target_pos = rebotarm.get_positions().copy()


def pos_vel_controller(r: RebotArm, dt: float) -> None:
    r.arm.send_pos_vel(target_pos[:r.arm.num_joints])
    if r.has_gripper:
        r.gripper.send_pos_vel(target_pos[r.arm.num_joints:])


rebotarm.start_control_loop(pos_vel_controller)

print(f"关节数 / Joint count: {n_total} (arm={n_arm}, gripper={n_gripper}) | {rebotarm.rate}Hz")
accepted_lengths = (n_arm, n_total) if n_gripper else (n_total,)
accepted_hint = " 或 ".join(str(n) for n in accepted_lengths)
print(f"命令 / Command: {accepted_hint}个角度(度)  q退出/exit  state查看状态/state\n")

try:
    while True:
        try:
            line = input("> ").split("#", 1)[0].strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not line:
            continue
        if line.lower() in ("q", "quit", "exit"):
            break

        if line.lower() == "state":
            pos = rebotarm.get_positions()
            print(f"  pos (deg): {[f'{x:+.2f}' for x in np.degrees(pos)]}")
            continue

        tokens = line.split()
        if len(tokens) not in accepted_lengths:
            print(f"需要 {accepted_hint} 个值（{n_arm} 关节 + {n_gripper} 夹爪）")
            print(f"Need {accepted_hint} values ({n_arm} joints + {n_gripper} gripper)")
            continue

        try:
            pos_deg = [float(x) for x in tokens]
        except ValueError:
            print("输入必须是数字 / Values must be numbers")
            continue
        if not np.all(np.isfinite(pos_deg)):
            print("输入必须是有限数字 / Values must be finite numbers")
            continue

        target_pos[:len(pos_deg)] = np.radians(pos_deg)
        print(f"  -> {[f'{x:+.1f}' for x in pos_deg]}")
finally:
    rebotarm.disconnect()
