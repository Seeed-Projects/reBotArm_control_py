#!/usr/bin/env python3
"""
reBotArm MIT 控制（全部关节，测试模式）。
输入: 6 个 arm 关节角度（度）；有夹爪时可额外输入夹爪角度
所有关节统一 MIT 模式，每周期同步发送。

MIT control for all joints (test mode).
Input: All joint angles (degrees), space-separated
All joints use MIT mode, synchronized sending every cycle.

用法 / Usage:
    python example/3_mit_control.py [rebotarm_dm.yaml | rebotarm_rs.yaml]

示例 / Examples:
    30 0 0 0 0 0        # 仅更新 arm；夹爪保持当前目标 / arm only; gripper target unchanged
    30 0 0 0 0 0 2.0    # arm + 夹爪 / arm + gripper

当前 RS/DM 默认配置都包含 1 个夹爪电机，因此 6 个值只更新 arm，7 个值会同时更新夹爪。

Both default RS/DM configurations include one gripper motor. Six values update
the arm only; seven values also update the gripper.
"""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reBotArm_control_py.actuator import RebotArm

_hw_yaml = sys.argv[1] if len(sys.argv) > 1 else None
rebotarm = RebotArm(_hw_yaml)
rebotarm.connect()
rebotarm.arm.mode_mit()
if rebotarm.has_gripper:
    rebotarm.gripper.mode_mit()
rebotarm.enable_all()

n_arm = rebotarm.arm.num_joints
n_gripper = rebotarm.gripper.num_joints
n_total = n_arm + n_gripper
target_pos = np.zeros(n_total)
target_pos[:] = rebotarm.get_positions()


def mit_controller(r: RebotArm, dt: float) -> None:
    r.arm.send_mit(target_pos[:r.arm.num_joints])
    if r.has_gripper:
        r.gripper.send_mit(target_pos[r.arm.num_joints:])


rebotarm.start_control_loop(mit_controller)

print(f"关节数 / Joint count: {n_total} (arm={n_arm}, gripper={n_gripper}) | {rebotarm.rate}Hz")
print(f"配置 / Config: {rebotarm.hardware_yaml}")
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
            pos_deg = [float(x) for x in tokens[:n_arm]]
        except ValueError:
            print("输入必须是数字 / Values must be numbers")
            continue

        gripper_deg: list[float] = []
        if len(tokens) == n_total:
            try:
                gripper_deg = [float(x) for x in tokens[n_arm:]]
            except ValueError:
                print("输入必须是数字 / Values must be numbers")
                continue

        target_pos[:n_arm] = np.radians(pos_deg)
        if len(tokens) == n_total:
            target_pos[n_arm:] = np.radians(gripper_deg)

        shown_deg = pos_deg + gripper_deg
        print(f"  -> {[f'{x:+.1f}' for x in shown_deg]}")
finally:
    rebotarm.disconnect()
