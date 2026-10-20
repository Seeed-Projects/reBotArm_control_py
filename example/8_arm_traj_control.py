#!/usr/bin/env python3
"""
RebotArmEndPose 交互控制示例（轨迹规划模式）。
输入: x y z [roll pitch yaw] [duration]  目标末端位置（米 / 弧度 / 秒）
      g <pos>                            设置夹爪目标位置

RebotArmEndPose interactive control example (trajectory planning mode).
Input: x y z [roll pitch yaw] [duration]  target end-effector pose (meters / radians / seconds)
       g <pos>                            set gripper target position

用法 / Usage:
    python example/8_arm_traj_control.py

退出 / Exit: q / quit / exit /ctrl+c
状态 / State: state, end_state
"""

import sys
import numpy as np
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reBotArm_control_py.actuator import RebotArm
from reBotArm_control_py.controllers import RebotArmEndPose
from reBotArm_control_py.kinematics import joint_to_pose


def main() -> None:
    rebotarm = RebotArm()
    ctrl = RebotArmEndPose(rebotarm)

    try:
        ctrl.start()
        interactive(ctrl, rebotarm)
    finally:
        ctrl.end()
    print("\n完成 / Done.")


def interactive(ctrl, rebotarm):
    print("--- 已启动末端位置控制器 ---\n")
    print("--- End-effector pose controller started ---\n")

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
            q, _, _ = rebotarm.get_state()
            print(f"  机械臂 / Arm (rad): {[f'{v:+.3f}' for v in q[:rebotarm.arm.num_joints]]}")
            if rebotarm.has_gripper:
                print(f"  夹爪 / Gripper (rad): {q[rebotarm.arm.num_joints]:+.3f}")
            continue

        if line.lower() == "end_state":
            q, _, _ = rebotarm.get_state()
            pos, rpy = joint_to_pose(q[:rebotarm.arm.num_joints])
            px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
            rx, ry, rz = float(rpy[0]), float(rpy[1]), float(rpy[2])
            print(f"  pos=[{px:+.3f} {py:+.3f} {pz:+.3f}] m  rpy=[{rx:+.2f} {ry:+.2f} {rz:+.2f}] rad")
            continue

        parts = line.split()
        cmd = parts[0].lower()

        if cmd == "g" and len(parts) >= 2:
            try:
                pos = float(parts[1])
                ctrl.set_gripper_target(pos)
                print(f"  夹爪 / Gripper -> {pos:.3f} rad")
            except ValueError:
                print("  用法 / Usage: g <pos>")
            continue

        try:
            vals = [float(v) for v in parts]
        except ValueError:
            print("  格式 / Format: x y z [roll pitch yaw] [duration]")
            continue

        if len(vals) not in (3, 6, 7) or not np.all(np.isfinite(vals)):
            print("  需要有效的位置/姿态输入 / Invalid pose input")
            continue
        x, y, z = vals[0], vals[1], vals[2]
        if len(vals) >= 6:
            roll, pitch, yaw = vals[3:6]
        else:
            q, _, _ = rebotarm.get_state()
            _, rpy = joint_to_pose(q[:rebotarm.arm.num_joints])
            roll, pitch, yaw = map(float, rpy)
        duration = vals[6] if len(vals) >= 7 else 2.0

        ok = ctrl.move_to_traj(
            x=x, y=y, z=z,
            roll=roll, pitch=pitch, yaw=yaw,
            duration=duration,
        )
        print(f"  -> ({x:+.3f}, {y:+.3f}, {z:+.3f})  "
              f"T={duration:.1f}{'ok' if ok else 'failed'}")



if __name__ == "__main__":
    main()
