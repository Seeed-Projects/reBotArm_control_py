#!/usr/bin/env python3
"""
RebotArmEndPose 交互控制示例（IK 模式）。
输入: x y z [roll pitch yaw] 目标末端位置（米 / 弧度）
      g <pos>               设置夹爪目标位置

RebotArmEndPose interactive control example (IK mode).
Input: x y z [roll pitch yaw]  target end-effector pose (meters / radians)
       g <pos>                set gripper target position

用法 / Usage:
    python example/7_arm_ik_control.py

退出 / Exit: q / quit / exit
状态 / State: state, end_state
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reBotArm_control_py.actuator import RebotArm
from reBotArm_control_py.controllers import RebotArmEndPose


def main() -> None:
    rebotarm = RebotArm()
    ctrl = RebotArmEndPose(rebotarm, arm_control_mode="mit")

    ctrl.start()
    print("--- 已启动末端位置控制器 ---\n")
    print("--- End-effector pose controller started ---\n")

    while True:
        try:
            line = input("> ").strip()
        except EOFError:
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
            from reBotArm_control_py.kinematics import joint_to_pose
            pos, rpy = joint_to_pose(q)
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
            print("  格式 / Format: x y z [roll pitch yaw]")
            continue

        x, y, z = vals[0], vals[1], vals[2]
        roll = vals[3] if len(vals) >= 6 else 0.0
        pitch = vals[4] if len(vals) >= 6 else 0.0
        yaw = vals[5] if len(vals) >= 6 else 0.0

        ok = ctrl.move_to_ik(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)
        print(f"  -> ({x:+.3f}, {y:+.3f}, {z:+.3f})  "
              f"rpy=[{roll:+.2f} {pitch:+.2f} {yaw:+.2f}]  "
              f"{'ok' if ok else 'failed'}")

    ctrl.end()
    print("\n完成 / Done.")


if __name__ == "__main__":
    main()
