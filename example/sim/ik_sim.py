#!/usr/bin/env python3
"""逆运动学仿真 — 交互式输入目标位姿 + MeshCat 实时可视化。

用法:
    uv run python example/sim/ik_sim.py

控制:
    输入目标位置 x y z (米)
    可选: 姿态 roll pitch yaw (弧度)
    例: 0.25 0.0 0.25                    (仅位置)
    例: 0.29545 0.0 0.28664 0 0.17453 0  (位置+姿态)
    q / quit / exit: 退出
"""

import sys
import signal
import time
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from reBotArm_control_py.kinematics.inverse_kinematics import compute_ik
from reBotArm_control_py.kinematics.robot_model import get_end_effector_frame_id
from reBotArm_control_py.kinematics.forward_kinematics import compute_fk
from example.sim.visualizer import Visualizer

should_exit = False


def signal_handler(sig, frame):
    global should_exit
    should_exit = True


def main():
    signal.signal(signal.SIGINT, signal_handler)

    print("加载可视化器...")
    viz = Visualizer()

    viz.neutral()
    q_seed = pin.neutral(viz.model)

    # gripper_end 挂在 joint6 后；只把该关节及其祖先作为机械臂关节显示，
    # 排除 URDF 中位于末端之后的两个夹爪滑动关节。
    end_frame = viz.model.frames[get_end_effector_frame_id(viz.model)]
    end_joint = viz.model.joints[end_frame.parentJoint]
    arm_nq = end_joint.idx_q + end_joint.nq

    print("MeshCat 已打开. 输入目标位姿:")
    print("  x y z                      (仅位置，米)")
    print("  x y z roll pitch yaw       (位置+姿态，弧度)")
    print("  可达位姿示例: 0.29545 0 0.28664 0 0.17453 0")
    print("  q/quit/exit: 退出\n")

    while not should_exit:
        time.sleep(0.01)

        try:
            line = input("目标位姿 > ").strip().lower()
        except EOFError:
            break

        if line in ("q", "quit", "exit", ""):
            break

        try:
            vals = [float(x) for x in line.split()]
            if len(vals) not in (3, 6):
                print("需要 3 个值（仅位置）或 6 个值（位置+姿态）\n")
                continue
        except ValueError:
            print("无效输入\n")
            continue

        target_pos = np.array(vals[:3])  # 获取位置
        target_rot = None
        if len(vals) == 6:
            r, p, y = vals[3], vals[4], vals[5]
            target_rot = pin.rpy.rpyToMatrix(r, p, y)  # 获取姿态

        result = compute_ik(q_seed, target_pos, target_rot)

        if result.success:
            viz.update(result.q)
            # 后续目标从当前已到达构型继续求解，避免每次跳回零位。
            q_seed = result.q.copy()
        status = "收敛" if result.success else "未收敛"
        actual_pos, actual_rot, _ = compute_fk(viz.model, result.q)
        position_error = float(np.linalg.norm(target_pos - actual_pos))

        if target_rot is None:
            print(
                f"  [{status}] 迭代={result.iterations} "
                f"位置误差={position_error:.2e}m"
            )
        else:
            orientation_error = float(np.linalg.norm(pin.log3(actual_rot.T @ target_rot)))
            print(
                f"  [{status}] 迭代={result.iterations} "
                f"位置误差={position_error:.2e}m "
                f"姿态误差={orientation_error:.2e}rad"
            )
        print(f"  机械臂关节角度(deg): {np.degrees(result.q[:arm_nq])}\n")

        if not result.success and target_rot is not None:
            position_result = compute_ik(q_seed, target_pos)
            if position_result.success:
                _, reachable_rot, _ = compute_fk(viz.model, position_result.q)
                reachable_rpy = pin.rpy.matrixToRpy(reachable_rot)
                print("  该位置可达，但指定姿态在当前关节限位下不可达。")
                print("  若只要求位置，请输入前 3 个值。")
                print(
                    "  当前位置附近的可达 RPY(rad): "
                    f"{np.array2string(reachable_rpy, precision=5)}\n"
                )


if __name__ == "__main__":
    main()
