#!/usr/bin/env python3
"""正运动学仿真 — 交互式关节角度控制 + MeshCat 实时可视化。

用法:
    python example/sim/fk_sim.py

控制:
    输入 6 个机械臂关节角度（度），可附加第 7 个夹爪电机角度
    第 7 个值仅兼容实机输入格式，仿真忽略该值，夹指位移保持 0 米
    例: 0 0 0 0 0 0
    例: 45 -30 15 -60 90 180
    例: 30 0 0 0 0 0 12  # 第 7 个夹爪电机角度在仿真中忽略
    q / quit / exit: 退出
"""

import sys
import signal
import time
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from reBotArm_control_py.kinematics import compute_fk, get_arm_joint_count, pad_q_for_model
from example.sim.visualizer import Visualizer

should_exit = False


def signal_handler(sig, frame):
    global should_exit
    should_exit = True
    print("\n退出.")


def main():
    signal.signal(signal.SIGINT, signal_handler)

    print("加载可视化器...")
    viz = Visualizer()
    n_arm = get_arm_joint_count(viz.model)
    q = pin.neutral(viz.model)
    viz.update(q)

    print(f"MeshCat 已打开. 输入 {n_arm} 或 {n_arm + 1} 个角度（度）:")
    print(f"  前 {n_arm} 个控制 arm；可选的夹爪电机角度将忽略，夹指位移保持 0 米")
    print("  q/quit/exit: 退出\n")

    while not should_exit:
        time.sleep(0.01)

        try:
            line = input("关节角度 > ").split("#", 1)[0].strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if line in ("q", "quit", "exit"):
            break

        if not line:
            continue

        try:
            q_deg = [float(x) for x in line.split()]
            if len(q_deg) not in (n_arm, n_arm + 1):
                print(f"需要 {n_arm} 或 {n_arm + 1} 个值（可选夹爪电机角度在仿真中忽略）\n")
                continue
        except ValueError:
            print("无效输入\n")
            continue

        if not np.all(np.isfinite(q_deg)):
            print("输入必须是有限数字\n")
            continue
        if len(q_deg) == n_arm + 1:
            print(f"  已忽略夹爪电机角度 {q_deg[-1]:g}°；URDF 夹指位移保持 0 米")
        q = pad_q_for_model(viz.model, np.radians(q_deg[:n_arm]), n_arm)
        viz.update(q)

        pos, rot, _ = compute_fk(viz.model, q)
        euler = np.degrees(pin.rpy.matrixToRpy(rot))
        print(f"  末端位置: [{pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f}] m")
        print(f"  末端姿态: [{euler[0]:+.2f}, {euler[1]:+.2f}, {euler[2]:+.2f}] deg\n")


if __name__ == "__main__":
    main()
