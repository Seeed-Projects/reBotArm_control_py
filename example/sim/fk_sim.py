#!/usr/bin/env python3
"""正运动学仿真 — 交互式关节角度控制 + MeshCat 实时可视化。

用法:
    python example/sim/fk_sim.py

控制:
    输入 6 个关节角度（度），空格分隔
    例: 0 0 0 0 0 0
    例: 45 -30 15 -60 90 180
    q / quit / exit: 退出
"""

import sys
import signal
import time
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from reBotArm_control_py.kinematics import compute_fk
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
    n_arm = 6
    q = np.zeros(viz.nq)
    viz.update(q)

    print("MeshCat 已打开. 输入 6 个 arm 关节角度（度）:")
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
            if len(q_deg) != n_arm:
                print(f"需要 {n_arm} 个值（夹爪在仿真中保持 0）\n")
                continue
        except ValueError:
            print("无效输入\n")
            continue

        q = np.zeros(viz.nq)
        q[:n_arm] = np.radians(q_deg)
        viz.update(q)

        pos, rot, _ = compute_fk(viz.model, q)
        euler = np.degrees(pin.rpy.matrixToRpy(rot))
        print(f"  末端位置: [{pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f}] m")
        print(f"  末端姿态: [{euler[0]:+.2f}, {euler[1]:+.2f}, {euler[2]:+.2f}] deg\n")


if __name__ == "__main__":
    main()
