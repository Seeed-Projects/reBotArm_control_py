#!/usr/bin/env python3
"""reBotArm 重力补偿控制演示。

使用 Pinocchio 计算当前关节构型下的广义重力向量 g(q)，
通过 MIT 模式的前馈力矩直接补偿重力。

控制律（MIT 模式）：
    rebotarm.arm 组: MIT 位置闭环 + 重力前馈
    rebotarm.gripper 组: MIT 控制（保持位置）

安全测试: 设置 ENABLED_JOINTS 只使能部分电机，其他电机保持失能状态。
默认为全部使能；修改为只包含要测试的关节名即可，例如 ["joint1", "joint2"]。

reBotArm gravity compensation control demo.

Uses Pinocchio to compute the generalized gravity vector g(q) for the
current joint configuration, and applies gravity feedforward via MIT mode.

Control law (MIT mode):
    rebotarm.arm group: MIT position closed-loop + gravity feedforward
    rebotarm.gripper group: MIT control (hold position)

Safety test: set ENABLED_JOINTS to enable only a subset of motors; all
other motors remain disabled. Default is all enabled; set to a list of
joint names to test motors individually, e.g. ["joint1", "joint2"].
"""
import signal
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reBotArm_control_py.actuator import RebotArm
from reBotArm_control_py.dynamics import (
    load_dynamics_model,
    compute_generalized_gravity,
    get_default_gravity,
)

# ── 安全测试配置 ──────────────────────────────────────────────────────────────────
# 只使能以下关节；留空 [] 则全部使能。用于逐个电机安全测试。

# ── Safety test configuration ────────────────────────────────────────────────────
# Only enable the following joints; empty [] means all enabled. Used for safe per-motor testing.

ENABLED_JOINTS: list[str] = []
# ENABLED_JOINTS: list[str] = ["joint1"]      # 单电机测试 / single-motor test
# ENABLED_JOINTS: list[str] = ["joint1", "joint2"]  # 双电机测试 / two-motor test

_running = True


def _sigint_handler(signum, frame):
    global _running
    print("\n[gravity_comp] 收到 Ctrl+C，准备停止... / Received Ctrl+C, preparing to stop...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)


def gravity_compensation_controller(r: RebotArm, dt: float) -> None:
    # 获取当前关节位置 / Get current joint positions
    q = r.arm.get_positions(request_feedback=False)
    # 计算广义重力向量 / Compute generalized gravity vector
    tau_g = compute_generalized_gravity(q=q)

    # MIT 模式: 位置闭环 + 重力前馈
    # MIT mode: position closed-loop + gravity feedforward
    r.arm.send_mit(
        pos=q,
        vel=np.zeros(r.arm.num_joints),
        kp=np.full(r.arm.num_joints, 2.0),
        kd=np.full(r.arm.num_joints, 1.0),
        tau=tau_g,
    )
    if r.has_gripper:
        r.gripper.send_mit(r.gripper.get_positions())

    gravity_compensation_controller._counter += 1
    if gravity_compensation_controller._counter % 20 == 0:
        print(
            f"[{gravity_compensation_controller._counter:4d}] "
            f"tau_g = " + "  ".join(f"{t:+.3f}" for t in tau_g) + "  N·m"
        )


gravity_compensation_controller._counter = 0


def main() -> None:
    print("=" * 60)
    print("  reBotArm 重力补偿演示")
    print("  reBotArm gravity compensation demo")
    print("  预计行为 / Expected behavior: 机械臂维持位置不动，可以手动掰动至任何位置")
    print("               The arm holds position and can be manually moved to any pose")
    print("  Ctrl+C 停止并断开连接 / Ctrl+C to stop and disconnect")
    print("=" * 60)

    model = load_dynamics_model()
    g_vec = get_default_gravity()
    print(f"\n[模型 / Model] nq={model.nq}, nv={model.nv}")
    print(f"[重力 / Gravity] {g_vec}  m/s²")

    rebotarm = RebotArm()
    rebotarm.connect()
    rebotarm.arm.mode_mit()
    rebotarm.gripper.mode_mit()
    rebotarm.disable_all()
    time.sleep(0.1)
    if ENABLED_JOINTS:
        for name in ENABLED_JOINTS:
            if name in rebotarm._motor_map:
                rebotarm._motor_map[name].enable()
        print(f"[安全模式 / Safety mode] 仅使能电机 / Motors enabled: {ENABLED_JOINTS}")
    else:
        rebotarm.enable_all()
        print("[使能 / Enabled] 全部电机已使能 / All motors enabled")
    rebotarm.start_control_loop(gravity_compensation_controller, rate=rebotarm.rate)
    print(f"[控制循环 / Control loop] 启动 @ {rebotarm.rate} Hz")
    print("-" * 60)
    print(f"{'step':>4}  tau_g (N·m)")
    print("-" * 60)

    try:
        while _running:
            time.sleep(0.01)
    finally:
        print("\n[停止 / Stopping] 关闭控制循环... / Closing control loop...")
        rebotarm.disconnect()
        print("[完成 / Done] 已安全断开连接 / Safely disconnected")


if __name__ == "__main__":
    main()
