#!/usr/bin/env python3
"""reBotArm 重力补偿控制演示。

使用 Pinocchio 计算当前关节构型下的广义重力向量 g(q)，
通过 MIT 模式的前馈力矩直接补偿重力。

控制律由 ``reBotArm_control_py.controllers.GravityCompensation`` 提供，
本脚本只负责演示入口和安全测试开关。

安全测试: 设置 ENABLED_JOINTS 只使能部分电机，其他电机保持失能状态。
默认为全部使能；修改为只包含要测试的关节名即可，例如 ["joint1", "joint2"]。

reBotArm gravity compensation control demo.

Uses Pinocchio to compute the generalized gravity vector g(q) for the
current joint configuration, and applies gravity feedforward via MIT mode.

The control law lives in ``reBotArm_control_py.controllers.GravityCompensation``;
this script is only the demo entry point and the safety-test switch.

Safety test: set ENABLED_JOINTS to enable only a subset of motors; all
other motors remain disabled. Default is all enabled; set to a list of
joint names to test motors individually, e.g. ["joint1", "joint2"].
"""
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reBotArm_control_py.actuator import RebotArm
from reBotArm_control_py.controllers import GravityCompensation
from reBotArm_control_py.dynamics import (
    load_dynamics_model,
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
    ctrl = GravityCompensation(
        rebotarm,
        enabled_joints=ENABLED_JOINTS,
        log_every=20,
    )
    ctrl.start()
    print(f"[控制循环 / Control loop] 启动 @ {rebotarm.rate} Hz")
    print("-" * 60)
    print(f"{'step':>4}  tau_g (N·m)")
    print("-" * 60)

    try:
        while _running:
            time.sleep(0.01)
    finally:
        print("\n[停止 / Stopping] 关闭控制循环... / Closing control loop...")
        ctrl.end()
        print("[完成 / Done] 已安全断开连接 / Safely disconnected")


if __name__ == "__main__":
    main()
