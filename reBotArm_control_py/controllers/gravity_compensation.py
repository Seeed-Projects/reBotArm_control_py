"""GravityCompensation — MIT 重力前馈控制器。

把 ``example/9_gravity_compensation.py`` 中已验证的控制律做成可复用 API，
供 demo 和外部工程（例如 Isaac Sim 关节镜像）共用同一套补偿。

控制律（MIT 模式）：
    arm 组: 当前位置闭环 + Pinocchio ``g(q)`` 前馈
    gripper 组: MIT 保持当前位置

使用示例::

    from reBotArm_control_py.actuator import RebotArm
    from reBotArm_control_py.controllers import GravityCompensation

    rebotarm = RebotArm()
    ctrl = GravityCompensation(rebotarm)
    ctrl.start()
    # 循环外可读 rebotarm.arm.get_positions() / ctrl.last_tau_g
    ctrl.end()
"""

from __future__ import annotations

import time
from typing import Sequence

import numpy as np

from ..actuator import RebotArm
from ..dynamics import compute_generalized_gravity

DEFAULT_KP = 2.0
DEFAULT_KD = 1.0


class GravityCompensation:
    """MIT 位置闭环 + 重力前馈。

    参数与 ``example/9_gravity_compensation.py`` 对齐：``kp=2.0``、``kd=1.0``，
    力矩直接使用 ``compute_generalized_gravity(q)``，不再额外放大。
    """

    def __init__(
        self,
        rebotarm: RebotArm,
        *,
        kp: float = DEFAULT_KP,
        kd: float = DEFAULT_KD,
        enabled_joints: Sequence[str] | None = None,
        log_every: int = 0,
    ) -> None:
        self.rebotarm = rebotarm
        self._kp = float(kp)
        self._kd = float(kd)
        self._enabled_joints = list(enabled_joints) if enabled_joints else []
        self._log_every = int(log_every)
        self._counter = 0
        self._running = False
        self._last_tau_g = np.zeros(0, dtype=np.float64)

    @property
    def last_tau_g(self) -> np.ndarray:
        """最近一次发给 arm 组的重力前馈力矩 ``g(q)``，单位 N·m。"""
        return self._last_tau_g.copy()

    def start(self) -> None:
        """连接总线、切 MIT、使能，并启动控制循环。"""
        if self._running:
            raise RuntimeError("GravityCompensation 已在运行，请先调用 end()")

        robot = self.rebotarm
        robot.connect()
        robot.arm.mode_mit()
        if robot.has_gripper:
            robot.gripper.mode_mit()
        robot.disable_all()
        time.sleep(0.1)
        if self._enabled_joints:
            for name in self._enabled_joints:
                if name in robot._motor_map:
                    robot._motor_map[name].enable()
            print(
                f"[安全模式 / Safety mode] 仅使能电机 / Motors enabled: "
                f"{self._enabled_joints}"
            )
        else:
            robot.enable_all()
            print("[使能 / Enabled] 全部电机已使能 / All motors enabled")

        self._counter = 0
        robot.start_control_loop(self._loop_cb, rate=robot.rate)
        self._running = True

    def end(self) -> None:
        """停止控制循环并断开连接。不回零，避免手掰姿态被强制拉回。"""
        if not self._running:
            return
        self.rebotarm.disconnect()
        self._running = False

    def __enter__(self) -> "GravityCompensation":
        return self

    def __exit__(self, *args) -> None:
        self.end()

    def _loop_cb(self, robot: RebotArm, dt: float) -> None:
        del dt
        q = robot.arm.get_positions()
        tau_g = compute_generalized_gravity(q=q)
        self._last_tau_g = np.asarray(tau_g, dtype=np.float64)

        n = robot.arm.num_joints
        tau = self._last_tau_g
        if tau.shape[0] < n:
            tau = np.pad(tau, (0, n - tau.shape[0]))
        elif tau.shape[0] > n:
            tau = tau[:n]
        robot.arm.send_mit(
            pos=q,
            vel=np.zeros(n),
            kp=np.full(n, self._kp),
            kd=np.full(n, self._kd),
            tau=tau,
        )
        if robot.has_gripper:
            robot.gripper.send_mit(robot.gripper.get_positions())

        self._counter += 1
        if self._log_every > 0 and self._counter % self._log_every == 0:
            print(
                f"[{self._counter:4d}] "
                f"tau_g = " + "  ".join(f"{t:+.3f}" for t in self._last_tau_g) + "  N·m"
            )
