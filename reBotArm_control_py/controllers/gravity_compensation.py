"""GravityCompensation — MIT 重力前馈控制器。

把 ``example/9_gravity_compensation.py`` 中已验证的控制律做成可复用 API，
供 demo 和外部工程（例如 Isaac Sim 关节镜像）共用同一套补偿。

控制律（MIT 模式）：
    arm 组: 当前位置闭环 + Pinocchio ``g(q)`` 前馈
    gripper 组: MIT 保持当前位置

与 ROS 包 ``hardware_manager.py`` 对齐的柔顺重力补偿参数:

- ``tau_scale``: 每关节重力补偿力矩缩放系数，用于补偿模型与真机的偏差。
- ``transition_duration``: 从刚性保持增益平滑淡入到柔顺增益的时间（秒），
  避免启动时刚度突变。
- ``joint_direction``: 每关节方向修正系数（+1 或 -1），用于关节正方向与
  URDF 不一致的情况。
- ``hold_kp`` / ``hold_kd``: 刚性保持增益，默认从 ``arm`` 组的 MIT 配置读取。

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
DEFAULT_TAU_SCALE = 1.0
DEFAULT_TRANSITION_DURATION = 0.5
DEFAULT_JOINT_DIRECTION = 1.0


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
        tau_scale: float | Sequence[float] = DEFAULT_TAU_SCALE,
        transition_duration: float = DEFAULT_TRANSITION_DURATION,
        joint_direction: float | Sequence[float] = DEFAULT_JOINT_DIRECTION,
        hold_kp: float | Sequence[float] | None = None,
        hold_kd: float | Sequence[float] | None = None,
    ) -> None:
        self.rebotarm = rebotarm
        self._kp = float(kp)
        self._kd = float(kd)
        self._enabled_joints = list(enabled_joints) if enabled_joints else []
        self._log_every = int(log_every)
        self._counter = 0
        self._running = False
        self._last_tau_g = np.zeros(0, dtype=np.float64)

        self._tau_scale_raw = tau_scale
        self._joint_direction_raw = joint_direction
        self._transition_duration = float(transition_duration)
        self._hold_kp_raw = hold_kp
        self._hold_kd_raw = hold_kd

        # Resolved to per-joint arrays in start(), once arm joint count is known.
        self._tau_scale: np.ndarray | None = None
        self._joint_direction: np.ndarray | None = None
        self._compliant_kp: np.ndarray | None = None
        self._compliant_kd: np.ndarray | None = None
        self._hold_kp: np.ndarray | None = None
        self._hold_kd: np.ndarray | None = None
        self._transition_q_hold: np.ndarray | None = None
        self._transition_started_at: float | None = None
        self._gc_model = None
        self._gc_data = None

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

        n = robot.arm.num_joints
        self._resolve_params(n)

        # Read the real pose before changing mode.  This pose, not zero, is
        # the first MIT target when gravity compensation starts away from home.
        q_hold = robot.arm.get_positions().copy()
        tau_hold = self._gravity_comp_torque(q_hold)

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

        # Begin at the already-active position-hold gains.  The control
        # loop follows measured position while smoothly reducing these
        # gains to the compliant gravity-compensation values.
        self._transition_q_hold = q_hold.copy()
        self._transition_started_at = None

        self._counter = 0
        robot.start_control_loop(self._loop_cb, rate=robot.rate)
        # Start the fade only after every motor has entered MIT and is
        # holding the measured pose.  Mode-switch latency must not consume
        # part of the configured transition interval.
        self._transition_started_at = time.perf_counter()
        self._running = True

    def end(self) -> None:
        """停止重力补偿并维持当前位置（与网页停止重补行为一致）。

        停止控制循环，用刚性增益 + 重力前馈保持当前位置。
        电机保持使能，不断开 CAN。如需安全归零并断开，
        请在调用 end() 后执行 safe_home() + rebotarm.disconnect()。
        """
        if not self._running:
            return
        robot = self.rebotarm
        robot.stop_control_loop()

        # Hold current position with stiff gains + gravity feedforward,
        # matching the web "stop gravity compensation" behaviour.
        q_hold = robot.arm.get_positions().copy()
        n = robot.arm.num_joints
        tau_hold = self._gravity_comp_torque(q_hold)
        robot.arm.send_mit(
            q_hold, vel=np.zeros(n),
            kp=self._hold_kp, kd=self._hold_kd, tau=tau_hold,
        )
        if robot.has_gripper:
            robot.gripper.send_mit(robot.gripper.get_positions())

        self._running = False

    def __enter__(self) -> "GravityCompensation":
        return self

    def __exit__(self, *args) -> None:
        self.end()

    # ------------------------------------------------------------------
    # transition helpers (ported from hardware_manager.py)
    # ------------------------------------------------------------------

    def _resolve_params(self, n: int) -> None:
        """Resolve scalar-or-list params to per-joint arrays of length n."""
        self._tau_scale = self._resolve_vector(self._tau_scale_raw, n, "tau_scale")
        self._joint_direction = self._resolve_vector(
            self._joint_direction_raw, n, "joint_direction"
        )
        self._compliant_kp = np.full(n, float(self._kp))
        self._compliant_kd = np.full(n, float(self._kd))
        self._hold_kp = (
            self._resolve_vector(self._hold_kp_raw, n, "hold_kp")
            if self._hold_kp_raw is not None
            else getattr(
                self.rebotarm.arm, "_mit_kp", np.full(n, float(self._kp))
            ).copy()
        )
        self._hold_kd = (
            self._resolve_vector(self._hold_kd_raw, n, "hold_kd")
            if self._hold_kd_raw is not None
            else getattr(
                self.rebotarm.arm, "_mit_kd", np.full(n, float(self._kd))
            ).copy()
        )

        from ..kinematics import load_robot_model
        from ..kinematics.robot_model import pad_q_for_model

        self._gc_model = load_robot_model()
        self._gc_data = self._gc_model.createData()
        self._pad_q_for_model = pad_q_for_model

    @staticmethod
    def _resolve_vector(
        value: float | Sequence[float], n: int, label: str
    ) -> np.ndarray:
        if isinstance(value, (int, float)):
            return np.full(n, float(value))
        arr = np.asarray(value, dtype=np.float64).reshape(-1)
        if len(arr) == 1:
            return np.full(n, float(arr[0]))
        if len(arr) != n:
            raise ValueError(f"{label} must be a scalar or {n} values")
        return arr

    def _gravity_comp_torque(self, q: np.ndarray) -> np.ndarray:
        """Compute scaled, direction-corrected gravity torque (N*m)."""
        q_for_model = q * self._joint_direction
        n = len(q)
        q_model = self._pad_q_for_model(self._gc_model, q_for_model, n)
        tau_model = compute_generalized_gravity(
            self._gc_model, q_model, self._gc_data
        )[:n]
        return tau_model * self._joint_direction * self._tau_scale

    def _transition_blend(self, elapsed: float) -> float:
        """Smoothstep blend factor in [0, 1] over transition_duration."""
        ratio = float(
            np.clip(
                elapsed / max(self._transition_duration, 1e-6),
                0.0,
                1.0,
            )
        )
        # Smoothstep has zero slope at both ends, avoiding a gain derivative
        # step when entering the compliant mode.
        return ratio * ratio * (3.0 - 2.0 * ratio)

    # ------------------------------------------------------------------
    # safe shutdown — minimum-jerk home before disconnect
    # ------------------------------------------------------------------

    def safe_home(
        self,
        max_vel: float = 0.5,
        send_freq: float = 50.0,
        settle: float = 0.5,
        timeout: float = 15.0,
    ) -> None:
        """Minimum-jerk 归零轨迹 + 重力前馈。

        从当前位置平滑运动到零位，使用刚性增益保持稳定。
        应在 ``end()`` 之后、``disconnect()`` 之前调用。
        """
        if self._hold_kp is None:
            raise RuntimeError("safe_home() 需先调用 start()")
        robot = self.rebotarm
        q_hold = robot.arm.get_positions().copy()
        n = robot.arm.num_joints
        home = np.zeros(n)
        kp = self._hold_kp
        kd = self._hold_kd

        # Hold current position briefly to stabilize before moving.
        tau_hold = self._gravity_comp_torque(q_hold)
        robot.arm.send_mit(q_hold, vel=np.zeros(n), kp=kp, kd=kd, tau=tau_hold)
        if robot.has_gripper:
            robot.gripper.send_mit(robot.gripper.get_positions())
        time.sleep(0.3)

        q_err = np.abs(home - q_hold)
        max_err = float(np.max(q_err))
        if max_err < 0.01:
            return

        t_ramp = max_err / max_vel
        t_total = t_ramp * 2.0
        dt_send = 1.0 / send_freq
        num_steps = max(2, int(t_total / dt_send))
        interval = t_total / num_steps

        deadline = time.monotonic() + timeout
        print(
            f"[safe_home] 归零轨迹 / homing trajectory: "
            f"{num_steps} steps @ {send_freq:.0f} Hz, {t_total:.1f}s"
        )
        for i in range(num_steps):
            if time.monotonic() > deadline:
                print("[safe_home] 超时 / timeout")
                break
            s = (i + 1) / num_steps
            # Minimum-jerk: q(s) = q0 + Δq * (10s³ - 15s⁴ + 6s⁵)
            q_traj = q_hold + (home - q_hold) * (
                10.0 * s ** 3 - 15.0 * s ** 4 + 6.0 * s ** 5
            )
            tau_traj = self._gravity_comp_torque(q_traj)
            robot.arm.send_mit(
                q_traj, vel=np.zeros(n), kp=kp, kd=kd, tau=tau_traj
            )
            if robot.has_gripper:
                robot.gripper.send_mit(robot.gripper.get_positions())
            time.sleep(interval)

        # Settle at zero before cutting power.
        robot.arm.send_mit(home, vel=np.zeros(n), kp=kp, kd=kd, tau=np.zeros(n))
        time.sleep(settle)

    def _loop_cb(self, robot: RebotArm, dt: float) -> None:
        del dt
        q = robot.arm.get_positions()
        tau_motor = self._gravity_comp_torque(q)
        self._last_tau_g = np.asarray(tau_motor, dtype=np.float64)

        n = robot.arm.num_joints
        tau = self._last_tau_g
        if tau.shape[0] < n:
            tau = np.pad(tau, (0, n - tau.shape[0]))
        elif tau.shape[0] > n:
            tau = tau[:n]

        # Fade from stiff hold gains to compliant gains without a sudden
        # loss of stiffness at gravity-compensation startup.
        transition_started_at = self._transition_started_at
        elapsed = (
            self._transition_duration
            if transition_started_at is None
            else max(0.0, time.perf_counter() - transition_started_at)
        )
        blend = self._transition_blend(elapsed)
        kp = self._hold_kp + blend * (self._compliant_kp - self._hold_kp)
        kd = self._hold_kd + blend * (self._compliant_kd - self._hold_kd)
        q_hold = self._transition_q_hold
        q_target = (
            q.copy()
            if q_hold is None
            else q_hold + blend * (q - q_hold)
        )
        if blend >= 1.0:
            self._transition_q_hold = None

        robot.arm.send_mit(
            pos=q_target,
            vel=np.zeros(n),
            kp=kp,
            kd=kd,
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
