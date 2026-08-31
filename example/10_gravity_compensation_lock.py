#!/usr/bin/env python3
"""reBotArm 重力补偿控制演示（RS 末端速度锁止版）。

在案例 9 的重力补偿控制器上增加末端速度锁止：

* ``STARTUP``：从刚性保持平滑过渡到锁止增益。
* ``LOCKED``：末端静止时保持目标关节角度。
* ``FOLLOW``：末端速度超过释放阈值后，目标角度跟随当前位置，
  允许手动推动。

RobStride 固件的 ``mechVel`` 反馈并非可靠的 rad/s，
因此本例只读取关节位置，使用相邻位置样本和真实采样间隔计算关节速度，
再经低通滤波后计算末端速度。释放和重新锁定使用不同阈值，
并要求持续静止一段时间，以避免阈值附近抖动。

控制律（MIT 模式）：

    tau = tau_scale * g(q) + integral_term
    pos = q_target
    kp = 8.0, kd = 1.0

积分项仅在 ``LOCKED`` 状态累积，并使用真实 ``dt``。
重力缩放使用当前 RS 标定值。
"""

from __future__ import annotations

import math
import signal
import sys
import time
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reBotArm_control_py.actuator import (
    RebotArm,
    load_gravity_compensation_config,
)
from reBotArm_control_py.controllers import GravityCompensation
from reBotArm_control_py.dynamics import get_default_gravity, load_dynamics_model
from reBotArm_control_py.kinematics import get_end_effector_frame

# 只使能以下关节；留空 [] 则全部使能。
# Only enable the listed joints; empty [] means all joints.
ENABLED_JOINTS: list[str] = []
# ENABLED_JOINTS: list[str] = ["joint1"]
# ENABLED_JOINTS: list[str] = ["joint1", "joint2"]

_running = True


def _sigint_handler(signum, frame) -> None:
    del signum, frame
    global _running
    print("\n[gravity_lock] 收到 Ctrl+C，准备停止... / Preparing to stop...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)


class VelocityLockGravityCompensation(GravityCompensation):
    """案例 9 重力补偿 + 基于位置差分速度的末端锁止。"""

    def __init__(self, robot: RebotArm) -> None:
        gravity_cfg = load_gravity_compensation_config("velocity_lock")
        super().__init__(
            robot,
            kp=float(gravity_cfg.get("kp", 8.0)),
            kd=float(gravity_cfg.get("kd", 1.0)),
            tau_scale=gravity_cfg.get("tau_scale", 1.0),
            transition_duration=float(
                gravity_cfg.get("transition_duration", 0.5)
            ),
            enabled_joints=ENABLED_JOINTS,
            log_every=int(gravity_cfg.get("log_every", 20)),
        )
        self._gravity_cfg = gravity_cfg
        self._ki = float(gravity_cfg.get("ki", 1.0))
        self._integral_limit = float(gravity_cfg.get("integral_limit", 0.3))
        self._linear_release_threshold = float(
            gravity_cfg.get("linear_release_threshold", 0.04)
        )
        self._angular_release_threshold = float(
            gravity_cfg.get("angular_release_threshold", 0.08)
        )
        self._linear_lock_threshold = float(
            gravity_cfg.get("linear_lock_threshold", 0.02)
        )
        self._angular_lock_threshold = float(
            gravity_cfg.get("angular_lock_threshold", 0.04)
        )
        self._lock_settle_duration = float(
            gravity_cfg.get("lock_settle_duration", 0.15)
        )
        self._velocity_filter_time_constant = float(
            gravity_cfg.get("velocity_filter_time_constant", 0.03)
        )
        self._max_valid_sample_dt = float(
            gravity_cfg.get("max_valid_sample_dt", 0.1)
        )
        self._q_target: np.ndarray | None = None
        self._q_previous: np.ndarray | None = None
        self._sample_time_previous: float | None = None
        self._qd_filtered: np.ndarray | None = None
        self._integral = np.zeros(0, dtype=np.float64)
        self._following = False
        self._quiet_since: float | None = None
        self._ee_frame_id: int | None = None
        self._lock_status = "STARTUP"

    def start(self) -> None:
        """Reset lock state, then use the safe startup sequence from example 9."""
        self._q_target = None
        self._q_previous = None
        self._sample_time_previous = None
        self._qd_filtered = None
        self._integral = np.zeros(0, dtype=np.float64)
        self._following = False
        self._quiet_since = None
        self._ee_frame_id = None
        self._lock_status = "STARTUP"
        super().start()

    def _estimate_joint_velocity(
        self,
        q: np.ndarray,
        now: float,
        nominal_dt: float,
    ) -> tuple[np.ndarray, float]:
        """Estimate RS joint velocity from position samples and low-pass it."""
        if self._q_previous is None or self._sample_time_previous is None:
            self._q_previous = q.copy()
            self._sample_time_previous = now
            self._qd_filtered = np.zeros_like(q)
            return self._qd_filtered.copy(), nominal_dt

        sample_dt = now - self._sample_time_previous
        q_previous = self._q_previous
        self._q_previous = q.copy()
        self._sample_time_previous = now

        if sample_dt <= 0.0 or sample_dt > self._max_valid_sample_dt:
            self._qd_filtered = np.zeros_like(q)
            return self._qd_filtered.copy(), nominal_dt

        qd_raw = (q - q_previous) / sample_dt
        if self._qd_filtered is None:
            self._qd_filtered = qd_raw
        else:
            alpha = 1.0 - math.exp(
                -sample_dt / max(self._velocity_filter_time_constant, 1e-6)
            )
            self._qd_filtered += alpha * (qd_raw - self._qd_filtered)
        return self._qd_filtered.copy(), sample_dt

    def _end_effector_speed(
        self,
        q: np.ndarray,
        qd: np.ndarray,
    ) -> tuple[float, float]:
        model = self._gc_model
        data = self._gc_data
        if model is None or data is None:
            raise RuntimeError("Gravity compensation model is not initialized")

        if self._ee_frame_id is None:
            ee_frame = get_end_effector_frame()
            frame_id = model.getFrameId(ee_frame)
            if frame_id >= model.nframes:
                raise ValueError(f"End-effector frame not found: {ee_frame}")
            self._ee_frame_id = frame_id

        direction = self._joint_direction
        q_for_model = q if direction is None else q * direction
        qd_for_model = qd if direction is None else qd * direction
        q_model = self._pad_q_for_model(model, q_for_model, len(q))
        qd_model = self._pad_q_for_model(model, qd_for_model, len(qd))

        pin.computeJointJacobians(model, data, q_model)
        pin.updateFramePlacements(model, data)
        jacobian = pin.getFrameJacobian(
            model,
            data,
            self._ee_frame_id,
            pin.ReferenceFrame.WORLD,
        )
        spatial_velocity = jacobian @ qd_model
        linear_speed = float(np.linalg.norm(spatial_velocity[:3]))
        angular_speed = float(np.linalg.norm(spatial_velocity[3:]))
        return linear_speed, angular_speed

    def _update_lock_target(
        self,
        q: np.ndarray,
        linear_speed: float,
        angular_speed: float,
        now: float,
    ) -> str:
        if self._q_target is None:
            self._q_target = q.copy()

        release = (
            linear_speed > self._linear_release_threshold
            or angular_speed > self._angular_release_threshold
        )
        quiet = (
            linear_speed < self._linear_lock_threshold
            and angular_speed < self._angular_lock_threshold
        )

        if not self._following and release:
            self._following = True
            self._quiet_since = None
            self._q_target = q.copy()
            self._integral.fill(0.0)

        if self._following:
            # While pushed, make the position loop transparent by following q.
            self._q_target = q.copy()
            self._integral.fill(0.0)
            if quiet:
                if self._quiet_since is None:
                    self._quiet_since = now
                elif now - self._quiet_since >= self._lock_settle_duration:
                    self._following = False
                    self._quiet_since = None
                    self._q_target = q.copy()
            else:
                self._quiet_since = None

        return "FOLLOW" if self._following else "LOCKED"

    def _loop_cb(self, robot: RebotArm, dt: float) -> None:
        # Read position once. In particular, do not call RS get_velocities().
        q = robot.arm.get_positions()
        now = time.perf_counter()
        qd, sample_dt = self._estimate_joint_velocity(q, now, dt)
        linear_speed, angular_speed = self._end_effector_speed(q, qd)

        tau_g = self._gravity_comp_torque(q)
        self._last_tau_g = np.asarray(tau_g, dtype=np.float64)
        n = robot.arm.num_joints
        if self._integral.shape != (n,):
            self._integral = np.zeros(n, dtype=np.float64)

        # Reuse example 9's smooth transition from the initial stiff hold.
        transition_started_at = self._transition_started_at
        elapsed = (
            0.0
            if transition_started_at is None
            else max(0.0, now - transition_started_at)
        )
        blend = self._transition_blend(elapsed)
        kp = self._hold_kp + blend * (self._compliant_kp - self._hold_kp)
        kd = self._hold_kd + blend * (self._compliant_kd - self._hold_kd)

        if blend < 1.0:
            q_hold = self._transition_q_hold
            q_target = q.copy() if q_hold is None else q_hold + blend * (q - q_hold)
            self._q_target = q.copy()
            self._integral.fill(0.0)
            self._following = False
            self._quiet_since = None
            status = "STARTUP"
        else:
            self._transition_q_hold = None
            status = self._update_lock_target(
                q,
                linear_speed,
                angular_speed,
                now,
            )
            q_target = self._q_target
            if status == "LOCKED":
                integral_dt = min(max(sample_dt, 0.0), 0.02)
                self._integral += self._ki * (q_target - q) * integral_dt
                np.clip(
                    self._integral,
                    -self._integral_limit,
                    self._integral_limit,
                    out=self._integral,
                )

        self._lock_status = status
        robot.arm.send_mit(
            pos=q_target,
            vel=np.zeros(n),
            kp=kp,
            kd=kd,
            tau=self._last_tau_g + self._integral,
        )
        if robot.has_gripper:
            robot.gripper.send_mit(robot.gripper.get_positions())

        self._counter += 1
        if self._log_every > 0 and self._counter % self._log_every == 0:
            max_error_deg = float(np.rad2deg(np.max(np.abs(q_target - q))))
            print(
                f"[{self._counter:4d}] {status:<7} "
                f"v={linear_speed:.4f}m/s  w={angular_speed:.4f}rad/s  "
                f"err={max_error_deg:.2f}deg  "
                f"tau_g="
                + "  ".join(f"{value:+.3f}" for value in self._last_tau_g)
                + "  N*m"
            )


def main() -> None:
    print("=" * 68)
    print("  reBotArm 重力补偿演示（RS 末端速度锁止版）")
    print("  reBotArm gravity compensation demo (RS EE velocity lock)")
    model = load_dynamics_model()
    print(f"\n[模型 / Model] nq={model.nq}, nv={model.nv}")
    print(f"[重力 / Gravity] {get_default_gravity()} m/s^2")
    print(f"[末端帧 / EE frame] {get_end_effector_frame()}")

    robot = RebotArm()
    controller = VelocityLockGravityCompensation(robot)
    print(
        "  释放阈值 / Release: "
        f"{controller._linear_release_threshold} m/s, "
        f"{controller._angular_release_threshold} rad/s"
    )
    print(
        "  锁定阈值 / Re-lock: "
        f"{controller._linear_lock_threshold} m/s, "
        f"{controller._angular_lock_threshold} rad/s "
        f"for {controller._lock_settle_duration}s"
    )
    print(
        "  重力缩放 / Gravity scale: "
        f"{controller._gravity_cfg.get('tau_scale', 1.0)}"
    )
    print("  预计行为：静止时锁定；用力推动时跟随；停止后重新锁定")
    print("  Ctrl+C 停止并安全归零 / Stop and safely return home")
    print("=" * 68)
    controller.start()
    print(f"[控制循环 / Control loop] 启动 @ {robot.rate} Hz")

    try:
        while _running:
            time.sleep(0.01)
    finally:
        print("\n[停止 / Stopping] 关闭控制循环...")
        controller.end()
        print("[归零 / Safe home] 最小 jerk 轨迹归零...")
        controller.safe_home()
        robot.disconnect()
        print("[完成 / Done] 已安全归零并断开连接")


if __name__ == "__main__":
    main()
