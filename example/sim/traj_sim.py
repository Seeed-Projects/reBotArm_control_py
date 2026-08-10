#!/usr/bin/env python3
"""测地线轨迹规划仿真 — 笛卡尔测地线 + CLIK 跟踪。

用法:
    python example/sim/traj_sim.py

交互:
    输入: x y z [roll pitch yaw]  (米 / 弧度)
    直接回车使用默认配置
    输入 q 退出

功能:
    - 从当前关节角出发，输入目标位姿
    - 使用 SE(3) 测地线插值 + CLIK 关节空间跟踪
    - 计算并显示轨迹统计信息
    - 在 MeshCat 中回放完整轨迹
"""

import sys
import time
import signal
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from reBotArm_control_py.kinematics import (
    compute_fk,
    get_end_effector_frame_id,
)
from reBotArm_control_py.trajectory import (
    plan_cartesian_geodesic_trajectory,
    track_trajectory,
    compute_traj_stats,
    TrajProfile,
    TrajPlanParams,
    IKParams,
)
from example.sim.visualizer import Visualizer

LINEAR_SPEED = 0.1
should_exit = False


def signal_handler(sig, frame):
    global should_exit
    should_exit = True


def make_pose(x: float, y: float, z: float,
              roll: float, pitch: float, yaw: float) -> pin.SE3:
    return pin.SE3(pin.rpy.rpyToMatrix(roll, pitch, yaw), np.array([x, y, z]))


def _solve_ik(model, end_frame_id, target, q_init, ik_params):
    from reBotArm_control_py.kinematics.inverse_kinematics import solve_ik
    data = model.createData()
    result = solve_ik(model, data, end_frame_id, target, q_init, ik_params)
    return result.q, result.success


def run_trajectory(viz, model, end_frame_id, q_start, q_end,
                   duration, dt=1.0/50.0, profile=TrajProfile.MIN_JERK,
                   accel_ratio=0.25, null_gain=0.1):
    """执行轨迹规划、CLIK 跟踪和动画回放。"""
    T_start = compute_fk(model, q_start)[2]
    T_end = compute_fk(model, q_end)[2]
    params = TrajPlanParams(dt=dt, profile=profile, accel_ratio=accel_ratio)
    ik_params = IKParams(max_iter=200, tolerance=1e-4, damping=1e-6, step_size=0.8)

    cart_result = plan_cartesian_geodesic_trajectory(T_start, T_end, duration, params)

    elapsed = time.time()
    joint_traj = track_trajectory(
        model, end_frame_id, cart_result.trajectory, q_start, ik_params, null_gain
    )
    elapsed = (time.time() - elapsed) * 1000.0

    stats = compute_traj_stats(
        model, end_frame_id, joint_traj, T_start, T_end, duration, params
    )

    times = np.array([pt.time for pt in joint_traj])
    ref_pts = cart_result.trajectory.points()
    data = model.createData()

    cart_errs = np.zeros(len(joint_traj))
    ee_positions = []
    for i, pt in enumerate(joint_traj):
        if i < len(ref_pts):
            pin.computeJointJacobians(model, data, pt.q)
            pin.updateFramePlacements(model, data)
            err_vec = pin.log6(
                data.oMf[end_frame_id].inverse() * ref_pts[i].pose
            ).vector
            cart_errs[i] = float(np.linalg.norm(err_vec))
        _, _, T = compute_fk(model, pt.q)
        ee_positions.append(T[:3, 3].tolist())

    ref_positions = [
        pt.pose.translation.tolist() if hasattr(pt.pose, 'translation')
        else pt.pose[:3, 3].tolist()
        for pt in ref_pts
    ]

    print(f"\n{'='*60}")
    print(f"  轨迹: {profile.value}  耗时={elapsed:.1f}ms  点数={len(joint_traj)}")
    print(f"  时长={duration:.2f}s  dt={dt}s  零空间={null_gain}")
    print(f"  关节: {np.degrees(q_start).round(1).tolist()} → {np.degrees(q_end).round(1).tolist()}")
    print(f"{'='*60}")
    print(f"  IK 成功率: {stats.success_rate:.1%}  "
          f"最大误差: {stats.max_ik_error:.3e}  "
          f"平均误差: {stats.avg_ik_error:.3e}")

    # ── MeshCat 回放 ──
    viz.clear_paths()
    viz.clear_trajectory_line()
    if ee_positions:
        viz.draw_ref_path(ref_positions)

    visited = []
    print("\n  播放动画 (MeshCat)...")
    for i, pt in enumerate(joint_traj):
        if should_exit:
            break
        viz.update(pt.q)
        if i < len(ee_positions):
            visited.append(ee_positions[i])
            viz.draw_actual_path(visited)
        if i < len(times) - 1:
            time.sleep(max(0.002, times[i + 1] - times[i]))
    print("  动画播放完毕。")

    if not should_exit:
        joint_arr = np.array([pt.q for pt in joint_traj])
        qv = np.diff(joint_arr, axis=0, prepend=joint_arr[:1])
        dt_arr = np.diff(times, axis=0, prepend=times[0])
        qv = np.divide(qv, dt_arr[:, None], where=dt_arr[:, None] > 0,
                       out=np.zeros_like(joint_arr))
        q_deg = np.degrees(joint_arr)
        qv_deg = np.degrees(qv)

        print(f"\n--- 统计摘要 ---")
        print(f"  关节角度 (deg):")
        for i in range(joint_arr.shape[1]):
            print(f"    j{i+1}: [{q_deg[:, i].min():.1f}, {q_deg[:, i].max():.1f}]")
        print(f"  关节速度 (deg/s):")
        for i in range(joint_arr.shape[1]):
            print(f"    j{i+1}: [{qv_deg[:, i].min():.1f}, {qv_deg[:, i].max():.1f}]")
        print(f"  笛卡尔误差 (m): avg={cart_errs.mean():.3e}, max={cart_errs.max():.3e}")

    return times, joint_traj, cart_errs, stats


def main():
    global should_exit
    signal.signal(signal.SIGINT, signal_handler)

    print("加载 MeshCat 可视化器...")
    viz = Visualizer(open_browser=True)
    model = viz.model
    end_frame_id = get_end_effector_frame_id(model)
    print(f"模型: {model.nq} 关节\n")

    ik_params = IKParams(max_iter=200, tolerance=1e-4, damping=1e-6, step_size=0.8)
    dt = 1.0 / 50.0
    q = pin.neutral(model).copy()
    q_last = q.copy()
    viz.update(q)
    print("已在零位显示机器人，打开 MeshCat 查看。")
    print('输入: x y z [roll pitch yaw] (米 / 弧度)，q 退出\n')

    while not should_exit:
        T0 = compute_fk(model, q)[2]
        p = T0[:3, 3]
        rpy = pin.rpy.matrixToRpy(T0[:3, :3])
        print(
            f'pos[{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}] '
            f'rpy[{rpy[0]:.3f} {rpy[1]:.3f} {rpy[2]:.3f}]> ',
            end="", flush=True
        )

        try:
            line = input().strip()
        except EOFError:
            break
        if not line:
            continue
        if line in ("q", "quit", "exit"):
            break

        parts = line.split()
        try:
            vals = [float(x) for x in parts]
        except ValueError:
            print("  格式: x y z [roll pitch yaw]")
            continue

        x, y, z = vals[0], vals[1], vals[2]
        roll = vals[3] if len(vals) >= 6 else 0.0
        pitch = vals[4] if len(vals) >= 6 else 0.0
        yaw = vals[5] if len(vals) >= 6 else 0.0
        target_pose = make_pose(x, y, z, roll, pitch, yaw)

        ik_res_q, ik_success = _solve_ik(
            model, end_frame_id, target_pose, q_last, ik_params
        )
        if not ik_success:
            print("  IK 无解\n")
            continue

        duration = max(1.0, np.linalg.norm(target_pose.translation - T0[:3, 3]) / LINEAR_SPEED)

        t0 = time.time()
        _, joint_traj, _, _ = run_trajectory(
            viz=viz, model=model, end_frame_id=end_frame_id,
            q_start=q_last, q_end=ik_res_q, duration=duration,
            dt=dt, profile=TrajProfile.MIN_JERK, accel_ratio=0.25, null_gain=0.1,
        )
        ms = (time.time() - t0) * 1000.0
        print(f"  总耗时: {ms:.1f} ms  点数: {len(joint_traj)}\n")

        q_last = joint_traj[-1].q.copy()
        viz.update(q_last)
        q = q_last.copy()

    viz.neutral()
    print("\n完成。")


if __name__ == "__main__":
    main()
