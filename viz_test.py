"""让仿真机械臂逐步画爱心并写 LOVE：uv run python viz_test.py。"""

import time

import numpy as np
import pinocchio as pin

from example.sim.visualizer import Visualizer
from reBotArm_control_py.kinematics import (
    IKSolverParams,
    compute_fk,
    get_end_effector_frame_id,
    solve_ik,
)


def plan_heart(model, frames=360):
    """在 YZ 竖直平面规划爱心，并逐点求解末端位置 IK。"""
    q = pin.neutral(model)
    q[:6] = np.deg2rad([0, 60, 60, 0, 0, 0])
    q = np.clip(q, model.lowerPositionLimit, model.upperPositionLimit)
    center, rotation, _ = compute_fk(model, q)
    t = np.linspace(0, 2 * np.pi, frames + 1)
    # 标准爱心参数方程，宽约 16 厘米，位于机械臂前方。
    points = np.tile(center, (len(t), 1))
    points[:, 1] += 0.005 * 16 * np.sin(t) ** 3
    points[:, 2] += 0.005 * (
        13 * np.cos(t) - 5 * np.cos(2 * t)
        - 2 * np.cos(3 * t) - np.cos(4 * t)
    )

    data = model.createData()
    frame_id = get_end_effector_frame_id(model)
    params = IKSolverParams(max_iter=300, tolerance=1e-5)
    configurations = []
    for i, point in enumerate(points):
        result = solve_ik(
            model, data, frame_id, pin.SE3(rotation, point), q,
            params=params, position_only=True,
        )
        if not result.success:
            raise RuntimeError(f"爱心第 {i + 1} 个点无法到达，误差 {result.error:.6f} 米")
        if configurations and np.max(np.abs(result.q - q)) > np.deg2rad(10):
            raise RuntimeError(f"爱心第 {i + 1} 个点发生关节跳变，停止播放")
        q = result.q.copy()
        configurations.append(q)
    return points, configurations


def plan_love(model, q_start):
    """规划爱心上方的字母笔画，以及不留线的抬笔移动。"""
    seed = pin.neutral(model)
    seed[:6] = np.deg2rad([0, 60, 60, 0, 0, 0])
    seed = np.clip(seed, model.lowerPositionLimit, model.upperPositionLimit)
    center, rotation, _ = compute_fk(model, seed)
    # 每个字母宽 3.2 厘米，高 4.5 厘米，字母间隔 1 厘米。
    theta = np.linspace(np.pi / 2, np.pi / 2 + 2 * np.pi, 81)
    letter_strokes = [
        (0, [(0, 1), (0, 0), (1, 0)]),  # L
        (1, np.column_stack((0.5 + 0.5 * np.cos(theta),
                             0.5 + 0.5 * np.sin(theta)))),  # O
        (2, [(0, 1), (0.5, 0), (1, 1)]),  # V
        (3, [(1, 1), (0, 1), (0, 0), (1, 0)]),  # E 外侧
        (3, [(0, 0.5), (0.85, 0.5)]),  # E 中间横线
    ]
    data = model.createData()
    frame_id = get_end_effector_frame_id(model)
    params = IKSolverParams(max_iter=300, tolerance=1e-5)
    q = q_start.copy()
    previous = compute_fk(model, q)[0]
    segments = []

    def solve_points(points, drawing):
        nonlocal q, previous
        configurations = []
        for point in points:
            result = solve_ik(
                model, data, frame_id, pin.SE3(rotation, point), q,
                params=params, position_only=True,
            )
            if not result.success:
                raise RuntimeError(f"LOVE 轨迹无法到达，误差 {result.error:.6f} 米")
            if np.max(np.abs(result.q - q)) > np.deg2rad(10):
                raise RuntimeError("LOVE 轨迹发生关节跳变，停止播放")
            q = result.q.copy()
            configurations.append(q)
        previous = np.asarray(points[-1])
        segments.append((drawing, np.asarray(points), configurations))

    def sample_polyline(vertices):
        samples = []
        for a, b in zip(vertices[:-1], vertices[1:]):
            count = max(2, int(np.ceil(np.linalg.norm(b - a) / 0.001)))
            samples.extend(np.linspace(a, b, count + 1)[1:])
        return np.asarray(samples)

    for index, stroke in letter_strokes:
        stroke = np.asarray(stroke)
        vertices = np.tile(center, (len(stroke), 1))
        vertices[:, 1] += -0.079 + index * 0.042 + stroke[:, 0] * 0.032
        vertices[:, 2] += 0.09 + stroke[:, 1] * 0.045
        # 沿 X 方向离开书写平面，再移动到下一笔起点并落笔。
        lift = np.array([-0.02, 0.0, 0.0])
        travel = sample_polyline(np.array([
            previous, previous + lift, vertices[0] + lift, vertices[0],
        ]))
        solve_points(travel, False)
        points = np.vstack((vertices[0], sample_polyline(vertices)))
        solve_points(points, True)
    return segments


def main():
    viz = Visualizer()
    viz.neutral()
    print("正在计算爱心和 LOVE 轨迹……", flush=True)
    reference, configurations = plan_heart(viz.model)
    love_segments = plan_love(viz.model, configurations[-1])

    try:
        input("请先在浏览器打开上面的 MeshCat 地址，再回到终端按回车开始画爱心和 LOVE……")

        # 先用 2 秒移动到爱心起点，此段移动不留线。
        q_start = pin.neutral(viz.model)
        fps = 30
        started = time.monotonic()
        for frame in range(1, 61):
            t = frame / 60
            alpha = t * t * (3 - 2 * t)
            viz.update(pin.interpolate(viz.model, q_start, configurations[0], alpha))
            time.sleep(max(0.0, started + frame / fps - time.monotonic()))

        # 灰线为理想路径，红线为根据每帧关节位姿计算出的末端轨迹。
        viz.draw_ref_path(reference.tolist())
        visited = []
        errors = []
        print("开始画爱心：灰色为理想路径，红色为仿真末端轨迹，约 12 秒。", flush=True)
        started = time.monotonic()
        for i, q in enumerate(configurations):
            viz.update(q)
            position, _, _ = compute_fk(viz.model, q)
            visited.append(position)
            errors.append(np.linalg.norm(position - reference[i]))
            viz.draw_path(visited, "path_name", color=0xFF0000)
            time.sleep(max(0.0, started + (i + 1) / fps - time.monotonic()))

        print("爱心完成，开始在上方写 LOVE……", flush=True)
        for stroke_id, (drawing, points, qs) in enumerate(love_segments):
            if drawing:
                viz.draw_path(points, f"love/ref/{stroke_id}", color=0x888888)
            visited = []
            started = time.monotonic()
            for i, q in enumerate(qs):
                viz.update(q)
                position, _, _ = compute_fk(viz.model, q)
                if drawing:
                    visited.append(position)
                    errors.append(np.linalg.norm(position - points[i]))
                    viz.draw_path(visited, f"love/actual/{stroke_id}", color=0xFF0000)
                time.sleep(max(0.0, started + (i + 1) / fps - time.monotonic()))

        print(f"爱心和 LOVE 绘制完成，最大位置误差：{max(errors) * 1000:.3f} 毫米。")
        input("可拖动浏览器视角查看爱心和 LOVE，再次按回车退出……")
    except (EOFError, KeyboardInterrupt):
        pass


if __name__ == "__main__":
    main()
