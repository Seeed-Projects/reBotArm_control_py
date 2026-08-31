#!/usr/bin/env python3
"""MeshCat 可视化器封装 — 加载 URDF 并显示机器人模型。

用法:
    from example.sim.visualizer import Visualizer
    viz = Visualizer()
    viz.update(q)  # q: 关节角度 (nq,)

功能:
    - 绘制 3D 折线路径（参考轨迹 / 实际轨迹）
    - 播放关节轨迹动画（逐帧 + 路径同步）
    - 显示 IK 目标位姿（三色轴 + 球体标记）
"""

import sys
import time
from pathlib import Path

import meshcat
import meshcat.geometry as mcg
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from reBotArm_control_py.kinematics import _resolve_urdf


class Visualizer:
    """MeshCat + Pinocchio 机器人可视化器。

    支持：
        - 逐帧更新机械臂姿态
        - 绘制末端参考路径（灰色）和已走路径（绿色）
        - IK 目标位姿可视化（三色轴 + 球体）
        - 轨迹动画播放

    配置: config/rebotarm.yaml
    """

    def __init__(
        self,
        open_browser: bool = True,
        urdf_path: str | None = None,
    ):
        """初始化可视化器。

        Args:
            open_browser: 是否在终端打印 MeshCat 访问地址。
            urdf_path:    URDF 文件路径，留空则从 hardware_yaml 指向的硬件配置文件中读取。
        """
        urdf_path, package_dirs = _resolve_urdf(urdf_path)

        self._model = pin.buildModelFromUrdf(urdf_path)
        self._data = self._model.createData()

        self._visual_model = pin.buildGeomFromUrdf(
            self._model, urdf_path, pin.GeometryType.VISUAL, package_dirs=package_dirs
        )
        self._visual_data = self._visual_model.createData()

        # zmq_url=None 时启动新服务器；传入 zmq_url 字符串则连接到已有服务器
        self._meshcat_viz = meshcat.Visualizer(zmq_url=None)

        self._viz = MeshcatVisualizer(
            self._model,
            collision_model=None,
            visual_model=self._visual_model,
            data=self._data,
            visual_data=self._visual_data,
        )
        self._viz.initViewer(self._meshcat_viz, loadModel=False)
        self._viz.loadViewerModel()

        if open_browser:
            print(f"MeshCat 地址: {self._meshcat_viz.url()}")

    @property
    def meshcat(self):
        """暴露底层 meshcat.Visualizer 用于自定义节点操作。"""
        return self._meshcat_viz

    def update(self, q) -> None:
        """更新机器人显示位姿。q 可以是 list 或 np.ndarray。"""
        q = np.asarray(q)
        if q.shape != (self._model.nq,):
            raise ValueError(f"q 必须为形状 ({self._model.nq},)，实际为 {q.shape}")
        self._viz.display(q)

    def neutral(self) -> None:
        """恢复到中位配置。"""
        q0 = pin.neutral(self._model)
        self._viz.display(q0)

    @property
    def nq(self) -> int:
        return self._model.nq

    @property
    def model(self):
        """暴露 model 供外部调用（如 compute_fk）。"""
        return self._model

    # ── 路径绘制 ────────────────────────────────────────────────────────────────

    def draw_path(
        self,
        points_xyz: list,
        node_name: str,
        color: int = 0x00aaff,
    ) -> None:
        """在场景中绘制 3D 折线路径。

        Args:
            points_xyz: 三维点列表 [[x,y,z], ...]
            node_name:  MeshCat 节点名称（用于更新或删除）
            color:      RGB 十六进制颜色值（默认浅蓝色）
        """
        if len(points_xyz) < 2:
            return
        pts = np.array(points_xyz, dtype=np.float32).T
        line = mcg.Line(
            mcg.PointsGeometry(pts),
            mcg.LineBasicMaterial(color=color, linewidth=2),
        )
        self._meshcat_viz[node_name].set_object(line)

    def draw_ref_path(self, points_xyz: list) -> None:
        """绘制灰色参考路径（笛卡尔规划轨迹）。"""
        self.draw_path(points_xyz, "traj_path/ref", color=0x888888)

    def draw_actual_path(self, points_xyz: list, color: int = 0x00cc44) -> None:
        """绘制已走路径（绿色）。"""
        self.draw_path(points_xyz, "traj_path/actual", color=color)

    def clear_paths(self) -> None:
        """清除所有轨迹路径节点。"""
        for name in ("traj_path/ref", "traj_path/actual"):
            try:
                del self._meshcat_viz[name]
            except Exception:
                pass

    # ── IK 目标可视化 ───────────────────────────────────────────────────────────

    def show_ik_pose(
        self,
        xyz: np.ndarray,
        R: np.ndarray,
        q: np.ndarray,
    ) -> None:
        """显示 IK 求解结果（目标位姿 + 对应关节角）。

        可视化内容：
            - 目标位姿：三色坐标轴（RGB = XYZ）+ 红色球体标记
            - 机械臂：更新到求解出的关节角配置

        Args:
            xyz: 目标位置 [x, y, z]
            R:   3x3 旋转矩阵
            q:   对应的关节角 (nq,)
        """
        # 构建 4x4 齐次变换矩阵
        H = np.eye(4)
        H[:3, :3] = R
        H[:3, 3] = xyz

        # 显示目标坐标系（三色轴）
        self._meshcat_viz["target/frame"].set_object(mcg.triad())
        self._meshcat_viz["target/frame"].set_transform(H)

        # 显示目标位置标记（红色小球）
        self._meshcat_viz["target/ball"].set_object(
            mcg.Sphere(0.015),
            mcg.MeshLambertMaterial(color=0xFF3300),
        )
        self._meshcat_viz["target/ball"].set_transform(H)

        # 更新机械臂姿态
        self.update(np.asarray(q))

    def clear_ik_pose(self) -> None:
        """清除 IK 目标可视化。"""
        for name in ("target/frame", "target/ball"):
            try:
                del self._meshcat_viz[name]
            except Exception:
                pass

    # ── 轨迹线 ─────────────────────────────────────────────────────────────────

    def plot_trajectory_line(
        self,
        joint_traj: list,
        color: int = 0xFF3300,
        name: str = "ee_trajectory",
    ) -> None:
        """在 MeshCat 中绘制末端执行器轨迹线。

        Args:
            joint_traj: 关节角度列表，每个元素为 np.ndarray (nq,) 或 JointTrajectoryPoint
            color:      RGB 颜色值
            name:       MeshCat 中的路径名
        """
        from reBotArm_control_py.kinematics import compute_fk

        positions = []
        for pt in joint_traj:
            q = np.asarray(pt.q) if hasattr(pt, "q") else np.asarray(pt)
            _, _, T = compute_fk(self._model, q)
            positions.append(T[:3, 3])
        positions = np.array(positions, dtype=float)

        if len(positions) < 2:
            return
        self.clear_trajectory_line(name)
        self._meshcat_viz[name].set_object(
            mcg.Line(
                mcg.PointsGeometry(positions),
                mcg.LineBasicMaterial(color=color, linewidth=2),
            )
        )

    def clear_trajectory_line(self, name: str = "ee_trajectory") -> None:
        """清除 MeshCat 中的轨迹线。"""
        try:
            del self._meshcat_viz[name]
        except Exception:
            pass

    # ── 轨迹播放 ────────────────────────────────────────────────────────────────

    def play_trajectory(
        self,
        name: str,
        dt: float,
        q_list: list,
        path: list | None = None,
    ) -> None:
        """播放关节轨迹动画。

        工作流程：
            1. 绘制参考路径（灰色）
            2. 逐帧显示机械臂姿态（按 dt 间隔）
            3. 同步绘制已走路径（绿色）

        Args:
            name: 轨迹名称（用于日志输出）
            dt:   帧间时间间隔 [秒]
            q_list: 关节角序列 [[q1,...,qn], ...]
            path: 末端位置序列 [[x,y,z], ...]（可选）
        """
        print(
            f"[viz] 播放轨迹: {name}  点数={len(q_list)}  dt={dt:.3f}s",
            flush=True,
        )

        if path:
            self.draw_ref_path(path)

        visited = []
        for i, q in enumerate(q_list):
            self.update(np.asarray(q))
            if path and i < len(path):
                visited.append(path[i])
                self.draw_actual_path(visited)
            time.sleep(dt)

        print(f"[viz] 轨迹 '{name}' 完毕", flush=True)
        time.sleep(1.0)
