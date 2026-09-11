"""reBot-DevArm 逆运动学模块。

基于阻尼最小二乘（CLIK）的闭环逆运动学算法，包含关节限位活动约束、
自适应阻尼和回退线搜索。
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Optional

import numpy as np
import pinocchio as pin

from .forward_kinematics import compute_fk


# ─── 参数与结果数据结构 ────────────────────────────────────────────────────────

@dataclass
class IKParams:
    """IK 求解器参数"""
    max_iter: int = 1000
    tolerance: float = 1e-4    # 收敛阈值 ||err||
    step_size: float = 0.5    # 每步更新的缩放系数
    damping: float = 1e-6      # Tikhonov 正则化系数 λ


@dataclass
class IKResult:
    """IK 求解结果"""
    q: np.ndarray
    success: bool
    error: float       # 最终 ||err||
    iterations: int


# Alias，与 C++ 头文件中的命名保持一致
IKSolverParams = IKParams


# ─── 工具函数 ──────────────────────────────────────────────────────────────────

def pos_rot_to_se3(
    pos: np.ndarray,
    rot: Optional[np.ndarray] = None,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
) -> pin.SE3:
    """从位置和旋转构建 pinocchio SE3 位姿。

    参数:
        pos:    (3,) 位置 [x, y, z]，单位：米。
        rot:    (3, 3) 旋转矩阵。若提供则忽略 rpy 参数。
        roll:  绕 X 轴转角（弧度），仅当 rot=None 时使用。
        pitch: 绕 Y 轴转角（弧度），仅当 rot=None 时使用。
        yaw:   绕 Z 轴转角（弧度），仅当 rot=None 时使用。

    返回:
        pin.SE3 目标末端位姿。
    """
    if rot is None:
        rot = pin.rpy.rpyToMatrix(roll, pitch, yaw)
    return pin.SE3(rot, pos)


def _clamp_config(model: pin.Model, q: np.ndarray) -> np.ndarray:
    """将 q 限制在关节限位范围内。

    非有限下限/上限分别视为无下限/无上限。
    """
    lo = np.where(np.isfinite(model.lowerPositionLimit),
                  model.lowerPositionLimit, -np.inf)
    hi = np.where(np.isfinite(model.upperPositionLimit),
                  model.upperPositionLimit, np.inf)
    clamped = np.maximum(q, lo)
    clamped = np.minimum(clamped, hi)
    return clamped


def _compute_error(
    model: pin.Model,
    data: pin.Data,
    end_frame_id: int,
    q: np.ndarray,
    target: pin.SE3,
    position_only: bool = False,
) -> tuple[float, np.ndarray]:
    """计算当前末端与目标之间的局部坐标系误差。

    返回:
        ``(err_norm, err_vector)``。仅位置模式返回 3 维线位移误差，
        完整位姿模式返回 6 维 twist。
    """
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    T_cur = data.oMf[end_frame_id]
    if position_only:
        # LOCAL 雅可比的线速度部分在末端局部坐标系中表达，因此位置
        # 误差也要从世界坐标系旋转到该坐标系中。
        err = T_cur.rotation.T @ (target.translation - T_cur.translation)
    else:
        err = pin.log6(T_cur.inverse() * target).vector
    return float(np.linalg.norm(err)), err


def _damped_step_with_active_limits(
    model: pin.Model,
    q: np.ndarray,
    J: np.ndarray,
    err: np.ndarray,
    damping: float,
    step_size: float,
) -> np.ndarray:
    """计算阻尼最小二乘步，并把已到限位且继续向外的关节设为活动约束。

    直接在求解后裁剪配置会破坏各关节速度之间的配合：一个关节被裁掉后，
    剩余关节的原始速度通常已经不是下降方向。这里移除被阻塞的雅可比列并
    重新求解，使位于限位上的初始构型也能离开奇异/边界位置。
    """
    free = np.ones(model.nv, dtype=bool)
    limit_eps = 1e-10
    dq = np.zeros(model.nv)

    for _ in range(model.nv + 1):
        J_free = J[:, free]
        dq.fill(0.0)
        if J_free.shape[1] == 0:
            return dq

        system = J_free @ J_free.T
        system.flat[::system.shape[0] + 1] += damping
        dq[free] = step_size * J_free.T @ np.linalg.solve(system, err)

        newly_blocked = np.zeros(model.nv, dtype=bool)
        # Pinocchio 的一般配置可能 nq != nv。当前机器人均为一维关节；
        # 对其它类型不臆造 q/v 映射，仍由后续配置裁剪保证限位安全。
        for joint in model.joints[1:]:
            if joint.nq != 1 or joint.nv != 1:
                continue
            iq = joint.idx_q
            iv = joint.idx_v
            lo = model.lowerPositionLimit[iq]
            hi = model.upperPositionLimit[iq]
            at_lower = np.isfinite(lo) and q[iq] <= lo + limit_eps
            at_upper = np.isfinite(hi) and q[iq] >= hi - limit_eps
            if (at_lower and dq[iv] < 0.0) or (at_upper and dq[iv] > 0.0):
                newly_blocked[iv] = True

        newly_blocked &= free
        if not np.any(newly_blocked):
            return dq
        free[newly_blocked] = False

    return dq


# ─── 核心求解器 ────────────────────────────────────────────────────────────────

def solve_ik(
    model: pin.Model,
    data: pin.Data,
    end_frame_id: int,
    target: pin.SE3,
    q_init: np.ndarray,
    params: Optional[IKParams] = None,
    controlled_joints: int | None = None,
    *,
    position_only: bool = False,
) -> IKResult:
    """阻尼最小二乘 CLIK 求解器。

      - LOCAL 坐标系雅可比
      - 自适应阻尼 lam = params.damping * max(1.0, prev_err * 10.0)
      - 回退线搜索（最多折半 8 次）

    参数:
        model:            Pinocchio 机器人模型。
        data:             Pinocchio 数据缓存（需外部创建并传入）。
        end_frame_id:     末端帧索引。
        target:           目标 SE3 位姿。
        q_init:           初始关节配置。若维度小于 model.nq，超出部分视为被动关节补 0；
                          若维度大于 model.nq，多余部分被忽略。
        params:           IK 参数，默认 IKParams{}。
        controlled_joints: 受控关节数量（默认为 model.nq）。
                          传入比 model.nq 小的值时，IK 在完整模型空间求解，
                          但 q_init 只需提供受控关节数，返回值也只截取受控部分。
                          这使得调用方无需感知 URDF 中被动关节的存在。
        position_only:     为 True 时只约束末端位置，不约束姿态。

    返回:
        IKResult，其中 q 为求解得到的关节角（维度与 q_init 一致）。
    """
    if params is None:
        params = IKParams()

    nq = model.nq
    n_ctrl = controlled_joints if controlled_joints is not None else nq

    # 补齐 q_init 到 model.nq
    q = np.zeros(nq)
    n_provided = min(q_init.shape[0], n_ctrl)
    q[:n_provided] = q_init[:n_provided]
    prev_err, err = _compute_error(
        model, data, end_frame_id, q, target, position_only,
    )

    # 初始误差即已满足容差时直接返回
    if prev_err < params.tolerance:
        return IKResult(q=q[:n_ctrl], success=True, error=prev_err, iterations=0)

    for iteration in range(params.max_iter):

        # LOCAL 系体雅可比
        pin.computeJointJacobians(model, data, q)
        J = pin.getFrameJacobian(model, data, end_frame_id, pin.LOCAL)
        if position_only:
            J = J[:3, :]

        # 自适应阻尼：误差较大时适当增加阻尼（Levenberg-Marquardt 风格）
        lam = params.damping * max(1.0, prev_err * 10.0)

        # 带活动限位约束的阻尼最小二乘。
        dq = _damped_step_with_active_limits(
            model, q, J, err, lam, params.step_size,
        )

        if float(np.linalg.norm(dq)) < 1e-12:
            return IKResult(
                q=q[:n_ctrl], success=False, error=prev_err,
                iterations=iteration,
            )

        # 回退线搜索：若新误差未减小则缩步，最多折半 8 次
        alpha = 1.0
        for _ in range(8):
            q_new = _clamp_config(model, pin.integrate(model, q, alpha * dq))
            new_err, err_new = _compute_error(
                model, data, end_frame_id, q_new, target, position_only,
            )
            if new_err < prev_err:
                q = q_new
                err = err_new
                prev_err = new_err
                if prev_err < params.tolerance:
                    return IKResult(
                        q=q[:n_ctrl], success=True, error=prev_err,
                        iterations=iteration + 1,
                    )
                break
            alpha *= 0.5
        else:
            # 当前活动约束下已找不到下降步，继续相同迭代不会改变结果。
            return IKResult(
                q=q[:n_ctrl], success=False, error=prev_err,
                iterations=iteration + 1,
            )

    # 循环结束后再次检查（可能刚收敛或误差已达机器精度）
    if prev_err < params.tolerance:
        return IKResult(q=q[:n_ctrl], success=True, error=prev_err, iterations=params.max_iter)
    return IKResult(q=q[:n_ctrl], success=False, error=prev_err, iterations=params.max_iter)


def solve_ik_with_retry(
    model: pin.Model,
    data: pin.Data,
    end_frame_id: int,
    target: pin.SE3,
    q_seed: np.ndarray,
    params: Optional[IKParams] = None,
    max_retries: int = 8,
) -> IKResult:
    """带随机重试的 IK 求解器。

      - 先用 q_seed 求解一次
      - 若失败则在关节限位内随机采样最多 max_retries 次
      - 返回误差最小的结果

    参数:
        model:        Pinocchio 机器人模型。
        data:         Pinocchio 数据缓存。
        end_frame_id: 末端帧索引。
        target:       目标 SE3 位姿。
        q_seed:       种子关节配置（会被更新为本次最优解）。
        params:       IK 参数。
        max_retries:  随机重试次数。

    返回:
        IKResult。
    """
    if params is None:
        params = IKParams()

    best = solve_ik(model, data, end_frame_id, target, q_seed, params)
    if best.success:
        q_seed[:] = best.q
        return best

    lo = model.lowerPositionLimit
    hi = model.upperPositionLimit
    nq = model.nq

    for _ in range(max_retries):
        q_rand = np.zeros(nq)
        for j in range(nq):
            l = lo[j] if np.isfinite(lo[j]) else -math.pi
            h = hi[j] if np.isfinite(hi[j]) else math.pi
            q_rand[j] = random.uniform(l, h)
        r = solve_ik(model, data, end_frame_id, target, q_rand, params)
        if r.error < best.error:
            best = r
        if best.success:
            break

    q_seed[:] = best.q
    return best


# ─── 便捷函数 ──────────────────────────────────────────────────────────────────

def compute_ik(
    q_init: np.ndarray | None,
    target_pos: np.ndarray,
    target_rot: np.ndarray | None = None,
    *,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
    params: IKSolverParams | None = None,
    position_only: bool | None = None,
) -> IKResult:
    """使用默认模型计算 IK（便捷函数）。

    参数:
        q_init:      初始关节配置。传入 ``None`` 则自动使用零位构型。
        target_pos:  目标位置 (3,)，单位：米。
        target_rot:  目标旋转矩阵 (3, 3)，可选。
        roll:        ZYX 欧拉角之 roll，仅当 rot=None 时使用。
        pitch:       ZYX 欧拉角之 pitch。
        yaw:         ZYX 欧拉角之 yaw。
        params:      IK 参数。
        position_only: 是否只求位置。默认自动判断：未提供旋转矩阵且 RPY
                       全为零时按仅位置处理；如需显式约束单位姿态，请传入
                       ``target_rot=np.eye(3)`` 或 ``position_only=False``。

    返回:
        IKResult。
    """
    from .robot_model import load_robot_model, get_end_effector_frame_id

    model = load_robot_model()
    data = model.createData()
    frame_id = get_end_effector_frame_id(model)
    target = pos_rot_to_se3(target_pos, target_rot, roll, pitch, yaw)

    if position_only is None:
        position_only = (
            target_rot is None
            and roll == 0.0
            and pitch == 0.0
            and yaw == 0.0
        )

    if q_init is None:
        q_init = pin.neutral(model)

    return solve_ik(
        model, data, frame_id, target, q_init, params,
        position_only=position_only,
    )
