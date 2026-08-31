# ReBot Arm DM 机械描述包

本目录是可独立复制的 B601-DM 机械描述资源。RS/DM 总体差异及跨引擎复用说明见上一级 [`README.md`](../README.md)。

```text
DM/
├── urdf/
│   └── ReBot_Arm_DM.urdf
└── meshes/
    ├── visual/             # 仅用于 URDF 渲染与配色的分件
    ├── collision/          # URDF 碰撞整件
    ├── shared/             # URDF 渲染与 MuJoCo 碰撞共用
    └── mujoco_collision/   # MuJoCo 夹指分段碰撞网格
```

- `urdf/ReBot_Arm_DM.urdf` 是当前 DM 主模型，包含机械臂与夹爪定义。
- `meshes/visual/` 包含 30 个仅用于 URDF 渲染和配色的分件。
- `meshes/collision/` 包含 10 个 URDF 碰撞整件。
- `meshes/shared/` 包含 4 个由 URDF 渲染和 MuJoCo 碰撞共用的夹指滑块、限位块网格。
- `meshes/mujoco_collision/` 包含 6 个 MuJoCo 使用的左右夹指前、中、后段碰撞网格。
- URDF 使用 `../meshes/...` 相对路径，不依赖 ROS package URI，复制整个 `DM/` 目录即可使用。
- ROS 2 运行时模型仍位于 `reBotArm_ros2_DM/src/rebotarm_bringup/description/`，两处模型更新后应同步核对。

例如，从仓库根目录加载：

```python
from pathlib import Path

urdf_path = Path("Rebot_Arm_description/DM/urdf/ReBot_Arm_DM.urdf")
```
