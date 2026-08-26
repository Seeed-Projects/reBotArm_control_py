# ReBot Arm RS 机械描述包

本目录是可独立复制的 B601-RS 机械描述资源。RS/DM 总体差异及跨引擎复用说明见上一级 [`README.md`](../README.md)。

```text
RS/
├── urdf/
│   └── ReBot_Arm_RS.urdf
└── meshes/
    ├── visual/             # 仅用于渲染的 CNC、电机和 PLA 分件
    ├── shared/             # 同时用于渲染和 URDF 碰撞的整件网格
    └── mujoco_collision/   # MuJoCo 夹指精细碰撞网格
```

- `urdf/ReBot_Arm_RS.urdf` 是当前 RS 主模型，包含机械臂与夹爪定义。
- `meshes/visual/` 包含 22 个仅用于渲染和配色的 STL。
- `meshes/shared/` 包含 10 个同时用于渲染和碰撞的 STL；RS 当前没有只用于 URDF 碰撞的独立网格。
- `meshes/mujoco_collision/` 包含 10 个 MuJoCo 使用的夹指前、中、后段以及滑块、限位块碰撞网格。
- URDF 使用 `../meshes/...` 相对路径，不依赖 ROS package URI，复制整个 `RS/` 目录即可使用。
- ROS 2 运行时模型仍位于 `rebotarm_ros2_RS/src/rebotarm_bringup/description/`，两处模型更新后应同步核对。

例如，从仓库根目录加载：

```python
from pathlib import Path

urdf_path = Path("Rebot_Arm_description/RS/urdf/ReBot_Arm_RS.urdf")
```
