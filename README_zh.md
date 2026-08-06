# reBot Arm B601-DM 的 Pinocchio 与 MeshCat 入门指南

<p align="center">
    <a href="./LICENSE">
        <img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="License: MIT">
    </a>
    <img src="https://img.shields.io/badge/Python-3.10+-blue.svg" alt="Python Version">
    <img src="https://img.shields.io/badge/Platform-Linux%20%7C%20Ubuntu-orange.svg" alt="Platform">
    <img src="https://img.shields.io/badge/Framework-Pinocchio-yellow.svg" alt="Pinocchio">
</p>

<p align="center">
  <strong>6 自由度机械臂 · 多电机支持 · 运动学求解 · 轨迹规划 · 完全开源</strong>
</p>

---

## 📖 项目简介

**reBotArm Control** 是一个面向 reBot Arm B601 系列机械臂的 Python 控制库，提供从底层电机控制到上层运动学解算的完整解决方案。

### ✨ 核心特性

- 🦾 **双型号支持** — B601-DM（达妙电机）和 B601-RS（灵足电机）两款机械臂
- 🧮 **运动学求解** — 基于 Pinocchio 的正/逆运动学计算
- 🛤️ **轨迹规划** — SE(3) 测地线轨迹 + CLIK 跟踪
- 🔧 **灵活配置** — YAML 配置文件，快速适配不同硬件

---

## ⚙️ 快速开始

### 环境要求

| 项目 | 要求 |
|------|------|
| **Python** | 3.10+ |
| **操作系统** | Ubuntu 22.04+ |
| **通信接口** | USB2CAN 串口桥 或 CAN 接口 |

### 安装步骤

#### 步骤 1. 安装 uv（如未安装）

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

#### 步骤 2. 同步环境（安装所有依赖）

```bash
git clone https://github.com/Seeed-Projects/reBotArm_control_py.git
cd reBotArm_control_py
uv sync
```

:::tip
`uv sync` 会自动创建虚拟环境（如不存在）并根据 `pyproject.toml` 和 `uv.lock` 安装所有依赖。
:::

---

## 🔌 硬件配置

### 默认配置：达妙 USB2CAN 串口桥

reBot Arm B601-DM 默认使用达妙 USB2CAN 串口桥模块。

**硬件连接**：
1. 将 USB2CAN 模块通过 USB 线连接到计算机
2. 系统会自动识别为 `/dev/ttyACM0` 设备

**配置验证**：
```bash
# 检查设备
ls /dev/ttyACM0

# 扫描电机
motorbridge-cli scan --vendor damiao --transport dm-serial \
    --serial-port /dev/ttyACM0 --serial-baud 921600
```

### 可选配置：标准 CAN 接口

使用其他 USB-CAN 适配器（CANable、PCAN 等）：

```bash
# 启动 CAN 接口
sudo ip link set can0 up type can bitrate 500000

# 验证接口
ip -details link show can0
```

### 电机品牌配置

| 电机品牌 | 传输方式 | 配置参数 | 波特率 |
|----------|---------|---------|--------|
| **达妙 (Damiao)** | 串口桥 | `dm-serial` | 921600 |
| **达妙 (Damiao)** | CAN 接口 | `socketcan` | 500000 |
| **RobStride** | CAN 接口 | `socketcan` | 500000 |

:::tip
- 达妙电机使用串口桥时，必须设置 `--transport dm-serial`
- 反馈 ID 规则：`feedback_id = motor_id + 0x10`
:::

---

## 📁 项目结构

```
reBotArm_control_py/
├── config/                     # 配置文件
│   └── robot.yaml              # 关节参数配置
├── example/                    # 示例程序
│   ├── 调试工具/
│   │   ├── 1_damiao_text.py        # 单电机控制台
│   │   └── 2_zero_and_read.py      # 零点校准
│   ├── 位置控制/
│   │   ├── 3_mit_control.py        # MIT 控制
│   │   └── 4_pos_vel_control.py    # POS_VEL 控制
│   ├── 运动学测试/
│   │   ├── 5_fk_test.py            # 正运动学
│   │   └── 6_ik_test.py            # 逆运动学
│   ├── 实机控制/
│   │   ├── 7_arm_ik_control.py     # IK 实时控制
│   │   ├── 8_arm_traj_control.py   # 轨迹规划
│   │   └── 9_gravity_compensation.py  # 重力补偿
│   └── sim/                    # 仿真工具
├── reBotArm_control_py/        # 核心库
│   ├── actuator/               # 执行器模块
│   ├── kinematics/             # 运动学模块
│   ├── controllers/            # 控制器模块
│   └── trajectory/             # 轨迹规划模块
├── urdf/                       # URDF 模型
└── README.md
```

---

## 📚 Wiki 文档

> **📖 各 demo 的完整使用方法，请参考官方 Wiki：**
>
> 👉 **DM 版本（达妙电机）**: [https://wiki.seeedstudio.com/cn/rebot_arm_b601_dm_pinocchio_meshcat/](https://wiki.seeedstudio.com/cn/rebot_arm_b601_dm_pinocchio_meshcat/)
>
> 👉 **RS 版本（灵足电机）**: [https://wiki.seeedstudio.com/cn/rebot_arm_b601_rs_pinocchio_meshcat/](https://wiki.seeedstudio.com/cn/rebot_arm_b601_rs_pinocchio_meshcat/)

### 示例脚本（仅列出文件名）

`example/` 目录下提供以下脚本。关于输入格式、交互命令、预期行为、安全提示、参数调节等细节，**本 README 不再维护**，请以 Wiki 为准。

- `0x01rs06_test.py` / `1_damiao_test.py` — 单电机控制台（按电机厂商区分）
- `2_zero_and_read.py` — 零点校准与角度监控
- `3_mit_control.py` — MIT 模式关节控制
- `4_pos_vel_control.py` — POS_VEL 模式关节控制
- `5_fk_test.py` / `6_ik_test.py` — 正/逆运动学测试
- `7_arm_ik_control.py` / `8_arm_traj_control.py` — IK 实时控制 / 轨迹控制
- `9_gravity_compensation.py` / `10_gravity_compensation_lock.py` — 重力补偿
- `sim/fk_sim.py` / `sim/ik_sim.py` / `sim/traj_sim.py` — 仿真工具（无需硬件）

> **为什么本 README 不再保留具体使用细节？**
>
> 为了保证**单一信息源**与教程一致性，所有 demo 的具体使用方法（输入格式、交互命令、预期行为、安全提示、参数调节）已统一迁移到 Wiki。如果两边出现不一致，以 Wiki 为准。如需更新内容，请在 Wiki 仓库提 Issue / PR。

---

## 📄 License

本项目采用 **MIT 许可证** 开源。

---

## ☎ 联系我们

- **技术支持**: [提交 Issue](https://github.com/Seeed-Projects/reBotArm_control_py/issues)
- **项目仓库**: [GitHub](https://github.com/Seeed-Projects/reBotArm_control_py)

---

<p align="center">
  <strong>🌟 如果本项目对你有帮助，请给个 Star 支持一下！</strong>
</p>
