# reBot Arm B601-DM Pinocchio & MeshCat Getting Started Guide

<p align="center">
    <a href="./LICENSE">
        <img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="License: MIT">
    </a>
    <img src="https://img.shields.io/badge/Python-3.10+-blue.svg" alt="Python Version">
    <img src="https://img.shields.io/badge/Platform-Linux%20%7C%20Ubuntu-orange.svg" alt="Platform">
    <img src="https://img.shields.io/badge/Framework-Pinocchio-yellow.svg" alt="Pinocchio">
</p>

<p align="center">
  <strong>6-DOF Robotic Arm · Multi-Motor Support · Kinematics Solver · Trajectory Planning · Fully Open Source</strong>
</p>

<p align="center">
  <strong>
    <a href="./README_zh.md">简体中文</a> &nbsp;|&nbsp;
    <a href="./README.md">English</a> &nbsp;|&nbsp;
    <a href="./README_JP.md">日本語</a>&nbsp;|&nbsp;
    <a href="./README_Fr.md">français</a>&nbsp;|&nbsp;
    <a href="./README_es.md">Español</a>
  </strong>
</p>

---

## 📖 Introduction

**reBotArm Control** is a Python control library for the reBot Arm B601 robotic arm, providing a complete solution from low-level motor control to high-level kinematics computation.

### ✨ Core Features

- 🦾 **Dual Model Support** — B601-DM (Damiao motors) and B601-RS (RobStride motors)
- 🧮 **Kinematics Solver** — Forward/Inverse kinematics based on Pinocchio
- 🛤️ **Trajectory Planning** — SE(3) geodesic trajectory + CLIK tracking
- 🔧 **Flexible Configuration** — YAML configuration file for quick hardware adaptation

---

## ⚙️ Quick Start

### Requirements

| Item | Requirement |
|------|-------------|
| **Python** | 3.10+ |
| **Operating System** | Ubuntu 22.04+ |
| **Communication Interface** | USB2CAN Serial Bridge or CAN Interface |

### Installation Steps

#### Step 1. Install uv (if not installed)

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

#### Step 2. Sync Environment (Install All Dependencies)

```bash
git clone https://github.com/Seeed-Projects/reBotArm_control_py.git
cd reBotArm_control_py
uv sync
```

:::tip
`uv sync` will automatically create a virtual environment (if it doesn't exist) and install all dependencies according to `pyproject.toml` and `uv.lock`.
:::

---

## 🔌 Hardware Configuration

### Default: Damiao USB2CAN Serial Bridge

reBot Arm B601-DM uses the Damiao USB2CAN serial bridge module by default.

**Hardware Connection**:
1. Connect the USB2CAN module to your computer via USB cable
2. The system will automatically recognize it as `/dev/ttyACM0` device

**Configuration Verification**:
```bash
# Check device
ls /dev/ttyACM0

# Scan motors
motorbridge-cli scan --vendor damiao --transport dm-serial \
    --serial-port /dev/ttyACM0 --serial-baud 921600
```

### Optional: Standard CAN Interface

Using other USB-CAN adapters (CANable, PCAN, etc.):

```bash
# Start CAN interface
sudo ip link set can0 up type can bitrate 500000

# Verify interface
ip -details link show can0
```

### Motor Brand Configuration

| Motor Brand | Transmission | Configuration | Baud Rate |
|-------------|--------------|---------------|-----------|
| **Damiao** | Serial Bridge | `dm-serial` | 921600 |
| **Damiao** | CAN Interface | `socketcan` | 500000 |
| **RobStride** | CAN Interface | `socketcan` | 500000 |

:::tip
- For Damiao motors using serial bridge, must set `--transport dm-serial`
- Feedback ID rule: `feedback_id = motor_id + 0x10`
:::

---

## 📁 Project Structure

```
reBotArm_control_py/
├── config/                     # Configuration files
│   └── robot.yaml              # Joint parameter configuration
├── example/                    # Example programs
│   ├── Debug Tools/
│   │   ├── 1_damiao_text.py        # Single motor console
│   │   └── 2_zero_and_read.py      # Zero calibration
│   ├── Kinematics Tests/
│   │   ├── 5_fk_test.py            # Forward kinematics
│   │   └── 6_ik_test.py            # Inverse kinematics
│   ├── Real Machine Control/
│   │   ├── 7_arm_ik_control.py     # IK real-time control
│   │   ├── 8_arm_traj_control.py   # Trajectory planning
│   │   └── 9_gravity_compensation.py  # Gravity compensation
│   └── sim/                    # Simulation tools
├── reBotArm_control_py/        # Core library
│   ├── actuator/               # Actuator module
│   ├── kinematics/             # Kinematics module
│   ├── controllers/            # Controller module
│   └── trajectory/             # Trajectory planning module
├── urdf/                       # URDF model
└── README.md
```

---

## 📚 Wiki Documentation

> **📖 For complete usage of every demo, please refer to the official wiki:**
>
> 👉 **DM Version (Damiao Motors)**: [https://wiki.seeedstudio.com/rebot_arm_b601_dm_pinocchio_meshcat/](https://wiki.seeedstudio.com/rebot_arm_b601_dm_pinocchio_meshcat/)
>
> 👉 **RS Version (RobStride Motors)**: [https://wiki.seeedstudio.com/rebot_arm_b601_rs_pinocchio_meshcat/](https://wiki.seeedstudio.com/rebot_arm_b601_rs_pinocchio_meshcat/)

### Example Scripts (overview only)

The `example/` folder ships the following scripts. For input formats, interactive commands, expected behavior, safety notes, and parameter tuning, please consult the wiki — those details are **no longer maintained in this README**.

- `0x01rs06_test.py` / `1_damiao_test.py` — Single motor console (vendor-specific)
- `2_zero_and_read.py` — Zero calibration & angle monitoring
- `3_mit_control.py` — MIT-mode joint control
- `4_pos_vel_control.py` — POS_VEL-mode joint control
- `5_fk_test.py` / `6_ik_test.py` — Forward / inverse kinematics tests
- `7_arm_ik_control.py` / `8_arm_traj_control.py` — IK real-time / trajectory control
- `9_gravity_compensation.py` / `10_gravity_compensation_lock.py` — Gravity compensation
- `sim/fk_sim.py` / `sim/ik_sim.py` / `sim/traj_sim.py` — Simulation utilities (no hardware required)

> **Why are the detailed usage instructions no longer in this README?**
>
> To keep **a single source of truth** and guarantee tutorial consistency, all demo-specific usage (input formats, interactive commands, expected behavior, safety notes, parameter tuning) has been moved to the wiki. If the two ever drift, the wiki wins. Please open issues / PRs against the wiki when content needs updating.

---

## 📄 License

This project is open source under the **MIT License**.

---

## ☎ Contact Us

- **Technical Support**: [Submit Issue](https://github.com/Seeed-Projects/reBotArm_control_py/issues)
- **Repository**: [GitHub](https://github.com/Seeed-Projects/reBotArm_control_py)

---

<p align="center">
  <strong>🌟 If this project helps you, please give us a Star!</strong>
</p>
