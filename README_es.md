# Guía de inicio de Pinocchio y MeshCat para el reBot Arm B601-DM

<p align="center">
    <a href="./LICENSE">
        <img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="Licencia: MIT">
    </a>
    <img src="https://img.shields.io/badge/Python-3.10+-blue.svg" alt="Versión de Python">
    <img src="https://img.shields.io/badge/Platform-Linux%20%7C%20Ubuntu-orange.svg" alt="Plataforma">
    <img src="https://img.shields.io/badge/Framework-Pinocchio-yellow.svg" alt="Pinocchio">
</p>

<p align="center">
  <strong>Brazo robótico de 6 DoF · Compatibilidad multimotor · Solucionador cinemático · Planificación de trayectorias · Totalmente de código abierto</strong>
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

## 📖 Introducción

**reBotArm Control** es una biblioteca de control en Python para el brazo robótico reBot Arm B601, que proporciona una solución completa desde el control de motores de bajo nivel hasta el cálculo cinemático de alto nivel.

### ✨ Características principales

- 🦾 **Compatibilidad con dos modelos** — B601-DM (motores Damiao) y B601-RS (motores RobStride)
- 🧮 **Solucionador cinemático** — Cinemática directa e inversa basada en Pinocchio
- 🛤️ **Planificación de trayectorias** — Trayectoria geodésica en SE(3) + seguimiento CLIK
- 🔧 **Configuración flexible** — Fichero de configuración YAML para adaptar el hardware rápidamente

---

## ⚙️ Inicio rápido

### Requisitos

| Elemento | Requisito |
|----------|-----------|
| **Python** | 3.10+ |
| **Sistema operativo** | Ubuntu 22.04+ |
| **Interfaz de comunicación** | Puente serie USB2CAN o interfaz CAN |

### Pasos de instalación

#### Paso 1. Instalar uv (si no está instalado)

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

#### Paso 2. Sincronizar el entorno (instalar todas las dependencias)

```bash
git clone https://github.com/Seeed-Projects/reBotArm_control_py.git
cd reBotArm_control_py
uv sync
```

:::tip
`uv sync` creará automáticamente un entorno virtual (si no existe) e instalará todas las dependencias según `pyproject.toml` y `uv.lock`.
:::

---

## 🔌 Configuración del hardware

### Predeterminado: puente serie USB2CAN de Damiao

El reBot Arm B601-DM utiliza por defecto el módulo de puente serie USB2CAN de Damiao.

**Conexión del hardware**:
1. Conecta el módulo USB2CAN a tu ordenador mediante un cable USB
2. El sistema lo reconocerá automáticamente como el dispositivo `/dev/ttyACM0`

**Verificación de la configuración**:
```bash
# Comprobar el dispositivo
ls /dev/ttyACM0

# Escanear los motores
motorbridge-cli scan --vendor damiao --transport dm-serial \
    --serial-port /dev/ttyACM0 --serial-baud 921600
```

### Opcional: interfaz CAN estándar

Si usas otros adaptadores USB-CAN (CANable, PCAN, etc.):

```bash
# Activar la interfaz CAN
sudo ip link set can0 up type can bitrate 500000

# Verificar la interfaz
ip -details link show can0
```

### Configuración según la marca del motor

| Marca del motor | Transmisión | Configuración | Velocidad en baudios |
|-----------------|-------------|---------------|----------------------|
| **Damiao** | Puente serie | `dm-serial` | 921600 |
| **Damiao** | Interfaz CAN | `socketcan` | 500000 |
| **RobStride** | Interfaz CAN | `socketcan` | 500000 |

:::tip
- Con motores Damiao mediante puente serie es obligatorio establecer `--transport dm-serial`
- Regla del ID de realimentación (feedback): `feedback_id = motor_id + 0x10`
:::

---

## 📁 Estructura del proyecto

```
reBotArm_control_py/
├── config/                     # Ficheros de configuración
│   └── robot.yaml              # Configuración de parámetros de las articulaciones
├── example/                    # Programas de ejemplo
│   ├── Debug Tools/            # Herramientas de depuración
│   │   ├── 1_damiao_text.py        # Consola de motor único
│   │   └── 2_zero_and_read.py      # Calibración del cero
│   ├── Kinematics Tests/       # Pruebas de cinemática
│   │   ├── 5_fk_test.py            # Cinemática directa
│   │   └── 6_ik_test.py            # Cinemática inversa
│   ├── Real Machine Control/   # Control del robot real
│   │   ├── 7_arm_ik_control.py     # Control IK en tiempo real
│   │   ├── 8_arm_traj_control.py   # Planificación de trayectorias
│   │   └── 9_gravity_compensation.py  # Compensación de gravedad
│   └── sim/                    # Herramientas de simulación
├── reBotArm_control_py/        # Biblioteca principal
│   ├── actuator/               # Módulo de actuadores
│   ├── kinematics/             # Módulo de cinemática
│   ├── controllers/            # Módulo de controladores
│   └── trajectory/             # Módulo de planificación de trayectorias
├── urdf/                       # Modelo URDF
└── README.md
```

---

## 📚 Documentación Wiki

> **📖 Para el uso completo de cada demo, consulta la wiki oficial:**
>
> 👉 **Versión DM (motores Damiao)**: [https://wiki.seeedstudio.com/es/rebot_arm_b601_dm_pinocchio_meshcat/](https://wiki.seeedstudio.com/es/rebot_arm_b601_dm_pinocchio_meshcat/)
>
> 👉 **Versión RS (motores RobStride)**: [https://wiki.seeedstudio.com/es/rebot_arm_b601_rs_pinocchio_meshcat/](https://wiki.seeedstudio.com/es/rebot_arm_b601_rs_pinocchio_meshcat/)

### Scripts de ejemplo (solo visión general)

La carpeta `example/` incluye los siguientes scripts. Los formatos de entrada, los comandos interactivos, el comportamiento esperado, las notas de seguridad y el ajuste de parámetros **ya no se mantienen en este README**: consúltalos en la wiki.

- `0x01rs06_test.py` / `1_damiao_test.py` — Consola de un solo motor (específico del fabricante)
- `2_zero_and_read.py` — Calibración de cero y monitor de ángulos
- `3_mit_control.py` — Control de articulaciones en modo MIT
- `4_pos_vel_control.py` — Control de articulaciones en modo POS_VEL
- `5_fk_test.py` / `6_ik_test.py` — Pruebas de cinemática directa / inversa
- `7_arm_ik_control.py` / `8_arm_traj_control.py` — Control IK en tiempo real / control por trayectoria
- `9_gravity_compensation.py` / `10_gravity_compensation_lock.py` — Compensación de gravedad
- `sim/fk_sim.py` / `sim/ik_sim.py` / `sim/traj_sim.py` — Utilidades de simulación (sin hardware)

> **¿Por qué las instrucciones detalladas ya no están en este README?**
>
> Para mantener **una única fuente de verdad** y garantizar la coherencia del tutorial, todos los detalles de uso específicos de cada demo (formatos de entrada, comandos interactivos, comportamiento esperado, notas de seguridad, ajuste de parámetros) se han trasladado a la wiki. Si hay divergencias entre ambos, la wiki gana. Por favor, abre issues / PRs en el repositorio de la wiki cuando el contenido necesite actualizarse.

---

## 📄 Licencia

Este proyecto es de código abierto bajo la **licencia MIT**.

---

## ☎ Contacto

- **Soporte técnico**: [Abrir una incidencia (issue)](https://github.com/Seeed-Projects/reBotArm_control_py/issues)
- **Repositorio**: [GitHub](https://github.com/Seeed-Projects/reBotArm_control_py)

---

<p align="center">
  <strong>🌟 ¡Si este proyecto te resulta útil, danos una estrella (Star)!</strong>
</p>
