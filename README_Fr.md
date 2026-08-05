# Guide de Démarrage Pinocchio & MeshCat pour reBot Arm B601-DM

<p align="center">
    <a href="./LICENSE">
        <img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="License: MIT">
    </a>
    <img src="https://img.shields.io/badge/Python-3.10+-blue.svg" alt="Version Python">
    <img src="https://img.shields.io/badge/Platform-Linux%20%7C%20Ubuntu-orange.svg" alt="Plateforme">
    <img src="https://img.shields.io/badge/Framework-Pinocchio-yellow.svg" alt="Pinocchio">
</p>

<p align="center">
  <strong>Bras Robotique 6-DOF · Support Multi-Moteurs · Solveur Cinématique · Planification de Trajectoire · Entièrement Open Source</strong>
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

**reBotArm Control** est une bibliothèque de contrôle Python pour le bras robotique reBot Arm B601, fournissant une solution complète du contrôle des moteurs de bas niveau au calcul cinématique de haut niveau.

### ✨ Fonctionnalités Principales

- 🦾 **Double Modèle** — B601-DM (moteurs Damiao) et B601-RS (moteurs RobStride)
- 🧮 **Solveur Cinématique** — Cinématique directe/inverse basée sur Pinocchio
- 🛤️ **Planification de Trajectoire** — Trajectoire géodésique SE(3) + suivi CLIK
- 🔧 **Configuration Flexible** — Fichier de configuration YAML pour une adaptation rapide du matériel

---

## ⚙️ Démarrage Rapide

### Configuration Requise

| Élément | Configuration Requise |
|---------|----------------------|
| **Python** | 3.10+ |
| **Système d'Exploitation** | Ubuntu 22.04+ |
| **Interface de Communication** | Pont série USB2CAN ou Interface CAN |

### Étapes d'Installation

#### Étape 1. Installer uv (si non installé)

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

#### Étape 2. Synchroniser l'Environnement (Installer Toutes les Dépendances)

```bash
git clone https://github.com/Seeed-Projects/reBotArm_control_py.git
cd reBotArm_control_py
uv sync
```

:::tip
`uv sync` créera automatiquement un environnement virtuel (s'il n'existe pas) et installera toutes les dépendances selon `pyproject.toml` et `uv.lock`.
:::

---

## 🔌 Configuration Matérielle

### Par Défaut : Pont Série Damiao USB2CAN

Le reBot Arm B601-DM utilise par défaut le module de pont série Damiao USB2CAN.

**Connexion Matérielle** :
1. Connectez le module USB2CAN à votre ordinateur via un câble USB
2. Le système le reconnaîtra automatiquement comme périphérique `/dev/ttyACM0`

**Vérification de la Configuration** :
```bash
# Vérifier le périphérique
ls /dev/ttyACM0

# Scanner les moteurs
motorbridge-cli scan --vendor damiao --transport dm-serial \
    --serial-port /dev/ttyACM0 --serial-baud 921600
```

### Optionnel : Interface CAN Standard

Utilisation d'autres adaptateurs USB-CAN (CANable, PCAN, etc.) :

```bash
# Démarrer l'interface CAN
sudo ip link set can0 up type can bitrate 500000

# Vérifier l'interface
ip -details link show can0
```

### Configuration des Marques de Moteurs

| Marque de Moteur | Transmission | Configuration | Baud Rate |
|-----------------|--------------|---------------|-----------|
| **Damiao** | Pont Série | `dm-serial` | 921600 |
| **Damiao** | Interface CAN | `socketcan` | 500000 |
| **RobStride** | Interface CAN | `socketcan` | 500000 |

:::tip
- Pour les moteurs Damiao utilisant le pont série, devez définir `--transport dm-serial`
- Règle d'ID de feedback : `feedback_id = motor_id + 0x10`
:::

---

## 📁 Structure du Projet

```
reBotArm_control_py/
├── config/                     # Fichiers de configuration
│   └── robot.yaml              # Configuration des paramètres des articulations
├── example/                    # Programmes d'exemple
│   ├── Outils de Débogage/
│   │   ├── 1_damiao_text.py        # Console mono-moteur
│   │   └── 2_zero_and_read.py      # Calibration zéro
│   ├── Tests Cinématiques/
│   │   ├── 5_fk_test.py            # Cinématique directe
│   │   └── 6_ik_test.py            # Cinématique inverse
│   ├── Contrôle Réel/
│   │   ├── 7_arm_ik_control.py     # Contrôle IK temps réel
│   │   ├── 8_arm_traj_control.py   # Planification de trajectoire
│   │   └── 9_gravity_compensation.py  # Compensation de gravité
│   └── sim/                    # Outils de simulation
├── reBotArm_control_py/        # Bibliothèque principale
│   ├── actuator/               # Module d'actionneur
│   ├── kinematics/             # Module de cinématique
│   ├── controllers/            # Module de contrôleur
│   └── trajectory/             # Module de planification de trajectoire
├── urdf/                       # Modèle URDF
└── README.md
```

---

## 📚 Documentation Wiki

> **📖 Pour l'utilisation complète de chaque démo, veuillez consulter le wiki officiel :**
>
> 👉 **Version DM (moteurs Damiao)**: [https://wiki.seeedstudio.com/rebot_arm_b601_dm_pinocchio_meshcat/](https://wiki.seeedstudio.com/rebot_arm_b601_dm_pinocchio_meshcat/)
>
> 👉 **Version RS (moteurs RobStride)**: [https://wiki.seeedstudio.com/rebot_arm_b601_rs_pinocchio_meshcat/](https://wiki.seeedstudio.com/rebot_arm_b601_rs_pinocchio_meshcat/) *(disponible en anglais)*

### Scripts d'exemple (aperçu uniquement)

Le dossier `example/` contient les scripts suivants. Les formats d'entrée, les commandes interactives, le comportement attendu, les notes de sécurité et le réglage des paramètres **ne sont plus maintenus dans ce README** : veuillez consulter le wiki.

- `0x01rs06_test.py` / `1_damiao_test.py` — Console mono-moteur (spécifique au fournisseur)
- `2_zero_and_read.py` — Calibration du zéro et surveillance des angles
- `3_mit_control.py` — Contrôle d'articulations en mode MIT
- `4_pos_vel_control.py` — Contrôle d'articulations en mode POS_VEL
- `5_fk_test.py` / `6_ik_test.py` — Tests de cinématique directe / inverse
- `7_arm_ik_control.py` / `8_arm_traj_control.py` — Contrôle IK en temps réel / contrôle par trajectoire
- `9_gravity_compensation.py` / `10_gravity_compensation_lock.py` — Compensation de gravité
- `sim/fk_sim.py` / `sim/ik_sim.py` / `sim/traj_sim.py` — Utilitaires de simulation (sans matériel)

> **Pourquoi les instructions détaillées ne sont-elles plus dans ce README ?**
>
> Pour maintenir **une source unique de vérité** et garantir la cohérence du tutoriel, tous les détails d'utilisation spécifiques à chaque démo (formats d'entrée, commandes interactives, comportement attendu, notes de sécurité, réglage des paramètres) ont été déplacés dans le wiki. En cas de divergence entre les deux, le wiki prime. Veuillez ouvrir des issues / PRs sur le dépôt du wiki lorsqu'une mise à jour du contenu est nécessaire.

---

## 📄 Licence

Ce projet est open source sous la **Licence MIT**.

---

## ☎ Nous Contacter

- **Support Technique** : [Soumettre un Issue](https://github.com/Seeed-Projects/reBotArm_control_py/issues)
- **Dépôt** : [GitHub](https://github.com/Seeed-Projects/reBotArm_control_py)

---

<p align="center">
  <strong>🌟 Si ce projet vous est utile, veuillez nous donner une Star !</strong>
</p>
