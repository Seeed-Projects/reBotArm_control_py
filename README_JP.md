# reBot Arm B601-DM の Pinocchio と MeshCat 入門ガイド

<p align="center">
    <a href="./LICENSE">
        <img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="License: MIT">
    </a>
    <img src="https://img.shields.io/badge/Python-3.10+-blue.svg" alt="Python Version">
    <img src="https://img.shields.io/badge/Platform-Linux%20%7C%20Ubuntu-orange.svg" alt="Platform">
    <img src="https://img.shields.io/badge/Framework-Pinocchio-yellow.svg" alt="Pinocchio">
</p>

<p align="center">
  <strong>6 自由度ロボットアーム · 多モーター対応 · 運動学ソルバー · 軌道計画 · 完全オープンソース</strong>
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

## 📖 プロジェクト概要

**reBotArm Control** は、reBot Arm B601 ロボットアーム向けの Python 制御ライブラリで、低レベルのモーター制御から高レベルの運動学計算までの完全なソリューションを提供します。

### ✨ 主な機能

- 🦾 **双型号サポート** — B601-DM（達妙モーター）と B601-RS（霊足モーター）
- 🧮 **運動学ソルバー** — Pinocchio ベースの順/逆運動学計算
- 🛤️ **軌道計画** — SE(3) 測地線軌道 + CLIK 追従
- 🔧 **柔軟な設定** — YAML 設定ファイルでハードウェアの迅速な適応

---

## ⚙️ クイックスタート

### 動作環境

| 項目 | 要件 |
|------|------|
| **Python** | 3.10+ |
| **オペレーティングシステム** | Ubuntu 22.04+ |
| **通信インターフェース** | USB2CAN シリアルブリッジ または CAN インターフェース |

### インストール手順

#### ステップ 1. uv のインストール（未インストールの場合）

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

#### ステップ 2. 環境の同期（すべての依存関係をインストール）

```bash
git clone https://github.com/Seeed-Projects/reBotArm_control_py.git
cd reBotArm_control_py
uv sync
```

:::tip
`uv sync` は、仮想環境を自動的に作成し（存在しない場合）、`pyproject.toml` と `uv.lock` に従ってすべての依存関係をインストールします。
:::

---

## 🔌 ハードウェア設定

### デフォルト：達妙 USB2CAN シリアルブリッジ

reBot Arm B601-DM はデフォルトで達妙 USB2CAN シリアルブリッジモジュールを使用します。

**ハードウェア接続**：
1. USB2CAN モジュールを USB ケーブルでコンピュータに接続
2. システムが自動的に `/dev/ttyACM0` デバイスとして認識します

**設定の確認**：
```bash
# デバイスの確認
ls /dev/ttyACM0

# モータースキャン
motorbridge-cli scan --vendor damiao --transport dm-serial \
    --serial-port /dev/ttyACM0 --serial-baud 921600
```

### オプション：標準 CAN インターフェース

他の USB-CAN アダプター（CANable、PCAN など）を使用する場合：

```bash
# CAN インターフェースの起動
sudo ip link set can0 up type can bitrate 500000

# インターフェースの確認
ip -details link show can0
```

### モーターブランドの設定

| モーターブランド | 伝送方式 | 設定パラメータ | ボーレート |
|-----------------|---------|---------------|-----------|
| **達妙 (Damiao)** | シリアルブリッジ | `dm-serial` | 921600 |
| **達妙 (Damiao)** | CAN インターフェース | `socketcan` | 500000 |
| **RobStride** | CAN インターフェース | `socketcan` | 500000 |

:::tip
- 達妙モーターでシリアルブリッジを使用する場合、`--transport dm-serial` の設定が必要です
- フィードバック ID ルール：`feedback_id = motor_id + 0x10`
:::

---

## 📁 プロジェクト構成

```
reBotArm_control_py/
├── config/                     # 設定ファイル
│   └── robot.yaml              # 関節パラメータ設定
├── example/                    # サンプルプログラム
│   ├── デバッグツール/
│   │   ├── 1_damiao_text.py        # 単一モーターコンソール
│   │   └── 2_zero_and_read.py      # ゼロキャリブレーション
│   ├── 運動学テスト/
│   │   ├── 5_fk_test.py            # 順運動学
│   │   └── 6_ik_test.py            # 逆運動学
│   ├── 実機制御/
│   │   ├── 7_arm_ik_control.py     # IK リアルタイム制御
│   │   ├── 8_arm_traj_control.py   # 軌道計画
│   │   └── 9_gravity_compensation.py  # 重力補償
│   └── sim/                    # シミュレーションツール
├── reBotArm_control_py/        # コアライブラリ
│   ├── actuator/               # アクチュエータモジュール
│   ├── kinematics/             # 運動学モジュール
│   ├── controllers/            # コントローラモジュール
│   └── trajectory/             # 軌道計画モジュール
├── urdf/                       # URDF モデル
└── README.md
```

---

## 📚 Wiki ドキュメント

> **📖 各デモの完全な使い方は、公式 Wiki を参照してください：**
>
> 👉 **DM バージョン（達妙モーター）**: [https://wiki.seeedstudio.com/ja/rebot_arm_b601_dm_pinocchio_meshcat/](https://wiki.seeedstudio.com/ja/rebot_arm_b601_dm_pinocchio_meshcat/)
>
> 👉 **RS バージョン（霊足モーター）**: [https://wiki.seeedstudio.com/ja/rebot_arm_b601_rs_pinocchio_meshcat/](https://wiki.seeedstudio.com/ja/rebot_arm_b601_rs_pinocchio_meshcat/)

### サンプルスクリプト（概要のみ）

`example/` フォルダには以下のスクリプトが含まれています。入力形式・対話コマンド・期待動作・安全上の注意・パラメータ調整などの詳細は **この README では管理しません**。Wiki を参照してください。

- `0x01rs06_test.py` / `1_damiao_test.py` — 単モーターコンソール（ベンダー別）
- `2_zero_and_read.py` — ゼロ点キャリブレーションと角度モニタリング
- `3_mit_control.py` — MIT モード関節制御
- `4_pos_vel_control.py` — POS_VEL モード関節制御
- `5_fk_test.py` / `6_ik_test.py` — 順/逆運動学テスト
- `7_arm_ik_control.py` / `8_arm_traj_control.py` — IK リアルタイム / 軌道制御
- `9_gravity_compensation.py` / `10_gravity_compensation_lock.py` — 重力補償
- `sim/fk_sim.py` / `sim/ik_sim.py` / `sim/traj_sim.py` — シミュレーションツール（ハードウェア不要）

> **なぜ詳細な使い方をこの README から削除したのですか？**
>
> **单一の真実の源 (single source of truth)** を保ち、チュートリアルの一貫性を保証するため、デモ固有の使い方（入力形式・対話コマンド・期待動作・安全上の注意・パラメータ調整）はすべて Wiki に集約しました。両者に食い違いが生じた場合は Wiki を優先してください。内容の更新は Wiki リポジトリへの Issue / PR でお願いします。

---

## 📄 ライセンス

本プロジェクトは **MIT ライセンス** の下でオープンソースです。

---

## ☎ お問い合わせ

- **技術サポート**: [Issue を提出](https://github.com/Seeed-Projects/reBotArm_control_py/issues)
- **リポジトリ**: [GitHub](https://github.com/Seeed-Projects/reBotArm_control_py)

---

<p align="center">
  <strong>🌟 このプロジェクトが役に立った場合は、Star をつけてサポートしてください！</strong>
</p>
