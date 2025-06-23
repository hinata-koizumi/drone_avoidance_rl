# Drone Avoidance RL Stack

[![CI](https://github.com/hinata-koizumi/drone_avoidance_rl/actions/workflows/ci.yml/badge.svg)](https://github.com/hinata-koizumi/drone_avoidance_rl/actions)  
📘 [日本語版はこちら](README.ja.md)

---

## Overview

- **PX4 SITL + ROS 2 Humble + Gazebo Fortress (LTS) + RL (Gym API) Unified Stack**
- **Reproducibility**: Ubuntu 22.04, ROS 2 Humble, PX4 v1.15, Gazebo Garden (LTS), ros_gz 0.244.0, multi-stage Docker
- **CI/CD Automation**: GitHub Actions for build, test, static analysis, E2E, coverage, security, release notes
- **Custom model/airframe support**
- **Type safety, code quality gates, auto-generated docs**

---

## Directory Layout

```
drone_avoidance_rl/
├── docker/          # Dockerfiles & entrypoints
├── src/             # ROS 2 nodes, Gym env, custom msgs
├── custom_model/    # Replaceable SDF model
├── custom_airframes/# PX4 airframe JSON
├── tests/           # Integration/E2E tests (pytest)
├── docs/            # mkdocs/Sphinx auto-docs
├── .github/         # CI/CD, PR templates, etc.
└── tools/           # Dev helper scripts
```

---

## Common Infrastructure & Utilities

- **BridgeBase base class**: Located in `src/common/bridge_base.py`, this class unifies QoS settings, parameter handling, and logging for all bridge nodes (e.g., angle_bridge, outer_motor_bridge, command_bridge, state_bridge). To implement a new bridge node, simply inherit from BridgeBase and provide a parameter dict.
- **Common utilities**: Generic helpers like `clamp` are collected in `src/common/utils.py` and can be imported anywhere.
- **Benefits**: Reduces code duplication, improves maintainability, and makes it easy to add new bridge nodes.

---

## Prerequisites

- Docker Desktop 4.30+ (BuildKit enabled)
- 12GB+ RAM
- macOS 12+, Linux, Windows (WSL2)
- Apple Silicon (arm64) / x86_64 supported
- (Optional) Apple M-series or NVIDIA CUDA 12

---

## Quick Start

```bash
# 1. Clone & submodules
git clone https://github.com/hinata-koizumi/drone_avoidance_rl.git
cd drone_avoidance_rl
git submodule update --init --recursive

# 2. (Optional) Insert your own drone model & airframe
cp -r ~/my_drone_sdf      custom_model/drone_model
cp    ~/4500_my_drone.json custom_airframes/

# 3. Build & launch (CPU)
docker compose --profile cpu up -d --build

# 4. (Optional) Apple GPU
docker compose --profile gpu up -d --build

# 5. Stop
docker compose down
```

---

## Python環境セットアップ（ローカル開発・テスト用）

このリポジトリのPython依存パッケージは requirements.txt で一元管理されています。

```bash
# 依存パッケージのインストール
python3 -m pip install --upgrade pip
pip install -r requirements.txt
```

- 依存パッケージ: numpy, gymnasium, torch, tensorboard, pytest, mypy, ruff, など
- テスト実行例:
  ```bash
  export PYTHONPATH=$(pwd):$(pwd)/src
  pytest tests/test_gym_api.py
  pytest tests/test_rl_longrun.py
  pytest tests/test_gym_env.py
  ```
- 静的解析:
  ```bash
  ruff src/ tests/
  mypy src/ tests/
  ```

---

## Customization

- **Reward weights**: `REWARD_ORI`, `REWARD_POS`, `REWARD_SMOOTH` env vars (see `src/gym_env.py`)
- **Domain randomization**: extend `DroneSimEnv._randomize_world()`
- **PX4 parameters**: edit JSON in `custom_airframes/`
- **Telemetry**: forward UDP 14550 to QGroundControl

## Version Management

- ROS 2やGazebo/Ignitionのバージョンは`.env`ファイルで一元管理します。
- 例:
  ```
  ROS_DISTRO=humble
  IGNITION_VERSION=fortress
  ```
- `.env.example` を `.env` にコピーして編集してください。
- Dockerfileやdocker-compose.ymlはこの値を参照してビルドされます。
- バージョンアップ時は`.env`の値を変更するだけで全体に反映されます。
- **package.xmlのバージョン一貫性は`check_package_versions.sh`で自動チェックされ、タグリリース時はCIでタグとpackage.xmlのバージョン一致も自動検証されます。**
- **依存性（requirements.txt, rosdep YAML, GitHub Actions workflow）はDependabot（`.github/dependabot.yml`）で自動監視・PR作成されます。**

---

## Testing & CI

- **Full CI/CD**: GitHub Actions for build, test, static analysis, E2E, coverage, security scan, release notes
- **Local test example**:
  ```bash
  tools/clean_workspace.sh
  docker compose up --build
  python3 -m pip install pytest gymnasium numpy pyyaml lark types-PyYAML
  cd src && PYTHONPATH=$PYTHONPATH:$(pwd) pytest ../tests/test_gym_api.py
  python3 -m pip install ruff mypy types-PyYAML
  ruff src/ tests/
  mypy src/ tests/
  ```

## ローカルでCI/CDと全く同じテストを走らせる手順

1. 必ずDockerとdocker composeを使ってテストしてください。
   ```sh
   docker compose build --no-cache
   bash tools/setup_rosdep_local.sh
   docker compose -f tests/ci-compose.yml up --abort-on-container-exit
   ```
   これによりCIと同じ環境・コマンドでテストできます。

2. Gym APIテストや静的解析もCI/CDと同じコマンドで実行してください。
   - Python 3.10.13系を推奨（pyenvで合わせる）
   - 依存インストール:
     ```sh
     python3 -m pip install --upgrade pip
     python3 -m pip install pytest gymnasium numpy pyyaml lark ruff mypy types-PyYAML
     ```
   - テスト:
     ```sh
     cd src
     PYTHONPATH=$PYTHONPATH:$(pwd) pytest ../tests/test_gym_api.py
     ruff src/ tests/
     mypy src/ tests/
     ```

3. rosdep依存解決は必ずlocal rosdep yamlを反映してください。
   ```sh
   bash tools/setup_rosdep_local.sh
   ```

4. PythonバージョンはCIと同じ3.10を推奨します。
   ```sh
   pyenv install 3.10.12
   pyenv local 3.10.12
   ```

---

## Documentation

- **Auto-generated docs**: [docs/](docs/) (mkdocs structure)
- **GitHub Pages auto-publish**
- Architecture, dev flow, FAQ, troubleshooting, etc.

---

## Contribution

- PR template & CONTRIBUTING.md required
- Code quality gates (ruff, mypy, ament_lint_auto) required
- Semantic Versioning
- See [docs/](docs/) for details

---

## License

Apache License 2.0 — see `LICENSE`.

---

*Contributions and issues are welcome!*

## Gym API仕様
- 観測空間: 15次元（姿勢, 位置, 速度, 角速度, 風）
- 行動空間: 4次元（2モーターのスロットル・角度）
- 報酬関数: REWARD_ORI, REWARD_POS, REWARD_SMOOTH で加重和
- カスタマイズ例: 
  - 環境変数で報酬重み変更
  - DroneSimEnv._randomize_world() を拡張してドメインランダム化

## 主要ROS 2ノード・トピック
- /drone{N}/inner_propeller_cmd (DroneControlCommand)
- /drone{N}/state (DroneState)
- launchファイルやノードパラメータでトピック名は変更可能

## ドキュメント生成・閲覧
mkdocs serve
# または
sphinx-build -b html docs/ docs/_build/html

---

## 強化学習アルゴリズムの利用例

本リポジトリは「環境（Env）」のみを提供しています。学習アルゴリズム本体はStable Baselines3やCleanRL等の外部ライブラリを利用してください。

### 例: Stable Baselines3でSACを使う
```python
from stable_baselines3 import SAC
from gym_env import DroneSimEnv

env = DroneSimEnv(reward_mode="hover", episode_max_steps=1000)
model = SAC("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=100_000)
```

### 例: ベクトル化環境で複数同時学習
```python
from stable_baselines3 import PPO
from gymnasium.vector import AsyncVectorEnv
from gym_env import DroneSimEnv

def make_env(i):
    def _init():
        return DroneSimEnv(instance_id=i, reward_mode="hover")
    return _init

env = AsyncVectorEnv([make_env(i) for i in range(4)])
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=200_000)
```

---

## パラメータの柔軟な指定方法

`DroneSimEnv`の主要パラメータ（報酬モード、エピソード長、目標高度など）は、
- Python引数
- ROS 2パラメータサーバ
- launchファイル
から柔軟に指定できます。

### 例: launchファイルやros2 paramで指定
```bash
ros2 run your_package your_node --ros-args \
  -p reward_mode:=hover \
  -p episode_max_steps:=1500 \
  -p target_alt:=2.5
```

---

## タスクごとの報酬モード
- `reward_mode="hover"` : ホバリング特化
- `reward_mode="path_follow"` : 経路追従
- `reward_mode="obstacle_avoid"` : 障害物回避
- `reward_mode="default"` : 従来型（環境変数で重み調整）

## Ignition Gazebo (Garden) のインストール

本プロジェクトはIgnition Gazebo Gardenを使用します。以下の手順でインストールしてください。

### Ubuntu 22.04 でのインストール例
```bash
sudo apt update
sudo apt install -y wget lsb-release gnupg2
wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update
sudo apt install -y gz-garden
```

### PATH/IGN_CONFIG_PATHの設定例
```bash
export PATH="/usr/bin:$PATH"
export IGN_CONFIG_PATH="/usr/share/gz/garden"
```

`ign`コマンドが使えることを確認してください:
```bash
ign --version
```

## Docker ベースイメージのビルド

本プロジェクトの各サービスは `drone-avoidance-base` イメージをベースにしています。CIやローカルでエラーが出る場合、以下のコマンドで事前にpullしてください。

```sh
docker pull ghcr.io/hinata-koizumi/drone-avoidance-base:2.0.1
```