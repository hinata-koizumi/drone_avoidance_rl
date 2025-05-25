# Drone Avoidance RL Stack

[![CI](https://github.com/yourname/drone_avoidance_rl/actions/workflows/ci.yml/badge.svg)](https://github.com/yourname/drone_avoidance_rl/actions)  
📘 [日本語版はこちら](README.ja.md)

---

## Overview

- **PX4 SITL + ROS 2 Humble + Gazebo Fortress (LTS) + RL (Gym API) Unified Stack**
- **Reproducibility**: Ubuntu 22.04, ROS 2 Humble, PX4 v1.15, Gazebo Fortress (LTS), ros_gz, multi-stage Docker
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
git clone https://github.com/yourname/drone_avoidance_rl.git
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

---

## Testing & CI

- **Full CI/CD**: GitHub Actions for build, test, static analysis, E2E, coverage, security scan, release notes
- **Local test example**:
  ```bash
  tools/clean_workspace.sh
  docker build -t drone_rl:unified -f docker/Dockerfile.unified .
  docker run --rm drone_rl:unified ros2 launch sim_launch sim_all.launch.py
  python3 -m pip install pytest gymnasium numpy pyyaml lark
  cd src && PYTHONPATH=$PYTHONPATH:$(pwd) pytest ../tests/test_gym_api.py
  python3 -m pip install ruff mypy
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
   - Python 3.10系を推奨（pyenvで合わせる）
   - 依存インストール:
     ```sh
     python3 -m pip install --upgrade pip
     python3 -m pip install pytest gymnasium numpy pyyaml lark ruff mypy
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
