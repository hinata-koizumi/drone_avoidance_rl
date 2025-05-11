# Drone Avoidance RL Stack

[![CI](https://github.com/yourname/drone_avoidance_rl/actions/workflows/ci.yml/badge.svg)](https://github.com/yourname/drone_avoidance_rl/actions)  
📘 [日本語版はこちら](README.ja.md)

---

## Overview

- **PX4 SITL + ROS 2 Humble + Gazebo Garden + RL (Gym API) 統合スタック**
- **完全再現性**: Ubuntu 22.04, ROS 2 Humble, PX4 v1.15, Gazebo Garden, multi-stage Docker
- **CI/CD自動化**: GitHub Actionsでビルド・テスト・静的解析・E2E・カバレッジ・セキュリティ・リリースノート自動生成
- **カスタムモデル/エアフレーム差し替え対応**
- **型安全・コード品質ゲート・ドキュメント自動生成**

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
├── .github/         # CI/CD, PRテンプレート等
└── tools/           # 開発補助スクリプト
```

---

## Prerequisites

- Docker Desktop 4.30+ (BuildKit有効)
- 12GB+ RAM
- macOS 12+, Linux, Windows (WSL2)
- Apple Silicon (arm64) / x86_64両対応
- (Optional) GPU: Apple Mシリーズ or NVIDIA CUDA 12

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

- **Reward weights**: `REWARD_ORI`, `REWARD_POS`, `REWARD_SMOOTH` 環境変数（`src/gym_env.py`参照）
- **Domain randomization**: `DroneSimEnv._randomize_world()`を拡張
- **PX4 parameters**: `custom_airframes/`のJSON編集
- **Telemetry**: UDP 14550をQGroundControl等に転送

---

## Testing & CI

- **全自動CI/CD**: GitHub Actionsで全ビルド・テスト・静的解析・E2E・カバレッジ・セキュリティスキャン・リリースノート自動生成
- **ローカルテスト例**:
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

---

## Documentation

- **自動生成ドキュメント**: [docs/](docs/) 配下にmkdocs構成
- **GitHub Pages自動公開対応**
- 開発フロー・運用ルール・FAQ・トラブルシューティング等も集約

---

## Contribution

- PRテンプレート・CONTRIBUTING.md必須
- コード品質ゲート（ruff, mypy, ament_lint_auto）必須
- Semantic Versioning運用
- 詳細は[docs/](docs/)参照

---

## License

Apache License 2.0 — see `LICENSE`.

---

*Contributions and issues are welcome!*
