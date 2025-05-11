# ドローン回避RLスタック

[![CI](https://github.com/yourname/drone_avoidance_rl/actions/workflows/ci.yml/badge.svg)](https://github.com/yourname/drone_avoidance_rl/actions)  
📄 [English version here](README.md)

---

## 概要

- **PX4 SITL + ROS 2 Humble + Gazebo Garden + RL (Gym API) 統合スタック**
- **完全再現性**: Ubuntu 22.04, ROS 2 Humble, PX4 v1.15, Gazebo Garden, multi-stage Docker
- **CI/CD自動化**: GitHub Actionsでビルド・テスト・静的解析・E2E・カバレッジ・セキュリティ・リリースノート自動生成
- **カスタムモデル/エアフレーム差し替え対応**
- **型安全・コード品質ゲート・ドキュメント自動生成**

---

## ディレクトリ構成

```
drone_avoidance_rl/
├── docker/          # Dockerfile・エントリポイント
├── src/             # ROS 2ノード・Gym環境・カスタムmsg
├── custom_model/    # 差し替え可能なSDFモデル
├── custom_airframes/# PX4エアフレームJSON
├── tests/           # 統合/E2Eテスト（pytest）
├── docs/            # mkdocs/Sphinx自動ドキュメント
├── .github/         # CI/CD・PRテンプレート等
└── tools/           # 開発補助スクリプト
```

---

## 必要環境

- Docker Desktop 4.30+（BuildKit有効）
- 12GB以上のRAM
- macOS 12+、Linux、Windows（WSL2）
- Apple Silicon（arm64）/ x86_64両対応
- （任意）Apple Mシリーズ or NVIDIA CUDA 12

---

## クイックスタート

```bash
# 1. クローン＆サブモジュール
git clone https://github.com/yourname/drone_avoidance_rl.git
cd drone_avoidance_rl
git submodule update --init --recursive

# 2. （任意）独自ドローンモデル・エアフレームを追加
cp -r ~/my_drone_sdf      custom_model/drone_model
cp    ~/4500_my_drone.json custom_airframes/

# 3. ビルド＆起動（CPU）
docker compose --profile cpu up -d --build

# 4. （Apple GPU）
docker compose --profile gpu up -d --build

# 5. 停止
docker compose down
```

---

## カスタマイズ

- **報酬重み**: `REWARD_ORI`, `REWARD_POS`, `REWARD_SMOOTH` 環境変数（`src/gym_env.py`参照）
- **ドメインランダム化**: `DroneSimEnv._randomize_world()`を拡張
- **PX4パラメータ**: `custom_airframes/`のJSON編集
- **テレメトリ**: UDP 14550をQGroundControl等に転送

---

## テスト・CI

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

## ドキュメント

- **自動生成ドキュメント**: [docs/](docs/) 配下にmkdocs構成
- **GitHub Pages自動公開対応**
- 開発フロー・運用ルール・FAQ・トラブルシューティング等も集約

---

## コントリビュート

- PRテンプレート・CONTRIBUTING.md必須
- コード品質ゲート（ruff, mypy, ament_lint_auto）必須
- Semantic Versioning運用
- 詳細は[docs/](docs/)参照

---

## ライセンス

Apache License 2.0 — `LICENSE`参照。

---

*コントリビューション・Issue歓迎！*
