# Drone Avoidance RL Stack

[![CI](https://github.com/hinata-koizumi/drone_avoidance_rl/actions/workflows/ci.yml/badge.svg)](https://github.com/hinata-koizumi/drone_avoidance_rl/actions)  
📘 [English version is here](README.md)

---

## 概要

- **PX4 SITL + ROS 2 Humble + Gazebo Fortress (LTS) + RL (Gym API) 統合スタック**
- **完全再現性**: Ubuntu 22.04, ROS 2 Humble, PX4 v1.15, Gazebo Fortress (LTS), ros_gz, multi-stage Docker
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
# 1. Clone & submodules
git clone https://github.com/hinata-koizumi/drone_avoidance_rl.git
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

## バージョン管理

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

## ドキュメント

- **自動生成ドキュメント**: [docs/](docs/) 配下にmkdocs構成
- **GitHub Pages自動公開対応**
- アーキテクチャ・開発フロー・FAQ・トラブルシューティング等も集約

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

## 共通基盤・ユーティリティ

- **BridgeBase基底クラス**: `src/common/bridge_base.py` にて、各種Bridgeノード（angle_bridge, outer_motor_bridge, command_bridge, state_bridge等）のQoS設定・パラメータ取得・ログ出力を共通化。新規BridgeノードはBridgeBaseを継承し、パラメータdictを渡すだけで実装可能。
- **共通ユーティリティ**: `src/common/utils.py` にclamp等の汎用関数を集約。各ノード・環境でimportして利用。
- **メリット**: コード重複削減・保守性向上・新規ノード追加が容易。
