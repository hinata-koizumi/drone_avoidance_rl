# Drone Avoidance RL Stack

[![CI](https://github.com/hinata-koizumi/drone_avoidance_rl/actions/workflows/ci.yml/badge.svg)](https://github.com/hinata-koizumi/drone_avoidance_rl/actions)  
📘 [English version is here](README.md)

---

## 概要

PX4 SITL + ROS 2 Humble + Gazebo Garden + 強化学習 (Gym API) の統合スタックです。

**このプロジェクトは、災害大国である日本において、災害現場での安全で確実なドローン派遣を実現することを目指しています。** 強化学習による自律飛行技術を開発し、災害時の捜索・救助活動や被害状況の把握に貢献するドローンの実用化を目指します。

### 特徴
- **再現性**: Ubuntu 22.04, ROS 2 Humble, PX4 v1.15, Gazebo Garden (LTS)
- **CI/CD**: GitHub Actions による自動ビルド・テスト・静的解析
- **カスタマイズ**: 独自ドローンモデル・エアフレーム対応
- **型安全性**: mypy, ruff によるコード品質管理
- **マルチステージDocker**: 効率的なビルドとデプロイ

---

## プロジェクト構造

```
drone_avoidance_rl/
├── docker/              # Dockerfiles & entrypoints
├── src/                 # ROS 2 nodes, Gym env, custom msgs
│   ├── drone_sim_env.py # Gym API 準拠のドローン環境
│   ├── common/          # 共通ユーティリティ・ベースクラス
│   └── [bridge_nodes]/  # 各種ブリッジノード
├── drone_manual_control/ # 手動制御環境
│   ├── src/manual_control/ # 事前定義行動実行ノード
│   ├── config/          # 行動シーケンス・パラメータ
│   └── scripts/         # セットアップ・デモスクリプト
├── custom_model/        # カスタムSDFモデル
├── custom_airframes/    # PX4エアフレーム設定
├── tests/               # 統合・E2Eテスト
├── docs/                # 自動生成ドキュメント
└── tools/               # 開発支援スクリプト
```

---

## 🚁 ドローン手動制御チュートリアル

**強化学習なしでドローン制御を体験しましょう！** 手動制御環境では、事前定義されたドローン行動を実行し、基本的な飛行力学を理解できます。

### クイックスタート - 手動制御

#### 1. 手動制御環境のセットアップ
```bash
# 手動制御ディレクトリに移動
cd drone_manual_control

# 環境セットアップ（メインプロジェクトからコンポーネントをコピー）
./scripts/setup_environment.sh

# Dockerコンテナをビルド
docker-compose build
```

#### 2. 事前定義ドローン行動の実行
```bash
# 手動制御付きシミュレーションを開始
./scripts/run_demo.sh

# または段階的に実行：
docker-compose up -d simulator    # Gazeboシミュレーション開始
docker-compose up -d bridge       # ブリッジノード開始
docker-compose up -d manual_control  # 行動実行ノード開始
```

#### 3. 利用可能な事前定義行動

**基本飛行行動:**
- **Hover**: 安定したホバリング位置を維持（10秒間）
- **Takeoff**: 目標高度への垂直上昇（5秒間）
- **Landing**: 制御された降下と着陸（8秒間）

**移動パターン:**
- **Waypoint Navigation**: 特定座標への移動
  - Forward: 前方5m移動
  - Backward: 後方5m移動
  - Left/Right: 左右5m移動
- **Circle Flight**: 半径5mの円形パターン（20秒間）
- **Square Pattern**: 5m辺の四角形飛行パターン（40秒間）

**複合シーケンス:**
- **Takeoff and Hover**: 完全な離陸→ホバリングシーケンス
- **Exploration**: 離陸→前方移動→ホバリング
- **Return to Base**: 原点への帰還→着陸

#### 4. 行動のカスタマイズ
`drone_manual_control/config/action_sequences.yaml` を編集して行動を変更：

```yaml
action_sequences:
  - name: "custom_hover"
    action_type: "hover"
    duration: 15.0  # 15秒間
    parameters:
      target_altitude: 5.0  # 5m高度
    next_action: "landing"  # 次の行動
```

#### 5. 監視と制御
```bash
# ログの表示
docker-compose logs -f manual_control

# ROSトピックの確認
docker-compose exec manual_control ros2 topic list
docker-compose exec manual_control ros2 topic echo /drone/control_command

# 環境の停止
docker-compose down
```

### 手動制御の特徴

- **RL不要**: 複雑なアルゴリズムなしでドローン制御を体験
- **事前定義行動**: すぐに使える飛行パターン
- **リアルタイム可視化**: Gazeboでドローンの動きを観察
- **簡単カスタマイズ**: YAML設定で行動を変更
- **安全機能**: 高度・距離制限が組み込み済み

---

## 前提条件

- Docker Desktop 4.30+ (BuildKit有効)
- 12GB+ RAM
- macOS 12+, Linux, Windows (WSL2)
- Apple Silicon (arm64) / x86_64 対応
- (オプション) Apple M-series または NVIDIA CUDA 12

---

## クイックスタート

### 1. リポジトリのクローン
```bash
git clone https://github.com/hinata-koizumi/drone_avoidance_rl.git
cd drone_avoidance_rl
git submodule update --init --recursive
```

### 2. (オプション) カスタムモデルの追加
```bash
# 独自ドローンモデルを追加
cp -r ~/my_drone_sdf      custom_model/drone_model
cp    ~/4500_my_drone.json custom_airframes/
```

### 3. ビルド・起動
```bash
# CPU版
docker compose --profile cpu up -d --build

# Apple GPU版 (M1/M2)
docker compose --profile gpu up -d --build
```

### 4. 停止
```bash
docker compose down
```

---

## ローカル開発環境

### Python環境セットアップ
```bash
# 依存パッケージのインストール
python3 -m pip install --upgrade pip
pip install -r requirements.txt
```

### テスト実行
```bash
# 環境変数設定
export PYTHONPATH=$(pwd):$(pwd)/src

# テスト実行
pytest tests/test_gym_api.py
pytest tests/test_rl_longrun.py
pytest tests/test_gym_env.py
```

### 静的解析
```bash
ruff src/ tests/
mypy src/ tests/
```

---

## Gym API 仕様

### 環境仕様
- **観測空間**: 15次元（姿勢, 位置, 速度, 角速度, 風）
- **行動空間**: 4次元（2モーターのスロットル・角度）
- **報酬関数**: REWARD_ORI, REWARD_POS, REWARD_SMOOTH の加重和

### 使用例
```python
from drone_sim_env import DroneSimEnv
from stable_baselines3 import SAC

# 環境作成
env = DroneSimEnv(reward_mode="hover", episode_max_steps=1000)

# 学習
model = SAC("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=100_000)
```

### 報酬モード
- `hover`: ホバリング特化
- `path_follow`: 経路追従
- `obstacle_avoid`: 障害物回避
- `default`: 従来型（環境変数で重み調整）

---

## カスタマイズ

### 報酬重みの調整
環境変数で報酬重みを変更できます：
```bash
export REWARD_ORI=1.0
export REWARD_POS=0.5
export REWARD_SMOOTH=0.1
```

### ドメインランダム化
`DroneSimEnv._randomize_world()` を拡張して環境のランダム化が可能です。

### PX4パラメータ
`custom_airframes/` 内のJSONファイルを編集してPX4パラメータを調整できます。

---

## CI/CD との同一環境でのテスト

### Docker環境でのテスト
```bash
# CIと同じ環境・コマンドでテスト
docker compose build --no-cache
bash tools/setup_rosdep_local.sh
docker compose -f tests/ci-compose.yml up --abort-on-container-exit
```

### ローカル環境でのテスト
```bash
# Python 3.10.12を推奨
pyenv install 3.10.12
pyenv local 3.10.12

# 依存インストール
python3 -m pip install --upgrade pip
python3 -m pip install pytest gymnasium numpy pyyaml lark ruff mypy types-PyYAML

# テスト実行
cd src
PYTHONPATH=$PYTHONPATH:$(pwd) pytest ../tests/test_gym_api.py
ruff src/ tests/
mypy src/ tests/
```

---

## 主要ROS 2ノード・トピック

- `/drone{N}/inner_propeller_cmd` (DroneControlCommand)
- `/drone{N}/state` (DroneState)

トピック名はlaunchファイルやノードパラメータで変更可能です。

---

## バージョン管理

- ROS 2やGazebo/Ignitionのバージョンは`.env`ファイルで一元管理
- package.xmlのバージョン一貫性は`check_package_versions.sh`で自動チェック
- Dependabotによる依存関係の自動監視・PR作成

---

## ドキュメント

- **自動生成ドキュメント**: [docs/](docs/) (mkdocs構造)
- **GitHub Pages自動公開**
- アーキテクチャ、開発フロー、FAQ、トラブルシューティング

---

## 貢献

- PR template & CONTRIBUTING.md 必須
- コード品質ゲート (ruff, mypy, ament_lint_auto) 必須
- セマンティックバージョニング
- 詳細は [docs/](docs/) を参照

---

## ライセンス

Apache License 2.0 — `LICENSE` を参照。

---

*Contributions and issues are welcome!*
