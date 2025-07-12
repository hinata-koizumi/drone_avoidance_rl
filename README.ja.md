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
- **マルチステージDocker**: BuildKit最適化による効率的なビルドとデプロイ
- **クロスプラットフォーム**: ARM64 (Apple Silicon) と x86_64 アーキテクチャに最適化

---

## プロジェクト構造

```
drone_avoidance_rl/
├── docker/              # Dockerfiles & entrypoints
├── src/                 # ROS 2 nodes, Gym env, custom msgs
│   ├── drone_sim_env.py # Gym API 準拠のドローン環境
│   ├── common/          # 共通ユーティリティ・ベースクラス
│   └── [bridge_nodes]/  # 各種ブリッジノード
├── assets/
│   ├── models/          # カスタムSDFモデル
│   └── airframes/       # PX4エアフレーム設定
├── tests/               # 統合・E2Eテスト
├── docs/                # 自動生成ドキュメント
└── tools/               # 開発支援スクリプト
```

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
cp -r ~/my_drone_sdf      assets/models/drone_model
cp    ~/4500_my_drone.json assets/airframes/
```

### 3. ビルド・起動
```bash
# 最適化されたDockerイメージをビルド
docker compose build

# 全サービスを起動
docker compose up -d

# コンテナ状態を確認
docker compose ps
```

### 4. 環境の検証
```bash
# シミュレーションログを確認
docker compose logs sim --tail 20

# ブリッジノードを確認
docker compose logs bridge --tail 10

# RLエージェントシェルにアクセス
docker compose exec rl-agent bash
```

### 5. 停止
```bash
docker compose down
```

---

## パフォーマンス最適化

### ビルド最適化
- **BuildKit 1.4**: apt/pip依存関係のキャッシュマウント（再ビルド40-60%高速化）
- **並列ビルド**: `$(nproc)` ワーカーによるcolconビルド
- **マルチステージ**: ビルドとランタイムレイヤーの分離
- **プラットフォーム特化**: ARM64とx86_64に最適化

### ランタイム最適化
- **クロスプラットフォーム**: ネイティブARM64サポートでQEMUエミュレーションを排除
- **メモリ効率**: マルチステージビルドでイメージサイズ削減
- **キャッシュフレンドリー**: コンテナ起動高速化のためのレイヤー最適化

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
- **観測空間**: 15次元 (姿勢、位置、速度、角速度、風)
- **行動空間**: 4次元 (2つのモーターのスロットルと角度)
- **報酬関数**: REWARD_ORI, REWARD_POS, REWARD_SMOOTHの重み付き和

### 使用例
```python
from drone_sim_env import DroneSimEnv
from stable_baselines3 import SAC

# 環境の作成
env = DroneSimEnv(reward_mode="hover", episode_max_steps=1000)

# 学習
model = SAC("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=100_000)
```

### 報酬モード
- `hover`: ホバリング重視
- `path_follow`: 経路追従
- `obstacle_avoid`: 障害物回避
- `default`: 従来型 (環境変数で重み調整)

---

## カスタマイズ

### 報酬重みの調整
環境変数を使用して報酬重みを変更：
```bash
export REWARD_ORI=1.0
export REWARD_POS=0.5
export REWARD_SMOOTH=0.1
```

### ドメインランダム化
`DroneSimEnv._randomize_world()` を拡張して環境ランダム化を実装。

### PX4パラメータ
`custom_airframes/` のJSONファイルを編集してPX4パラメータを調整。

---

## CI/CD環境でのテスト

### Docker環境テスト
```bash
# CIと同じ環境とコマンドでテスト
docker compose build --no-cache
bash tools/setup_rosdep_local.sh
docker compose -f tests/ci-compose.yml up --abort-on-container-exit
```

### ローカル環境テスト
```bash
# Python 3.10.12を推奨
pyenv install 3.10.12
pyenv local 3.10.12

# 依存パッケージのインストール
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

トピック名はlaunchファイルまたはノードパラメータで変更可能。

---

## バージョン管理

- ROS 2とGazebo/Ignitionバージョンは`.env`ファイルで一元管理
- Package.xmlバージョン整合性は`check_package_versions.sh`で自動チェック
- Dependabotが依存関係を自動監視しPRを作成

---

## ドキュメント

- **自動生成ドキュメント**: [docs/](docs/) (mkdocs構造)
- **GitHub Pages自動公開**
- アーキテクチャ、開発フロー、FAQ、トラブルシューティング

---

## コントリビューション

- PRテンプレートとCONTRIBUTING.md必須
- コード品質ゲート (ruff, mypy, ament_lint_auto) 必須
- セマンティックバージョニング
- 詳細は [docs/](docs/) を参照

---

## ライセンス

Apache License 2.0 — `LICENSE` を参照。

---

*コントリビューションとイシューの報告を歓迎します！*
