# Drone Avoidance RL Wiki (日本語)

[English version is here](wiki_en.md)

---

## プロジェクト概要

- PX4 SITL + ROS 2 Humble + Gazebo Garden + RL (Gym API) 統合スタック
- 再現性: Ubuntu 22.04, ROS 2 Humble, PX4 v1.15, Gazebo Garden, マルチステージDocker
- CI/CD自動化: GitHub Actionsによるビルド、テスト、静的解析、E2E、カバレッジ、セキュリティ、リリースノート
- カスタムモデル・エアフレーム対応
- 型安全性、コード品質ゲート、自動生成ドキュメント
- **クロスプラットフォーム**: ARM64 (Apple Silicon) と x86_64 アーキテクチャに最適化
- **BuildKit最適化**: キャッシュマウントによる40-60%高速な再ビルド

---

## ディレクトリ構造

```
drone_avoidance_rl/
├── docker/              # Dockerfiles & entrypoints
│   ├── px4-simulator/   # PX4 SITL + Gazebo Garden
│   ├── bridge/          # ROS 2 bridge nodes
│   ├── msgs/            # Message packages
│   └── rl-agent/        # Reinforcement learning environment
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

## 開発フロー
- ブランチ戦略: main, feature/*, fix/*
- PRテンプレートとCONTRIBUTING.md必須
- 全コードはCI/CD (ビルド、テスト、リント、静的解析) を通過必須
- セマンティックバージョニングによるリリース
- Dependabotによる自動依存関係更新

---

## Dockerアーキテクチャ

### マルチステージビルド
- **ベースイメージ**: ROS 2 Humble + Gazebo Garden + PX4 SITL
- **ブリッジイメージ**: 最適化されたcolconビルドによるROS 2ブリッジノード
- **RLエージェントイメージ**: gymnasiumによるPython RL環境
- **シミュレーターイメージ**: PX4 SITL + Gazebo Gardenランタイム

### ビルド最適化
- **BuildKit 1.4**: apt/pip依存関係のキャッシュマウント
- **並列ビルド**: `$(nproc)` ワーカーによるcolconビルド
- **プラットフォーム特化**: ネイティブARM64サポートでQEMUエミュレーションを排除
- **コンテキスト分割**: サービスごとの専用ビルドコンテキスト

### サービス
- `sim`: PX4 SITL + Gazebo Gardenシミュレーション
- `bridge`: ROS 2ブリッジノード (state_bridge, command_bridge等)
- `msgs`: メッセージパッケージ (drone_msgs, px4_msgs)
- `rl-agent`: 強化学習環境

---

## カスタマイズ
- 報酬重み: `REWARD_ORI`, `REWARD_POS`, `REWARD_SMOOTH` (参照: `src/drone_sim_env.py`)
- ドメインランダム化: `DroneSimEnv._randomize_world()` を拡張
- PX4パラメータ: `assets/airframes/` のJSONを編集
- カスタムモデル: SDFファイルを `assets/models/` に追加
- テレメトリ: UDP 14550をQGroundControlに転送

---

## CI/CD & テスト
- GitHub Actions: ビルド、テスト、リント、E2E、カバレッジ、セキュリティ、リリースノート
- ローカルテスト例:
  ```bash
  # 最適化されたイメージをビルド
  docker compose build --no-cache
  
  # テスト実行
  docker compose -f tests/ci-compose.yml up --abort-on-container-exit
  
  # ローカル開発
  python3 -m pip install pytest gymnasium numpy pyyaml lark
  cd src && PYTHONPATH=$PYTHONPATH:$(pwd) pytest ../tests/test_gym_api.py
  python3 -m pip install ruff mypy
  ruff src/ tests/
  mypy src/ tests/
  ```

---

## パフォーマンス最適化

### ビルドパフォーマンス
- **キャッシュマウント**: apt/pip依存関係がビルド間でキャッシュ
- **並列colcon**: `$(nproc)` ワーカーでビルド時間を2-3倍短縮
- **マルチステージ**: ビルド/ランタイムレイヤー分離でイメージサイズ削減
- **コンテキスト分割**: 専用コンテキストでDockerコンテキストサイズ削減

### ランタイムパフォーマンス
- **ネイティブARM64**: Apple SiliconでQEMUエミュレーションなし
- **メモリ効率**: マルチステージビルドでイメージサイズ削減
- **キャッシュフレンドリー**: コンテナ起動高速化のためのレイヤー最適化

---

## ドキュメント
- 自動生成ドキュメント: [docs/](../docs/)
- mkdocs + GitHub Pages
- アーキテクチャ、開発フロー、トラブルシューティング、FAQ等
- Gym環境のAPIドキュメント

---

## FAQ
- Q: Apple Siliconで動作しますか？  
  A: はい、arm64とx86_64両方に対応し、ネイティブARM64最適化を実装しています。
- Q: CIでGazeboがハングした場合は？  
  A: タイムアウト/終了コード監視が有効です。トラブルシューティングを参照してください。
- Q: カスタムドローンモデルを追加するには？
  A: SDFファイルを `assets/models/` に配置し、docker-composeボリュームマウントを更新してください。
- Q: ビルド時間が遅い場合は？
  A: BuildKitキャッシュマウント付きの `docker compose build` を使用すると再ビルドが40-60%高速化されます。
- Q: 報酬関数をカスタマイズするには？
  A: 環境変数を変更するか、`src/drone_sim_env.py` の `DroneSimEnv` クラスを拡張してください。

---

## ライセンス
Apache License 2.0 