# Drone Avoidance RL

ROS 2 Humbleベースのドローン回避強化学習プロジェクトです。PX4とIgnition Gazebo Gardenを統合し、強化学習によるドローン制御を実現します。

## プロジェクト構成

このプロジェクトは以下の4つの独立したリポジトリで構成されています：

### 1. drone-msgs
- **目的**: ROS 2メッセージ定義
- **内容**: ドローン制御用のカスタムメッセージ型
- **技術**: ROS 2 Humble, CMake

### 2. drone-sim-core
- **目的**: シミュレーション環境
- **内容**: PX4 + Ignition Gazebo Garden + ブリッジノード
- **技術**: ROS 2 Humble, PX4, Ignition Gazebo Garden

### 3. drone-rl
- **目的**: 強化学習環境
- **内容**: Gymnasium API準拠のRL環境
- **技術**: Python, Gymnasium, Ray

### 4. integration-tests
- **目的**: 統合テスト環境
- **内容**: 3つのリポジトリを統合したE2Eテスト
- **技術**: Docker Compose, pytest

## 統合テスト環境

統合テスト環境は、3つの独立したリポジトリを統合してエンドツーエンドテストを実行します。

### アーキテクチャ

```
 ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
 │   drone-msgs    │     │  drone-sim-core │     │    drone-rl     │
 │   (Messages)    │     │  (Simulation)   │     │ (RL Environment)│
 └─────────────────┘     └─────────────────┘     └─────────────────┘
          │                       │                       │
          └───────────────────────┼───────────────────────┘
                                  │
                         ┌─────────────────┐
                         │ Integration Test│
                         │   Environment   │
                         └─────────────────┘
```

### 統合テストの実行

```bash
# 環境の確認
cd integration-tests
./scripts/check_environment.sh

# 統合テストの実行
./scripts/run_tests.sh

# または手動で実行
docker-compose up -d
docker-compose run --rm integration-test
docker-compose down
```

### テスト内容

1. **システム統合テスト**
   - Dockerサービスの起動確認
   - ROS2ノードの動作確認
   - トピック通信の確認
   - メッセージ型の利用可能性確認

2. **機能統合テスト**
   - Gymnasium API環境の作成と動作確認
   - シミュレーションとの通信テスト
   - 完全な統合ワークフローのテスト

3. **パフォーマンステスト**
   - システム応答時間の測定
   - メモリ使用量の監視
   - CPU使用率の監視

## セットアップ

### 前提条件

- Docker
- Docker Compose
- Git（サブモジュール用）
- Python 3.10+

### 初期セットアップ

```bash
# リポジトリのクローン（サブモジュール含む）
git clone --recursive <repository-url>
cd drone_avoidance_rl

# サブモジュールの初期化
git submodule update --init --recursive

# 統合テスト環境の確認
cd integration-tests
./scripts/check_environment.sh
```

## 使用方法

### 個別リポジトリのテスト

```bash
# drone-msgsのテスト
cd drone-msgs
colcon build
colcon test

# drone-sim-coreのテスト
cd drone-sim-core
docker-compose up -d
docker-compose logs

# drone-rlのテスト
cd drone-rl
python -m pytest tests/
```

### 統合テストの実行

```bash
# 完全な統合テスト
cd integration-tests
./scripts/run_tests.sh

# クイックテスト
python tests/run_integration_tests.py --quick
```

## CI/CD

### GitHub Actions

各リポジトリには独立したCI/CDワークフローがあります：

- **drone-msgs**: メッセージ定義のビルドとテスト
- **drone-sim-core**: シミュレーション環境のビルドとテスト
- **drone-rl**: RL環境のビルドとテスト
- **integration-tests**: 統合テストの実行

### 統合テストの実行タイミング

- **Push to main/develop**: メインブランチへのプッシュ時
- **Pull Request**: プルリクエスト作成時
- **Scheduled**: 毎日午前2時（定期実行）
- **Manual**: 手動実行（workflow_dispatch）

## 開発ガイド

### 新しい機能の追加

1. **メッセージ定義の追加** (drone-msgs)
   ```bash
   cd drone-msgs
   # 新しい.msgファイルを追加
   colcon build
   ```

2. **シミュレーション機能の追加** (drone-sim-core)
   ```bash
   cd drone-sim-core
   # 新しいノードやブリッジを追加
   docker-compose build
   ```

3. **RL環境の拡張** (drone-rl)
   ```bash
   cd drone-rl
   # 新しい環境やエージェントを追加
   python -m pytest tests/
   ```

4. **統合テストの更新** (integration-tests)
   ```bash
   cd integration-tests
   # 新しいテストケースを追加
   ./scripts/run_tests.sh
   ```

### トラブルシューティング

#### よくある問題

1. **サブモジュールの初期化エラー**
   ```bash
   git submodule update --init --recursive
   ```

2. **Dockerサービスの起動失敗**
   ```bash
   docker-compose down
   docker system prune -f
   docker-compose up -d
   ```

3. **ROS2通信エラー**
   ```bash
   export ROS_DOMAIN_ID=0
   docker-compose restart
   ```

4. **メモリ不足**
   ```bash
   # Docker Desktopのメモリ制限を増やす
   # または
   docker-compose down
   docker system prune -a
   ```

## 技術仕様

### 対応バージョン

- **ROS**: Humble
- **PX4**: 最新版
- **Ignition Gazebo**: Garden
- **Python**: 3.10+
- **Docker**: 最新版

### アーキテクチャ原則

- **モジュラー設計**: 各リポジトリは独立して動作
- **Docker化**: 全ての環境はDockerコンテナで実行
- **CI/CD**: 自動化されたテストとデプロイ
- **GPU対応**: 必要に応じてGPUアクセラレーション

## ライセンス

このプロジェクトはMITライセンスの下で公開されています。

## 貢献

プロジェクトへの貢献を歓迎します。詳細は[CONTRIBUTING.md](CONTRIBUTING.md)を参照してください。

### 開発フロー

1. 機能ブランチを作成
2. 変更を実装
3. テストを実行
4. プルリクエストを作成
5. 統合テストが通ることを確認
6. マージ

## サポート

問題や質問がある場合は、GitHubのIssuesページで報告してください。
