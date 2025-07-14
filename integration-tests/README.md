# Integration Tests

このディレクトリは、drone-msgs、drone-sim-core、drone-rlの統合テスト環境です。

## 概要

統合テスト環境は以下の3つのリポジトリを統合してテストします：

- **drone-msgs**: ROS 2メッセージ定義
- **drone-sim-core**: シミュレーション環境（PX4 + Ignition Gazebo Garden）
- **drone-rl**: 強化学習環境（Gymnasium API）

## アーキテクチャ

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   drone-msgs    │    │  drone-sim-core │    │    drone-rl     │
│   (Messages)    │    │  (Simulation)   │    │  (RL Environment)│
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                    ┌─────────────────┐
                    │ Integration Test│
                    │   Environment   │
                    └─────────────────┘
```

## セットアップ

### 前提条件

- Docker
- Docker Compose
- Git（サブモジュール用）

### 初期セットアップ

```bash
# リポジトリのクローン（サブモジュール含む）
git clone --recursive <repository-url>
cd integration-tests

# 環境のセットアップ
./scripts/setup_integration_env.sh
```

## 使用方法

### 統合テストの実行

```bash
# 完全な統合テストの実行
./scripts/run_tests.sh

# または手動で実行
docker-compose up -d
docker-compose run --rm integration-test
docker-compose down
```

### 個別サービスのテスト

```bash
# メッセージ定義のテスト
docker-compose run --rm msgs

# シミュレーション環境のテスト
docker-compose run --rm sim

# ブリッジノードのテスト
docker-compose run --rm bridge

# RLエージェントのテスト
docker-compose run --rm rl-agent
```

## テスト内容

### 1. システム統合テスト

- **Docker Services**: 全サービスの起動確認
- **ROS2 Nodes**: 必要なノードの動作確認
- **ROS2 Topics**: トピック通信の確認
- **Message Types**: メッセージ型の利用可能性確認

### 2. 機能統合テスト

- **Gym Environment**: Gymnasium API環境の作成と動作確認
- **Simulation Communication**: シミュレーションとの通信テスト
- **Full Workflow**: 完全な統合ワークフローのテスト

### 3. パフォーマンステスト

- **Response Time**: システム応答時間の測定
- **Memory Usage**: メモリ使用量の監視
- **CPU Usage**: CPU使用率の監視

## 結果とレポート

テスト結果は以下の場所に保存されます：

- **JSON Report**: `results/integration_test_report_YYYYMMDD_HHMMSS.json`
- **Logs**: `logs/` ディレクトリ
- **Artifacts**: CI/CD実行時の成果物

### レポート形式

```json
{
  "timestamp": "20240101_120000",
  "total_tests": 4,
  "passed": 3,
  "failed": 1,
  "results": [
    {
      "test_name": "ROS2 Nodes",
      "status": "PASS",
      "duration": 2.5,
      "error_message": null
    }
  ]
}
```

## CI/CD

### GitHub Actions

統合テストは以下のタイミングで自動実行されます：

- **Push to main/develop**: メインブランチへのプッシュ時
- **Pull Request**: プルリクエスト作成時
- **Scheduled**: 毎日午前2時（定期実行）
- **Manual**: 手動実行（workflow_dispatch）

### 実行環境

- **OS**: Ubuntu Latest
- **Docker**: 最新版
- **Python**: 3.10
- **ROS**: Humble

## トラブルシューティング

### よくある問題

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

### ログの確認

```bash
# 全サービスのログ
docker-compose logs

# 特定サービスのログ
docker-compose logs sim

# リアルタイムログ
docker-compose logs -f
```

## 開発者向け情報

### テストの追加

新しいテストを追加する場合：

1. `tests/test_integration.py` にテストケースを追加
2. `tests/run_integration_tests.py` にテスト関数を追加
3. CI/CDワークフローでテストを実行

### カスタマイズ

- **テストタイムアウト**: 各テストの `timeout` パラメータを調整
- **ヘルスチェック**: `docker-compose.yml` の `healthcheck` 設定を調整
- **環境変数**: `.env` ファイルで環境変数を設定

## ライセンス

このプロジェクトはMITライセンスの下で公開されています。 