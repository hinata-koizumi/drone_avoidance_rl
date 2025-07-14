#!/bin/bash
"""
統合テスト実行スクリプト
"""

set -e

# スクリプトのディレクトリに移動
cd "$(dirname "$0")/.."

echo "Starting integration tests..."

# 環境のセットアップ
./scripts/setup_integration_env.sh

# Docker Composeでサービスを起動
echo "Starting Docker services..."
docker-compose up -d

# サービスの起動を待つ
echo "Waiting for services to start..."
sleep 30

# ヘルスチェック
echo "Performing health checks..."
for service in msgs sim bridge rl-agent; do
    echo "Checking $service..."
    docker-compose ps $service
done

# 統合テストの実行
echo "Running integration tests..."
docker-compose run --rm integration-test

# 結果の表示
echo "Test results:"
ls -la results/

# サービスの停止
echo "Stopping services..."
docker-compose down

echo "Integration tests completed!" 