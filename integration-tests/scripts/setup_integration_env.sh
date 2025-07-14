#!/bin/bash
"""
統合テスト環境のセットアップスクリプト
"""

set -e

echo "Setting up integration test environment..."

# サブモジュールの初期化
echo "Initializing submodules..."
git submodule update --init --recursive

# 必要なディレクトリの作成
echo "Creating necessary directories..."
mkdir -p results
mkdir -p logs
mkdir -p tests

# Docker Composeの設定確認
echo "Checking Docker Compose configuration..."
if ! command -v docker-compose &> /dev/null; then
    echo "Error: docker-compose is not installed"
    exit 1
fi

# 環境変数の設定
export ROS_DOMAIN_ID=0
export PYTHONPATH=/workspace

echo "Integration test environment setup completed!" 