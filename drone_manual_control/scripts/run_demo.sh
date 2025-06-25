#!/bin/bash

# ドローン手動制御デモ実行スクリプト

set -e

echo "🚁 Drone Manual Control Demo"
echo "============================"

# 環境変数を設定
export DISPLAY=${DISPLAY:-:0}

# 1. 環境をビルド
echo "🔨 Building environment..."
docker-compose build

# 2. シミュレーションを起動
echo "🚀 Starting simulation..."
docker-compose up -d simulator

# 3. ブリッジノードを起動
echo "🌉 Starting bridge nodes..."
docker-compose up -d bridge

# 4. 少し待機
echo "⏳ Waiting for simulation to initialize..."
sleep 10

# 5. 手動制御ノードを起動
echo "🎮 Starting manual control..."
docker-compose up -d manual_control

echo ""
echo "🎉 Demo is running!"
echo ""
echo "Available commands:"
echo "- View logs: docker-compose logs -f manual_control"
echo "- Stop demo: docker-compose down"
echo "- Restart: docker-compose restart manual_control"
echo ""
echo "Predefined actions:"
echo "- hover: Maintain hover position"
echo "- takeoff: Takeoff sequence"
echo "- landing: Landing sequence"
echo "- waypoint_forward: Move forward 5m"
echo "- circle_flight: Circular flight pattern"
echo "- square_pattern: Square flight pattern"
echo ""
echo "To change action, edit config/action_sequences.yaml and restart manual_control" 