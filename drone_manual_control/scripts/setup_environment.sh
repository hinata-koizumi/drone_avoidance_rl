#!/bin/bash

# ドローン手動制御環境セットアップスクリプト

set -e

echo "🚁 Drone Manual Control Environment Setup"
echo "========================================"

# 1. 既存環境からコンポーネントをコピー
echo "📋 Copying components from existing environment..."

# 基盤コンポーネント
if [ -d "../drone_avoidance_rl/src/common" ]; then
    cp -r ../drone_avoidance_rl/src/common src/
    echo "✅ Copied common components"
fi

if [ -d "../drone_avoidance_rl/src/drone_msgs" ]; then
    cp -r ../drone_avoidance_rl/src/drone_msgs src/
    echo "✅ Copied drone_msgs"
fi

if [ -d "../drone_avoidance_rl/src/px4_msgs" ]; then
    cp -r ../drone_avoidance_rl/src/px4_msgs src/
    echo "✅ Copied px4_msgs"
fi

# シミュレーション環境
if [ -d "../drone_avoidance_rl/src/sim_launch" ]; then
    cp -r ../drone_avoidance_rl/src/sim_launch src/
    echo "✅ Copied sim_launch"
fi

# ブリッジノード
if [ -d "../drone_avoidance_rl/src/command_bridge" ]; then
    cp -r ../drone_avoidance_rl/src/command_bridge src/
    echo "✅ Copied command_bridge"
fi

if [ -d "../drone_avoidance_rl/src/state_bridge" ]; then
    cp -r ../drone_avoidance_rl/src/state_bridge src/
    echo "✅ Copied state_bridge"
fi

if [ -d "../drone_avoidance_rl/src/angle_bridge" ]; then
    cp -r ../drone_avoidance_rl/src/angle_bridge src/
    echo "✅ Copied angle_bridge"
fi

if [ -d "../drone_avoidance_rl/src/outer_motor_bridge" ]; then
    cp -r ../drone_avoidance_rl/src/outer_motor_bridge src/
    echo "✅ Copied outer_motor_bridge"
fi

# モデルとエアフレーム
if [ -d "../drone_avoidance_rl/models" ]; then
    cp -r ../drone_avoidance_rl/models .
    echo "✅ Copied models"
fi

if [ -d "../drone_avoidance_rl/custom_airframes" ]; then
    cp -r ../drone_avoidance_rl/custom_airframes .
    echo "✅ Copied custom_airframes"
fi

# 2. 設定ファイルをコピー
if [ -d "../drone_avoidance_rl/config" ]; then
    cp -r ../drone_avoidance_rl/config .
    echo "✅ Copied config files"
fi

# 3. Dockerファイルをコピー
if [ -d "../drone_avoidance_rl/docker" ]; then
    cp -r ../drone_avoidance_rl/docker .
    echo "✅ Copied docker files"
fi

# 4. 権限を設定
chmod +x docker/entrypoint-manual-control.sh
chmod +x scripts/setup_environment.sh

echo ""
echo "🎉 Environment setup completed!"
echo ""
echo "Next steps:"
echo "1. Build the environment: docker-compose build"
echo "2. Start the simulation: docker-compose up -d"
echo "3. Run manual control: ros2 run manual_control action_executor"
echo ""
echo "Available actions:"
echo "- hover: Maintain hover position"
echo "- takeoff: Takeoff sequence"
echo "- landing: Landing sequence"
echo "- waypoint_forward: Move forward 5m"
echo "- circle_flight: Circular flight pattern"
echo "- square_pattern: Square flight pattern" 