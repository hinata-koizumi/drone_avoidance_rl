#!/usr/bin/env bash
set -euo pipefail
set +u

# ROS 2 Humble環境セットアップ
source /opt/ros/humble/setup.bash
if [ -f /bridge_ws/install/setup.bash ]; then
  source /bridge_ws/install/setup.bash
fi

# デバッグ用出力
export PATH="/bridge_ws/install/bin:$PATH"
echo "[entrypoint-bridge] PATH=$PATH"

# 引数があればそのまま実行、なければros2 launch
if [ $# -gt 0 ]; then
  exec "$@"
else
  # 例: state_bridge起動
  ros2 run state_bridge state_bridge_node || { echo "state_bridge_nodeの起動に失敗"; exit 1; }
fi 