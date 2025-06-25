#!/bin/bash
set -e
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f /bridge_ws/install/setup.bash ]; then
  source /bridge_ws/install/setup.bash
fi

# PYTHONPATHの安全な設定
if [ -z "$PYTHONPATH" ]; then
  export PYTHONPATH="/bridge_ws/src:/bridge_ws/install/lib/python3.10/site-packages"
else
  export PYTHONPATH="/bridge_ws/src:/bridge_ws/install/lib/python3.10/site-packages:$PYTHONPATH"
fi

exec "$@"

# デバッグ用出力
export PATH="/bridge_ws/install/bin:$PATH"
echo "[entrypoint-bridge] PATH=$PATH"

echo "[entrypoint-bridge] PYTHONPATH=$PYTHONPATH"
# 引数があればそのまま実行、なければros2 launch
if [ $# -gt 0 ]; then
  exec "$@"
else
  # 例: state_bridge起動
  ros2 run state_bridge state_bridge_node || { echo "state_bridge_nodeの起動に失敗"; exit 1; }
fi 