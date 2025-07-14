#!/bin/bash
set -e
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f /manual_ws/install/setup.bash ]; then
  source /manual_ws/install/setup.bash
fi

# PYTHONPATHの安全な設定
if [ -z "$PYTHONPATH" ]; then
  export PYTHONPATH="/manual_ws/src:/manual_ws/install/lib/python3.10/site-packages"
else
  export PYTHONPATH="/manual_ws/src:/manual_ws/install/lib/python3.10/site-packages:$PYTHONPATH"
fi

# デバッグ用出力
export PATH="/manual_ws/install/bin:$PATH"
echo "[entrypoint-manual-control] PATH=$PATH"
echo "[entrypoint-manual-control] PYTHONPATH=$PYTHONPATH"

# 引数があればそのまま実行、なければaction_executorを起動
if [ $# -gt 0 ]; then
  exec "$@"
else
  # デフォルトでaction_executorを起動
  ros2 run manual_control action_executor || { echo "action_executorの起動に失敗"; exit 1; }
fi 