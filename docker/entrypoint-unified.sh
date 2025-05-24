#!/bin/bash
set -e

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# PX4環境変数
export PX4_HOME_LAT=35.6586
export PX4_HOME_LON=139.7454
export PX4_HOME_ALT=10.0
export PX4_SIM_MODEL=drone_model

# PX4 SITL起動（バックグラウンド）
if [ -x /usr/local/bin/px4 ]; then
  # px4-alias.shがなければtouchして空ファイルを作る
  if [ ! -f /PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/px4-alias.sh ]; then
    touch /PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/px4-alias.sh
    chmod +x /PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/px4-alias.sh
  fi
  /usr/local/bin/px4 &
  PX4_PID=$!
fi

# ROS 2ワークスペースのsource
if [ -f /sim_ws/install/setup.bash ]; then
  source /sim_ws/install/setup.bash
fi

# Gazebo Garden起動（必要なら）
# gz sim ... &

# シミュレーションlaunch
ros2 launch sim_launch sim_all.launch.py headless:=true

# PX4プロセスの監視
if [ -n "$PX4_PID" ]; then
  wait $PX4_PID
fi 