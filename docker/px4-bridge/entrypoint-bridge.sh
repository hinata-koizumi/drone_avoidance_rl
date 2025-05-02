#!/usr/bin/env bash
# entrypoint-bridge.sh   ★R3 – add outer_motor_bridge
set -euo pipefail
source /opt/ros/humble/setup.bash
source /bridge_ws/install/setup.bash

micrortps_agent -t UDP &
sleep 2

# ROS2 ⇄ Ignition Bridge for tilt joints
ros2 run ros_ign_bridge parameter_bridge \
  /servo/fan1_tilt@std_msgs/msg/Float64@ignition.msgs.Double \
  /servo/fan2_tilt@std_msgs/msg/Float64@ignition.msgs.Double  &
sleep 1

# Bridge Nodes
ros2 run state_bridge        state_bridge        &
ros2 run command_bridge      command_bridge      &
ros2 run angle_bridge        angle_bridge        &
ros2 run outer_motor_bridge  outer_motor_bridge  &   # ★追加

# readiness probe
until ros2 topic echo -n1 /drone/state > /dev/null 2>&1; do
  echo "[bridge] Waiting for /drone/state ..."
  sleep 2
done
echo "[bridge] Ready"
tail -F /dev/null
