#!/bin/bash
set -e

# Source ROS and workspace overlays
source /opt/ros/humble/setup.bash
[ -f /msgs_ws/install/setup.bash ] && source /msgs_ws/install/setup.bash
[ -f /bridge_ws/install/setup.bash ] && source /bridge_ws/install/setup.bash
[ -f /sim_ws/install/setup.bash ] && source /sim_ws/install/setup.bash

exec "$@"
