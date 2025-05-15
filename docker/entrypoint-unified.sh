#!/usr/bin/env bash
set -eo pipefail

MODE=${1:-sim}
shift || true

source /opt/ros/humble/setup.bash
[ -f /sim_ws/install/setup.bash ] && source /sim_ws/install/setup.bash
PYTHON_VERSION=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
export PYTHONPATH="$(find /sim_ws/install -type d -name site-packages | tr '\n' ':'):$PYTHONPATH"
export PATH="/sim_ws/install/command_bridge/bin:/sim_ws/install/state_bridge/bin:/sim_ws/install/angle_bridge/bin:/sim_ws/install/outer_motor_bridge/bin:$PATH"
echo "[entrypoint] PATH=$PATH"

export GZ_RESOURCE_PATH="/models:/usr/share/gz/garden/models:${GZ_RESOURCE_PATH:-}"
export GZ_SIM_RESOURCE_PATH="/models:/usr/share/gz/garden/models:${GZ_SIM_RESOURCE_PATH:-}"
export GAZEBO_MODEL_PATH="/models:/usr/share/gz/garden/models:${GAZEBO_MODEL_PATH:-}"
export IGN_IP=127.0.0.1
export GZ_IP=127.0.0.1

case "$MODE" in
  sim)
    # Gazebo起動
    timeout 180s gz sim -r /usr/share/ignition/gazebo/worlds/empty.sdf --headless-rendering --verbose &
    sleep 3
    # モデルスポーン
    if [[ -f "/models/drone_model/model.sdf" ]]; then
      gz service -s /world/empty/create --reqtype ignition.msgs.EntityFactory \
        --reptype ignition.msgs.Boolean --timeout 300 \
        --req "sdf_filename: \"/models/drone_model/model.sdf\" pose: { position:{z:0.25} }"
    fi
    # PX4起動
    px4 -i 0 -d -s /PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/rcS -w build/px4_sitl_rtps &
    until nc -z localhost 11345; do sleep 1; done
    echo "[entrypoint] PX4 RTPS ready."
    tail -F /dev/null
    ;;
  ros)
    # ROS 2 launch (例: sim_all.launch.py)
    ros2 launch sim_launch sim_all.launch.py "$@"
    ;;
  bash)
    exec "/bin/bash" "$@"
    ;;
  *)
    exec "$MODE" "$@"
    ;;
esac 