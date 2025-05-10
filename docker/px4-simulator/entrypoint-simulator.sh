#!/usr/bin/env bash
# entrypoint-simulator.sh   ★2025-05-02 R5
#   * PX4 ログに ch0-1 (SERVO) を含める
set -eo pipefail
source /opt/ros/humble/setup.sh

export SYS_AUTOSTART=${SYS_AUTOSTART:-4500}
export MODEL_PATH=${MODEL_PATH:-/models/drone_model}
export GZ_RESOURCE_PATH="/models:/usr/share/gz/garden/models:${GZ_RESOURCE_PATH:-}"
export GZ_SIM_RESOURCE_PATH="/models:/usr/share/gz/garden/models:${GZ_SIM_RESOURCE_PATH:-}"
ls -lR /models
ls -lR /usr/share/gz/garden/models

echo "[entrypoint] PATH=$PATH"
echo "[entrypoint] which px4: $(which px4 || echo not found)"
echo "[entrypoint] which nc: $(which nc || echo not found)"
export PATH="/PX4-Autopilot/build/px4_sitl_default/bin:/usr/bin:$PATH"
echo "[entrypoint] PATH(after export)=$PATH"
echo "[entrypoint] which px4 (after export): $(which px4 || echo not found)"
echo "[entrypoint] which nc (after export): $(which nc || echo not found)"

if ! command -v nc >/dev/null 2>&1; then
  echo "[entrypoint] nc not found, trying busybox nc..."
  if command -v busybox >/dev/null 2>&1; then
    alias nc="busybox nc"
    echo "[entrypoint] using busybox nc"
  else
    echo "[entrypoint] installing busybox for nc fallback..."
    apt-get update && apt-get install -y busybox-static && alias nc="busybox nc"
  fi
fi

gz sim -r /usr/share/ignition/gazebo/worlds/empty.sdf --headless-rendering &
sleep 3

# ---------- model spawn ----------
if [[ -f "${MODEL_PATH}/model.sdf" ]]; then
  gz service -s /world/empty/create --reqtype ignition.msgs.EntityFactory \
      --reptype ignition.msgs.Boolean --timeout 300 \
      --req "sdf_filename: \"${MODEL_PATH}/model.sdf\" pose: { position:{z:0.25} }"
else
  echo "[entrypoint] model.sdf not found – skipping insertion"
fi

# ---------- PX4 ----------
px4 -i 0 -d -s /PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/rcS -w build/px4_sitl_rtps &
until nc -z localhost 11345; do sleep 1; done
echo "[entrypoint] PX4 RTPS ready."

# --- enable logging of servo outputs (fans) ---
printf 'param set SDLOG_PROFILE 1\nparam set SDLOG_MODE 1\n' | nc -u -w1 localhost 14556

tail -F /dev/null
