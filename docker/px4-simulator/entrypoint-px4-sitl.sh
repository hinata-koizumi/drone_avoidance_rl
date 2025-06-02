#!/bin/bash
set -e

echo "[Entrypoint] PWD: $(pwd)"
echo "[Entrypoint] Listing /PX4-Autopilot/build/px4_sitl_default/bin:"
ls -l /PX4-Autopilot/build/px4_sitl_default/bin || echo "[Entrypoint] PX4 bin dir not found"

MODEL=${1:-"none"}
PARAMS=${2:-""}

if [ ! -f ./build/px4_sitl_default/bin/px4 ]; then
    echo "[Entrypoint] PX4 binary not found: ./build/px4_sitl_default/bin/px4" >&2
    exit 2
fi

echo "[Entrypoint] Launching PX4: ./build/px4_sitl_default/bin/px4 -d $PARAMS ${MODEL:+--model $MODEL}"

if [ "$MODEL" = "none" ]; then
    exec ./build/px4_sitl_default/bin/px4 -d $PARAMS
else
    exec ./build/px4_sitl_default/bin/px4 -d $PARAMS --model $MODEL
fi 