#!/bin/bash
set -e

echo "[Entrypoint] PWD: $(pwd)"
echo "[Entrypoint] ENV:"
env

echo "[Entrypoint] Directory tree from /PX4-Autopilot:"
find /PX4-Autopilot | head -n 100

PX4_BIN=/PX4-Autopilot/build/px4_sitl_default/bin/px4
echo "[Entrypoint] Checking PX4 binary: $PX4_BIN"
ls -l $PX4_BIN || { echo "[Entrypoint] PX4 binary not found!"; sleep 10; exit 2; }

MODEL=${1:-"none"}
PARAMS=${2:-""}

echo "[Entrypoint] Launching PX4: $PX4_BIN -d $PARAMS ${MODEL:+--model $MODEL}"
sleep 2

if [ "$MODEL" = "none" ]; then
    exec $PX4_BIN -d $PARAMS
else
    exec $PX4_BIN -d $PARAMS --model $MODEL
fi 