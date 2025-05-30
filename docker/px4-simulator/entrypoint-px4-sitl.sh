#!/bin/bash
set -e

MODEL=${1:-"none"}
PARAMS=${2:-""}

cd /home/px4user/PX4-Autopilot

if [ "$MODEL" = "none" ]; then
    exec ./build/px4_sitl_default/bin/px4 -d $PARAMS
else
    exec ./build/px4_sitl_default/bin/px4 -d $PARAMS --model $MODEL
fi 