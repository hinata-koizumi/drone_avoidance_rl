#!/usr/bin/env bash
# Validate the manual control stack: build, launch, health-check, tear down
# Usage: ./scripts/validate_manual_control.sh
set -euo pipefail

STACK_FILE="docker-compose.manual_control.yaml"
WAIT_TIME=${WAIT_TIME:-20}

bold() { echo -e "\033[1m$1\033[0m"; }

bold "[1/4] Building manual control stack images…"
docker compose -f "$STACK_FILE" build --pull

bold "[2/4] Starting manual control stack…"
docker compose -f "$STACK_FILE" up -d

cleanup() {
  bold "[4/4] Stopping and removing containers…"
  docker compose -f "$STACK_FILE" down -v --remove-orphans
}
trap cleanup EXIT

bold "[3/4] Waiting $WAIT_TIME s for services to initialize…"
sleep "$WAIT_TIME"

# ── Health checks ────────────────────────────────────────────────────────────

bold "→ Checking ROS 2 topics are published…"
ROS_TOPICS=$(docker compose -f "$STACK_FILE" exec -T manual_control bash -c "source /opt/ros/humble/setup.bash && ros2 topic list")
[[ "$ROS_TOPICS" == *"/drone/pose"* ]] || { echo "ROS 2 topics missing"; exit 1; }

bold "→ Checking Web UI is reachable…"
curl -fsSL http://localhost:8080 > /dev/null

bold "✅ Manual control validation PASSED" 