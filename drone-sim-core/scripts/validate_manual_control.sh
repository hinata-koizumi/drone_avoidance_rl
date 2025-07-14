#!/usr/bin/env bash
# Validate the manual control stack: build, launch, health-check, tear down
# Usage: ./scripts/validate_manual_control.sh
set -euo pipefail

# 統合されたdocker-compose.ymlを使用
COMPOSE_FILE="docker-compose.yml"
PROFILE="default"
WAIT_TIME=${WAIT_TIME:-20}

bold() { echo -e "\033[1m$1\033[0m"; }

bold "[1/4] Building manual control stack images…"
docker compose -f "$COMPOSE_FILE" --profile "$PROFILE" build --pull

bold "[2/4] Starting manual control stack…"
docker compose -f "$COMPOSE_FILE" --profile "$PROFILE" up -d

cleanup() {
  bold "[4/4] Stopping and removing containers…"
  docker compose -f "$COMPOSE_FILE" --profile "$PROFILE" down -v --remove-orphans
}
trap cleanup EXIT

bold "[3/4] Waiting $WAIT_TIME s for services to initialize…"
sleep "$WAIT_TIME"

# ── Health checks ────────────────────────────────────────────────────────────

bold "→ Checking ROS 2 topics are published…"
ROS_TOPICS=$(docker compose -f "$COMPOSE_FILE" --profile "$PROFILE" exec -T bridge bash -c "source /opt/ros/humble/setup.bash && ros2 topic list" 2>/dev/null || echo "")
[[ "$ROS_TOPICS" == *"/drone0/state"* ]] || { echo "ROS 2 topics missing"; exit 1; }

bold "→ Checking Web UI is reachable…"
curl -fsSL http://localhost:8080 > /dev/null || { echo "Web UI not reachable"; exit 1; }

bold "✅ Manual control validation PASSED" 