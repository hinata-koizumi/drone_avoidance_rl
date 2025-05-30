# ────────────────────────────────────────────
# docker/rl-agent/entrypoint-rl-agent.sh
# ────────────────────────────────────────────
#!/bin/bash
set -e
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f /rl_ws/install/setup.bash ]; then
  source /rl_ws/install/setup.bash
fi
exec "$@"
