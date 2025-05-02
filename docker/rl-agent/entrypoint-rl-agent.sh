# ────────────────────────────────────────────
# docker/rl-agent/entrypoint-rl-agent.sh
# ────────────────────────────────────────────
#!/usr/bin/env bash
set -eo pipefail
set +u

source /opt/ros/humble/setup.bash
if [ -f /drone_ws/install/setup.bash ]; then
  source /drone_ws/install/setup.bash
else
  echo "[entrypoint] install/setup.bash not found." >&2
  exit 1
fi

set -u
source /opt/venv/bin/activate
echo "[entrypoint] PYTHONPATH=$PYTHONPATH"

# 短時間 CI のときは学習をスキップ
if [ "${RUN_TRAINING:-1}" = "1" ]; then
  exec python3 /drone_ws/src/train_agent.py
else
  tail -F /dev/null
fi
