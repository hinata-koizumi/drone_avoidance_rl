#!/usr/bin/env bash
set -euxo pipefail

# 1. Dockerイメージを最新化

docker compose build --no-cache

# 2. rosdepローカルYAML反映
bash tools/setup_rosdep_local.sh

# 3. E2E/統合テスト（CI完全再現）
docker compose -f tests/ci-compose.yml up --abort-on-container-exit

echo "[test_all.sh] E2Eテスト完了"

# 4. Gym APIテスト・静的解析（CI完全再現）
# Python 3.10系推奨（pyenvで合わせておくこと）
python3 -m pip install --upgrade pip
python3 -m pip install pytest gymnasium numpy pyyaml lark ruff mypy
cd src
PYTHONPATH=$PYTHONPATH:$(pwd) pytest ../tests/test_gym_api.py
ruff src/ ../tests/
mypy src/ ../tests/
cd ..
echo "[test_all.sh] Gym API/静的解析テスト完了" 