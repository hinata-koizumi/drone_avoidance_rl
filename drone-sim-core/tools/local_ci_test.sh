#!/bin/bash

# ローカルCIテストスクリプト
# GitHub Actionsの各ステップを手動で実行

set -e

echo "=== ローカルCIテスト開始 ==="

# 1. 依存関係チェック
echo "1. rosdep依存関係チェック"
rosdep check --from-paths src --ignore-src --rosdistro humble || true

# 2. 静的解析
echo "2. 静的解析 (mypy)"
python3 -m pip install mypy
mypy src/ --config-file mypy.ini || true

echo "3. 静的解析 (ruff)"
python3 -m pip install ruff
ruff check src/ || true

# 4. Gym環境テスト
echo "4. Gym環境テスト"
python3 -m pip install pytest gymnasium numpy
python3 -m pytest tests/test_gym_env.py -v

# 5. Gym APIテスト
echo "5. Gym APIテスト"
python3 -m pytest tests/test_gym_api.py -v

# 6. パッケージバージョンチェック
echo "6. パッケージバージョンチェック"
./check_package_versions.sh

echo "=== ローカルCIテスト完了 ===" 