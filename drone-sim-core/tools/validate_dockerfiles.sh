#!/bin/bash

# Dockerfile検証スクリプト

set -e

echo "=== Dockerfile Validation Script ==="

# プロジェクトルートディレクトリの確認
if [ ! -f "docker/rl-agent/Dockerfile" ]; then
    echo "ERROR: docker/rl-agent/Dockerfile not found"
    exit 1
fi

# entrypointファイルの存在確認
if [ ! -f "docker/rl-agent/entrypoint-rl-agent.sh" ]; then
    echo "ERROR: docker/rl-agent/entrypoint-rl-agent.sh not found"
    exit 1
fi

echo "✓ Required files found"

# Dockerfileの内容確認
echo "=== Checking Dockerfile content ==="
if grep -q "COPY docker/rl-agent/entrypoint-rl-agent.sh /entrypoint.sh" docker/rl-agent/Dockerfile; then
    echo "✓ COPY command found in Dockerfile"
else
    echo "ERROR: COPY command not found in Dockerfile"
    exit 1
fi

# PYTHONPATH設定の確認
if grep -q "ENV PYTHONPATH=/drone_ws/src:\${PYTHONPATH:-}" docker/rl-agent/Dockerfile; then
    echo "✓ PYTHONPATH environment variable set correctly"
else
    echo "WARNING: PYTHONPATH environment variable not set correctly"
fi

# entrypointファイルの実行権限確認
if [ -x "docker/rl-agent/entrypoint-rl-agent.sh" ]; then
    echo "✓ entrypoint-rl-agent.sh is executable"
else
    echo "WARNING: entrypoint-rl-agent.sh is not executable"
    chmod +x docker/rl-agent/entrypoint-rl-agent.sh
    echo "✓ Made entrypoint-rl-agent.sh executable"
fi

# ファイルサイズの確認
echo "=== File size check ==="
ls -lh docker/rl-agent/entrypoint-rl-agent.sh

echo "=== Dockerfile validation completed ===" 