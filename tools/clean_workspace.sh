#!/usr/bin/env bash
set -euxo pipefail

# Remove Python cache and build artifacts
echo "[clean] Removing Python cache and build artifacts..."
find . -type f \( -name '*.pyc' -o -name '*.pyo' -o -name '*.log' -o -name '.DS_Store' \) -delete
find . -type d \( -name '__pycache__' -o -name '*.egg-info' -o -name '.pytest_cache' -o -name '.mypy_cache' -o -name '.nox' -o -name '.tox' \) -prune -exec rm -rf {} +

# Remove build/install/log/dist directories
rm -rf build/ install/ log/ dist/
find . -type d -name 'build' -prune -exec rm -rf {} +
find . -type d -name 'install' -prune -exec rm -rf {} +
find . -type d -name 'log' -prune -exec rm -rf {} +

# Remove .vscode/settings.json if exists
rm -f .vscode/settings.json || true

echo "[clean] Done." 