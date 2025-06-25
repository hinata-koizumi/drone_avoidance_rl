#!/bin/bash

# drone_manual_controlテスト実行スクリプト

set -e

echo "=== Starting drone_manual_control tests ==="

# ディレクトリの確認
if [ ! -d "src/manual_control" ]; then
    echo "ERROR: manual_control package not found"
    exit 1
fi

# 設定ファイルの確認
if [ ! -f "config/action_sequences.yaml" ]; then
    echo "ERROR: action_sequences.yaml not found"
    exit 1
fi

if [ ! -f "config/drone_specs.yaml" ]; then
    echo "ERROR: drone_specs.yaml not found"
    exit 1
fi

echo "=== Configuration files found ==="

# 単体テスト実行
echo "=== Running unit tests ==="
python3 -m pytest tests/test_manual_control.py -v --tb=short

# 統合テスト実行
echo "=== Running integration tests ==="
python3 -m pytest tests/test_integration.py -v --tb=short

# パッケージ構造テスト
echo "=== Testing package structure ==="
python3 -c "
import os
import sys
sys.path.insert(0, 'src')
from manual_control.action_executor import ActionExecutorNode, ActionType, ActionSequence
print('✓ Package structure is valid')
"

# 設定ファイル読み込みテスト
echo "=== Testing configuration loading ==="
python3 -c "
import yaml
import os
with open('config/action_sequences.yaml', 'r') as f:
    data = yaml.safe_load(f)
print(f'✓ Loaded {len(data.get(\"action_sequences\", []))} action sequences')
"

# ビルドテスト（シミュレーション）
echo "=== Testing build configuration ==="
python3 -c "
import os
required_files = [
    'src/manual_control/package.xml',
    'src/manual_control/setup.py',
    'src/manual_control/manual_control/action_executor.py'
]
for file_path in required_files:
    if not os.path.exists(file_path):
        raise FileNotFoundError(f'Required file not found: {file_path}')
print('✓ All required build files found')
"

echo "=== All tests completed successfully ===" 