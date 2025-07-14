#!/bin/bash
"""
統合テスト環境の設定確認スクリプト
"""

set -e

echo "Checking integration test environment..."

# 色付き出力の設定
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# チェック関数
check_command() {
    local cmd=$1
    local name=$2
    
    if command -v $cmd &> /dev/null; then
        echo -e "${GREEN}✓${NC} $name is installed"
        return 0
    else
        echo -e "${RED}✗${NC} $name is not installed"
        return 1
    fi
}

check_file() {
    local file=$1
    local name=$2
    
    if [ -f "$file" ]; then
        echo -e "${GREEN}✓${NC} $name exists"
        return 0
    else
        echo -e "${RED}✗${NC} $name does not exist"
        return 1
    fi
}

check_directory() {
    local dir=$1
    local name=$2
    
    if [ -d "$dir" ]; then
        echo -e "${GREEN}✓${NC} $name exists"
        return 0
    else
        echo -e "${RED}✗${NC} $name does not exist"
        return 1
    fi
}

# 結果カウンター
passed=0
failed=0

echo "=== System Requirements ==="

# システムコマンドの確認
check_command "docker" "Docker" && ((passed++)) || ((failed++))
check_command "docker" "Docker Compose" && ((passed++)) || ((failed++))
check_command "git" "Git" && ((passed++)) || ((failed++))
check_command "python3" "Python 3" && ((passed++)) || ((failed++))

echo ""
echo "=== Repository Structure ==="

# ディレクトリ構造の確認
check_directory "../drone-msgs" "drone-msgs repository" && ((passed++)) || ((failed++))
check_directory "../drone-sim-core" "drone-sim-core repository" && ((passed++)) || ((failed++))
check_directory "../drone-rl" "drone-rl repository" && ((passed++)) || ((failed++))
check_directory "tests" "tests directory" && ((passed++)) || ((failed++))
check_directory "scripts" "scripts directory" && ((passed++)) || ((failed++))

echo ""
echo "=== Configuration Files ==="

# 設定ファイルの確認
check_file "docker-compose.yml" "docker-compose.yml" && ((passed++)) || ((failed++))
check_file "requirements-test.txt" "requirements-test.txt" && ((passed++)) || ((failed++))
check_file "Dockerfile.test" "Dockerfile.test" && ((passed++)) || ((failed++))
check_file "tests/run_integration_tests.py" "run_integration_tests.py" && ((passed++)) || ((failed++))
check_file "tests/test_integration.py" "test_integration.py" && ((passed++)) || ((failed++))

echo ""
echo "=== Docker Images ==="

# Dockerイメージの確認
if docker images | grep -q "drone-msgs"; then
    echo -e "${GREEN}✓${NC} drone-msgs Docker image exists"
    ((passed++))
else
    echo -e "${YELLOW}⚠${NC} drone-msgs Docker image not found (will be built)"
    ((passed++))
fi

if docker images | grep -q "drone-sim-core"; then
    echo -e "${GREEN}✓${NC} drone-sim-core Docker image exists"
    ((passed++))
else
    echo -e "${YELLOW}⚠${NC} drone-sim-core Docker image not found (will be built)"
    ((passed++))
fi

if docker images | grep -q "drone-rl"; then
    echo -e "${GREEN}✓${NC} drone-rl Docker image exists"
    ((passed++))
else
    echo -e "${YELLOW}⚠${NC} drone-rl Docker image not found (will be built)"
    ((passed++))
fi

echo ""
echo "=== Environment Variables ==="

# 環境変数の確認
if [ -n "$ROS_DOMAIN_ID" ]; then
    echo -e "${GREEN}✓${NC} ROS_DOMAIN_ID is set: $ROS_DOMAIN_ID"
    ((passed++))
else
    echo -e "${YELLOW}⚠${NC} ROS_DOMAIN_ID is not set (will use default: 0)"
    export ROS_DOMAIN_ID=0
    ((passed++))
fi

if [ -n "$PYTHONPATH" ]; then
    echo -e "${GREEN}✓${NC} PYTHONPATH is set: $PYTHONPATH"
    ((passed++))
else
    echo -e "${YELLOW}⚠${NC} PYTHONPATH is not set"
    ((passed++))
fi

echo ""
echo "=== Summary ==="
echo "Passed checks: $passed"
echo "Failed checks: $failed"
echo "Total checks: $((passed + failed))"

if [ $failed -eq 0 ]; then
    echo -e "${GREEN}✓${NC} All checks passed! Environment is ready for integration tests."
    exit 0
else
    echo -e "${RED}✗${NC} Some checks failed. Please fix the issues before running integration tests."
    exit 1
fi 