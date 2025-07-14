#!/bin/bash
# Local test runner script for drone_avoidance_rl
# This script runs all tests locally to verify GitHub Actions compatibility

set -e

echo "=== Running Local Tests for drone_avoidance_rl ==="
echo "Date: $(date)"
echo "Working directory: $(pwd)"

# Check if we're in the right directory
if [ ! -f "src/drone_sim_env.py" ]; then
    echo "ERROR: Not in drone_avoidance_rl root directory"
    exit 1
fi

# Create test results directory
mkdir -p test_results

echo "=== 1. Testing Bridge Nodes ==="
python3 -m pytest tests/test_bridge_nodes.py -v --tb=short > test_results/bridge_nodes.log 2>&1
if [ $? -eq 0 ]; then
    echo "✓ Bridge nodes tests passed"
else
    echo "✗ Bridge nodes tests failed"
    cat test_results/bridge_nodes.log
fi

echo "=== 2. Testing Simulation Launch ==="
python3 -m pytest tests/test_simulation_launch.py -v --tb=short > test_results/simulation_launch.log 2>&1
if [ $? -eq 0 ]; then
    echo "✓ Simulation launch tests passed"
else
    echo "✗ Simulation launch tests failed"
    cat test_results/simulation_launch.log
fi

echo "=== 3. Testing Sample Agents ==="
python3 -m pytest tests/test_sample_agents.py -v --tb=short > test_results/sample_agents.log 2>&1
if [ $? -eq 0 ]; then
    echo "✓ Sample agents tests passed"
else
    echo "✗ Sample agents tests failed"
    cat test_results/sample_agents.log
fi

echo "=== 4. Testing Common Utils ==="
python3 -m pytest tests/test_common_utils.py -v --tb=short > test_results/common_utils.log 2>&1
if [ $? -eq 0 ]; then
    echo "✓ Common utils tests passed"
else
    echo "✗ Common utils tests failed"
    cat test_results/common_utils.log
fi

echo "=== 5. Testing Gym API ==="
python3 -m pytest tests/test_gym_api.py -v --tb=short > test_results/gym_api.log 2>&1
if [ $? -eq 0 ]; then
    echo "✓ Gym API tests passed"
else
    echo "✗ Gym API tests failed"
    cat test_results/gym_api.log
fi

echo "=== 6. Testing Gym Environment ==="
python3 -m pytest tests/test_gym_env.py -v --tb=short > test_results/gym_env.log 2>&1
if [ $? -eq 0 ]; then
    echo "✓ Gym environment tests passed"
else
    echo "✗ Gym environment tests failed"
    cat test_results/gym_env.log
fi

echo "=== 7. Testing RL Long Run ==="
python3 -m pytest tests/test_rl_longrun.py -v --tb=short > test_results/rl_longrun.log 2>&1
if [ $? -eq 0 ]; then
    echo "✓ RL long run tests passed"
else
    echo "✗ RL long run tests failed"
    cat test_results/rl_longrun.log
fi

echo "=== 8. Running All Tests Together ==="
python3 -m pytest tests/ -v --tb=short > test_results/all_tests.log 2>&1
if [ $? -eq 0 ]; then
    echo "✓ All tests passed"
else
    echo "✗ Some tests failed"
    cat test_results/all_tests.log
fi

echo "=== Test Summary ==="
echo "Test results saved in test_results/ directory"
echo "Individual test logs:"
ls -la test_results/

echo "=== Checking Test Coverage ==="
echo "Files tested:"
find tests/ -name "test_*.py" | wc -l
echo "Total test files found"

echo "=== Local Test Run Complete ==="
echo "Date: $(date)" 