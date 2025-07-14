#!/usr/bin/env python3
"""
統合テスト実行スクリプト
drone-msgs、drone-sim-core、drone-rlの統合テストを実行します
"""

import os
import sys
import time
import subprocess
import requests
import json
import logging
import argparse
from typing import Dict, List, Optional
from dataclasses import dataclass
from datetime import datetime

# ログ設定
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

@dataclass
class TestResult:
    """テスト結果を格納するデータクラス"""
    test_name: str
    status: str  # 'PASS', 'FAIL', 'SKIP'
    duration: float
    error_message: Optional[str] = None
    details: Optional[Dict] = None

class IntegrationTestRunner:
    """統合テスト実行クラス"""
    
    def __init__(self, quick_mode: bool = False):
        self.results: List[TestResult] = []
        self.ros_domain_id = os.getenv('ROS_DOMAIN_ID', '0')
        self.quick_mode = quick_mode
        
    def run_test(self, test_name: str, test_func, timeout: int = 300) -> TestResult:
        """個別テストを実行"""
        start_time = time.time()
        try:
            logger.info(f"Running test: {test_name}")
            test_func()
            duration = time.time() - start_time
            return TestResult(test_name, 'PASS', duration)
        except Exception as e:
            duration = time.time() - start_time
            logger.error(f"Test {test_name} failed: {str(e)}")
            return TestResult(test_name, 'FAIL', duration, str(e))
    
    def test_ros2_nodes(self) -> None:
        """ROS2ノードの動作確認"""
        try:
            # ノードリストの取得
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode != 0:
                raise Exception(f"Failed to get ROS2 nodes: {result.stderr}")
            
            nodes = result.stdout.strip().split('\n')
            logger.info(f"Found ROS2 nodes: {nodes}")
            
            # 必要なノードの確認
            required_nodes = ['/gz_sim', '/state_bridge_node']
            for node in required_nodes:
                if node not in nodes:
                    raise Exception(f"Required node {node} not found")
                    
        except subprocess.TimeoutExpired:
            raise Exception("Timeout while checking ROS2 nodes")
    
    def test_ros2_topics(self) -> None:
        """ROS2トピックの動作確認"""
        try:
            # トピックリストの取得
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode != 0:
                raise Exception(f"Failed to get ROS2 topics: {result.stderr}")
            
            topics = result.stdout.strip().split('\n')
            logger.info(f"Found ROS2 topics: {topics}")
            
            # 必要なトピックの確認
            required_topics = ['/drone_state', '/drone_control']
            for topic in required_topics:
                if topic not in topics:
                    raise Exception(f"Required topic {topic} not found")
                    
        except subprocess.TimeoutExpired:
            raise Exception("Timeout while checking ROS2 topics")
    
    def test_gym_environment(self) -> None:
        """Gym環境の動作確認"""
        try:
            # PythonスクリプトでGym環境をテスト
            test_script = """
import sys
sys.path.append('/workspace')

try:
    from drone_sim_env import DroneSimEnv
    import gymnasium as gym
    
    # 環境の作成
    env = DroneSimEnv()
    
    # 環境のリセット
    obs, info = env.reset()
    print(f"Environment reset successful. Observation shape: {obs.shape if hasattr(obs, 'shape') else type(obs)}")
    
    # 簡単なアクションの実行
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    print(f"Step successful. Reward: {reward}")
    
    # 環境のクローズ
    env.close()
    print("Environment test completed successfully")
    
except Exception as e:
    print(f"Environment test failed: {e}")
    sys.exit(1)
"""
            
            result = subprocess.run(
                ['python', '-c', test_script],
                capture_output=True,
                text=True,
                timeout=60
            )
            
            if result.returncode != 0:
                raise Exception(f"Gym environment test failed: {result.stderr}")
                
            logger.info("Gym environment test passed")
            
        except subprocess.TimeoutExpired:
            raise Exception("Timeout while testing Gym environment")
    
    def test_docker_services(self) -> None:
        """Dockerサービスの動作確認"""
        try:
            # Docker Composeサービスの状態確認
            result = subprocess.run(
                ['docker-compose', 'ps'],
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode != 0:
                raise Exception(f"Failed to check Docker services: {result.stderr}")
            
            services_output = result.stdout
            logger.info(f"Docker services status:\n{services_output}")
            
            # 全てのサービスが起動しているか確認
            if 'Up' not in services_output:
                raise Exception("Not all services are running")
                
        except subprocess.TimeoutExpired:
            raise Exception("Timeout while checking Docker services")
    
    def test_basic_system_health(self) -> None:
        """基本的なシステムヘルスチェック"""
        try:
            # 基本的なシステムコマンドの確認
            commands = [
                ['docker', '--version'],
                ['docker-compose', '--version'],
                ['python', '--version'],
                ['git', '--version']
            ]
            
            for cmd in commands:
                result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                
                if result.returncode != 0:
                    raise Exception(f"Command {cmd[0]} failed: {result.stderr}")
                    
            logger.info("Basic system health check passed")
            
        except subprocess.TimeoutExpired:
            raise Exception("Timeout while checking system health")
    
    def test_file_structure(self) -> None:
        """ファイル構造の確認"""
        try:
            required_files = [
                'docker-compose.yml',
                'requirements-test.txt',
                'Dockerfile.test',
                'tests/run_integration_tests.py',
                'tests/test_integration.py'
            ]
            
            for file_path in required_files:
                if not os.path.exists(file_path):
                    raise Exception(f"Required file {file_path} not found")
                    
            logger.info("File structure check passed")
            
        except Exception as e:
            raise Exception(f"File structure check failed: {str(e)}")
    
    def generate_report(self) -> None:
        """テスト結果レポートの生成"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 結果ディレクトリのパスを環境に応じて設定
        if os.path.exists("/workspace"):
            results_dir = "/workspace/results"
        else:
            results_dir = "results"
        
        report_file = f"{results_dir}/integration_test_report_{timestamp}.json"
        
        # 結果ディレクトリの作成
        os.makedirs(results_dir, exist_ok=True)
        
        # レポートデータの準備
        report_data = {
            "timestamp": timestamp,
            "quick_mode": self.quick_mode,
            "total_tests": len(self.results),
            "passed": len([r for r in self.results if r.status == 'PASS']),
            "failed": len([r for r in self.results if r.status == 'FAIL']),
            "results": [
                {
                    "test_name": r.test_name,
                    "status": r.status,
                    "duration": r.duration,
                    "error_message": r.error_message
                }
                for r in self.results
            ]
        }
        
        # JSONファイルに保存
        with open(report_file, 'w') as f:
            json.dump(report_data, f, indent=2)
        
        logger.info(f"Test report saved to: {report_file}")
        
        # サマリーの表示
        logger.info("=" * 50)
        logger.info("INTEGRATION TEST SUMMARY")
        logger.info("=" * 50)
        logger.info(f"Mode: {'Quick' if self.quick_mode else 'Full'}")
        logger.info(f"Total tests: {report_data['total_tests']}")
        logger.info(f"Passed: {report_data['passed']}")
        logger.info(f"Failed: {report_data['failed']}")
        logger.info("=" * 50)
        
        # 失敗したテストの詳細表示
        failed_tests = [r for r in self.results if r.status == 'FAIL']
        if failed_tests:
            logger.error("FAILED TESTS:")
            for test in failed_tests:
                logger.error(f"  - {test.test_name}: {test.error_message}")
        
        # 終了コードの設定
        if failed_tests:
            sys.exit(1)
        else:
            sys.exit(0)

def main():
    """メイン実行関数"""
    parser = argparse.ArgumentParser(description='Integration Test Runner')
    parser.add_argument('--quick', action='store_true', help='Run quick tests only')
    args = parser.parse_args()
    
    logger.info("Starting integration tests...")
    logger.info(f"Quick mode: {args.quick}")
    
    runner = IntegrationTestRunner(quick_mode=args.quick)
    
    # テストの実行
    if args.quick:
        # クイックテスト（基本的なチェックのみ）
        tests = [
            ("Basic System Health", runner.test_basic_system_health, 30),
            ("File Structure", runner.test_file_structure, 30),
        ]
    else:
        # 完全なテスト
        tests = [
            ("Docker Services", runner.test_docker_services, 60),
            ("ROS2 Nodes", runner.test_ros2_nodes, 120),
            ("ROS2 Topics", runner.test_ros2_topics, 120),
            ("Gym Environment", runner.test_gym_environment, 180),
        ]
    
    for test_name, test_func, timeout in tests:
        result = runner.run_test(test_name, test_func, timeout)
        runner.results.append(result)
    
    # レポートの生成
    runner.generate_report()

if __name__ == "__main__":
    main() 