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
    
    def __init__(self):
        self.results: List[TestResult] = []
        self.ros_domain_id = os.getenv('ROS_DOMAIN_ID', '0')
        
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
    
    def generate_report(self) -> None:
        """テスト結果レポートの生成"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_file = f"/workspace/results/integration_test_report_{timestamp}.json"
        
        # 結果ディレクトリの作成
        os.makedirs("/workspace/results", exist_ok=True)
        
        # レポートデータの準備
        report_data = {
            "timestamp": timestamp,
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
    logger.info("Starting integration tests...")
    
    runner = IntegrationTestRunner()
    
    # テストの実行
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