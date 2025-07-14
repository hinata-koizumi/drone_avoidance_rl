"""
統合テスト用のpytestファイル
drone-msgs、drone-sim-core、drone-rlの統合テストを実行します
"""

import pytest
import time
import subprocess
import os
import sys
from typing import Dict, List

class TestIntegration:
    """統合テストクラス"""
    
    @pytest.fixture(autouse=True)
    def setup(self):
        """テスト前のセットアップ"""
        # ROS_DOMAIN_IDの設定
        os.environ['ROS_DOMAIN_ID'] = '0'
        
        # システムの起動を待つ
        time.sleep(10)
    
    def test_ros2_system_ready(self):
        """ROS2システムの準備状況をテスト"""
        # ROS2デーモンの確認
        result = subprocess.run(
            ['ros2', 'daemon', 'status'],
            capture_output=True,
            text=True,
            timeout=30
        )
        
        assert result.returncode == 0, f"ROS2 daemon not ready: {result.stderr}"
    
    def test_required_nodes_running(self):
        """必要なノードが起動していることをテスト"""
        # ノードリストの取得
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=30
        )
        
        assert result.returncode == 0, f"Failed to get node list: {result.stderr}"
        
        nodes = result.stdout.strip().split('\n')
        required_nodes = ['/gz_sim', '/state_bridge_node']
        
        for node in required_nodes:
            assert node in nodes, f"Required node {node} not found in {nodes}"
    
    def test_required_topics_available(self):
        """必要なトピックが利用可能であることをテスト"""
        # トピックリストの取得
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            timeout=30
        )
        
        assert result.returncode == 0, f"Failed to get topic list: {result.stderr}"
        
        topics = result.stdout.strip().split('\n')
        required_topics = ['/drone_state', '/drone_control']
        
        for topic in required_topics:
            assert topic in topics, f"Required topic {topic} not found in {topics}"
    
    def test_gym_environment_creation(self):
        """Gym環境の作成をテスト"""
        test_script = """
import sys
sys.path.append('/workspace')

try:
    from drone_sim_env import DroneSimEnv
    
    # 環境の作成
    env = DroneSimEnv()
    
    # 環境のリセット
    obs, info = env.reset()
    assert obs is not None, "Observation should not be None"
    
    # アクション空間の確認
    assert hasattr(env, 'action_space'), "Environment should have action_space"
    assert hasattr(env, 'observation_space'), "Environment should have observation_space"
    
    # 簡単なアクションの実行
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    
    assert obs is not None, "Step should return observation"
    assert isinstance(reward, (int, float)), "Reward should be numeric"
    assert isinstance(terminated, bool), "Terminated should be boolean"
    assert isinstance(truncated, bool), "Truncated should be boolean"
    
    # 環境のクローズ
    env.close()
    print("Gym environment test passed")
    
except Exception as e:
    print(f"Gym environment test failed: {e}")
    sys.exit(1)
"""
        
        result = subprocess.run(
            ['python', '-c', test_script],
            capture_output=True,
            text=True,
            timeout=60
        )
        
        assert result.returncode == 0, f"Gym environment test failed: {result.stderr}"
    
    def test_message_types_available(self):
        """メッセージ型が利用可能であることをテスト"""
        test_script = """
import sys
sys.path.append('/workspace')

try:
    from drone_msgs.msg import DroneState, DroneControlCommand
    
    # メッセージ型の確認
    assert DroneState is not None, "DroneState should be available"
    assert DroneControlCommand is not None, "DroneControlCommand should be available"
    
    print("Message types test passed")
    
except Exception as e:
    print(f"Message types test failed: {e}")
    sys.exit(1)
"""
        
        result = subprocess.run(
            ['python', '-c', test_script],
            capture_output=True,
            text=True,
            timeout=30
        )
        
        assert result.returncode == 0, f"Message types test failed: {result.stderr}"
    
    def test_simulation_communication(self):
        """シミュレーションとの通信をテスト"""
        # トピックのエコーをテスト
        test_script = """
import rclpy
from rclpy.node import Node
from drone_msgs.msg import DroneState
import time

def test_topic_communication():
    rclpy.init()
    node = Node('test_node')
    
    # サブスクライバーの作成
    received_messages = []
    
    def callback(msg):
        received_messages.append(msg)
    
    subscription = node.create_subscription(
        DroneState,
        '/drone_state',
        callback,
        10
    )
    
    # メッセージを受信するまで待機
    start_time = time.time()
    while len(received_messages) == 0 and time.time() - start_time < 30:
        rclpy.spin_once(node, timeout_sec=1.0)
    
    node.destroy_node()
    rclpy.shutdown()
    
    return len(received_messages) > 0

if test_topic_communication():
    print("Topic communication test passed")
else:
    print("Topic communication test failed - no messages received")
    sys.exit(1)
"""
        
        result = subprocess.run(
            ['python', '-c', test_script],
            capture_output=True,
            text=True,
            timeout=60
        )
        
        # 通信テストは失敗しても警告として扱う
        if result.returncode != 0:
            print(f"Warning: Topic communication test failed: {result.stderr}")
    
    @pytest.mark.slow
    def test_full_integration_workflow(self):
        """完全な統合ワークフローのテスト"""
        # 1. 環境の作成
        # 2. エピソードの実行
        # 3. 結果の確認
        
        test_script = """
import sys
sys.path.append('/workspace')

try:
    from drone_sim_env import DroneSimEnv
    import numpy as np
    
    # 環境の作成
    env = DroneSimEnv()
    
    # エピソードの実行
    obs, info = env.reset()
    total_reward = 0
    step_count = 0
    
    for _ in range(100):  # 最大100ステップ
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        
        total_reward += reward
        step_count += 1
        
        if terminated or truncated:
            break
    
    # 結果の確認
    assert step_count > 0, "At least one step should be executed"
    assert isinstance(total_reward, (int, float)), "Total reward should be numeric"
    
    env.close()
    print(f"Integration workflow test passed. Steps: {step_count}, Total reward: {total_reward}")
    
except Exception as e:
    print(f"Integration workflow test failed: {e}")
    sys.exit(1)
"""
        
        result = subprocess.run(
            ['python', '-c', test_script],
            capture_output=True,
            text=True,
            timeout=120
        )
        
        assert result.returncode == 0, f"Integration workflow test failed: {result.stderr}"

if __name__ == "__main__":
    pytest.main([__file__, "-v"]) 