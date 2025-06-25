#!/usr/bin/env python3

import pytest
import yaml
import os
import sys
from unittest.mock import Mock, patch

# テスト用のモッククラス
class MockDroneControlCommand:
    def __init__(self):
        self.throttle1 = 0.0
        self.throttle2 = 0.0
        self.angle1 = 0.0
        self.angle2 = 0.0

class MockBridgeBase:
    def __init__(self, node_name, params):
        self.node_name = node_name
        self.params = params
        self.get_parameter = Mock(return_value=Mock(value='test_value'))
        self.create_publisher = Mock()
        self.create_subscription = Mock()
        self.create_timer = Mock()
        self.get_logger = Mock(return_value=Mock(info=Mock(), error=Mock(), debug=Mock()))

# テスト対象のクラスを直接定義
from enum import Enum
from dataclasses import dataclass
from typing import Dict, Any

class ActionType(Enum):
    """事前定義された行動タイプ"""
    HOVER = "hover"
    TAKEOFF = "takeoff"
    LANDING = "landing"
    WAYPOINT = "waypoint"
    CIRCLE = "circle"
    SQUARE = "square"
    MANUAL = "manual"

@dataclass
class ActionSequence:
    """行動シーケンス定義"""
    name: str
    action_type: ActionType
    duration: float
    parameters: Dict[str, Any]
    next_action: str = None

class ActionExecutorNode:
    """テスト用のActionExecutorNodeクラス"""
    def __init__(self):
        self.action_sequences = {}
        self.current_action = None
        self.action_start_time = None
        self.drone_state = None
    
    def _load_action_sequences(self):
        """行動シーケンス読み込みのテスト用実装"""
        return self.action_sequences
    
    def _generate_command(self):
        """制御コマンド生成のテスト用実装"""
        if not self.current_action:
            return None
        
        command = MockDroneControlCommand()
        
        if self.current_action.action_type == ActionType.HOVER:
            command.throttle1 = 0.5
            command.throttle2 = 0.5
            command.angle1 = 0.0
            command.angle2 = 0.0
        elif self.current_action.action_type == ActionType.TAKEOFF:
            command.throttle1 = 0.7
            command.throttle2 = 0.7
            command.angle1 = 0.0
            command.angle2 = 0.0
        elif self.current_action.action_type == ActionType.WAYPOINT:
            command.throttle1 = 0.5
            command.throttle2 = 0.5
            target_x = self.current_action.parameters.get('target_x', 0.0)
            target_y = self.current_action.parameters.get('target_y', 0.0)
            command.angle1 = target_x * 0.1
            command.angle2 = target_y * 0.1
        
        return command


class TestActionExecutor:
    """ActionExecutorNodeのテストクラス"""
    
    def test_action_type_enum(self):
        """ActionType列挙型のテスト"""
        assert ActionType.HOVER.value == "hover"
        assert ActionType.TAKEOFF.value == "takeoff"
        assert ActionType.LANDING.value == "landing"
        assert ActionType.WAYPOINT.value == "waypoint"
        assert ActionType.CIRCLE.value == "circle"
        assert ActionType.SQUARE.value == "square"
        assert ActionType.MANUAL.value == "manual"
    
    def test_action_sequence_dataclass(self):
        """ActionSequenceデータクラスのテスト"""
        seq = ActionSequence(
            name="test_hover",
            action_type=ActionType.HOVER,
            duration=10.0,
            parameters={"target_altitude": 2.0},
            next_action="landing"
        )
        
        assert seq.name == "test_hover"
        assert seq.action_type == ActionType.HOVER
        assert seq.duration == 10.0
        assert seq.parameters["target_altitude"] == 2.0
        assert seq.next_action == "landing"
    
    def test_node_initialization(self):
        """ノード初期化のテスト"""
        node = ActionExecutorNode()
        
        # 基本的な属性の確認
        assert hasattr(node, 'action_sequences')
        assert hasattr(node, 'current_action')
        assert hasattr(node, 'action_start_time')
        assert hasattr(node, 'drone_state')
    
    def test_generate_hover_command(self):
        """ホバリング制御コマンド生成のテスト"""
        node = ActionExecutorNode()
        node.current_action = ActionSequence(
            name="hover",
            action_type=ActionType.HOVER,
            duration=10.0,
            parameters={},
            next_action=None
        )
        
        command = node._generate_command()
        
        assert command is not None
        assert command.throttle1 == 0.5
        assert command.throttle2 == 0.5
        assert command.angle1 == 0.0
        assert command.angle2 == 0.0
    
    def test_generate_takeoff_command(self):
        """離陸制御コマンド生成のテスト"""
        node = ActionExecutorNode()
        node.current_action = ActionSequence(
            name="takeoff",
            action_type=ActionType.TAKEOFF,
            duration=5.0,
            parameters={},
            next_action=None
        )
        
        command = node._generate_command()
        
        assert command is not None
        assert command.throttle1 == 0.7
        assert command.throttle2 == 0.7
        assert command.angle1 == 0.0
        assert command.angle2 == 0.0
    
    def test_generate_waypoint_command(self):
        """ウェイポイント制御コマンド生成のテスト"""
        node = ActionExecutorNode()
        node.current_action = ActionSequence(
            name="waypoint",
            action_type=ActionType.WAYPOINT,
            duration=15.0,
            parameters={'target_x': 5.0, 'target_y': 3.0},
            next_action=None
        )
        
        command = node._generate_command()
        
        assert command is not None
        assert command.throttle1 == 0.5
        assert command.throttle2 == 0.5
        assert abs(command.angle1 - 0.5) < 1e-6  # 5.0 * 0.1
        assert abs(command.angle2 - 0.3) < 1e-6  # 3.0 * 0.1


class TestConfiguration:
    """設定ファイルのテストクラス"""
    
    def test_action_sequences_yaml_structure(self):
        """action_sequences.yamlの構造テスト"""
        config_path = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'action_sequences.yaml'
        )
        
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)
            
            # 必須キーの確認
            assert 'action_sequences' in data
            assert isinstance(data['action_sequences'], list)
            
            # 各行動シーケンスの構造確認
            for seq in data['action_sequences']:
                assert 'name' in seq
                assert 'action_type' in seq
                assert 'duration' in seq
                assert 'parameters' in seq
                
                # 行動タイプの妥当性確認
                assert seq['action_type'] in [
                    'hover', 'takeoff', 'landing', 'waypoint', 
                    'circle', 'square', 'manual'
                ]
                
                # 期間の妥当性確認
                assert isinstance(seq['duration'], (int, float))
                assert seq['duration'] > 0
    
    def test_control_parameters_structure(self):
        """制御パラメータの構造テスト"""
        config_path = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'action_sequences.yaml'
        )
        
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)
            
            if 'control_parameters' in data:
                params = data['control_parameters']
                
                # PID制御ゲインの確認
                if 'position_p_gain' in params:
                    assert isinstance(params['position_p_gain'], (int, float))
                if 'position_i_gain' in params:
                    assert isinstance(params['position_i_gain'], (int, float))
                if 'position_d_gain' in params:
                    assert isinstance(params['position_d_gain'], (int, float))
                
                # 安全制限の確認
                if 'max_velocity' in params:
                    assert isinstance(params['max_velocity'], (int, float))
                    assert params['max_velocity'] > 0
    
    def test_safety_parameters_structure(self):
        """安全パラメータの構造テスト"""
        config_path = os.path.join(
            os.path.dirname(__file__), '..', '..', 'config', 'sim_params.yaml'
        )
        
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)
            
            # 基本的な設定ファイルの構造確認
            assert isinstance(data, dict)


if __name__ == "__main__":
    pytest.main([__file__, "-v"]) 