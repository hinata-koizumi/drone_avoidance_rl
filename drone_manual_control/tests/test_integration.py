#!/usr/bin/env python3

import pytest
import yaml
import os
import sys
import time
import subprocess
from pathlib import Path

# テスト用の設定
TEST_TIMEOUT = 30  # 秒
NODE_STARTUP_TIMEOUT = 10  # 秒

class TestManualControlIntegration:
    """drone_manual_controlの統合テスト"""
    
    def test_config_files_exist(self):
        """設定ファイルの存在確認"""
        config_dir = Path(__file__).parent.parent / 'config'
        
        # 必須設定ファイルの確認
        required_files = [
            'action_sequences.yaml',
            'drone_specs.yaml'
        ]
        
        for file_name in required_files:
            file_path = config_dir / file_name
            assert file_path.exists(), f"Required config file not found: {file_path}"
    
    def test_action_sequences_yaml_validity(self):
        """action_sequences.yamlの妥当性テスト"""
        config_path = Path(__file__).parent.parent / 'config' / 'action_sequences.yaml'
        
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)
        
        # 基本構造の確認
        assert 'action_sequences' in data
        assert isinstance(data['action_sequences'], list)
        assert len(data['action_sequences']) > 0
        
        # 各行動シーケンスの確認
        for seq in data['action_sequences']:
            # 必須フィールドの確認
            assert 'name' in seq
            assert 'action_type' in seq
            assert 'duration' in seq
            assert 'parameters' in seq
            
            # データ型の確認
            assert isinstance(seq['name'], str)
            assert isinstance(seq['action_type'], str)
            assert isinstance(seq['duration'], (int, float))
            assert isinstance(seq['parameters'], dict)
            
            # 値の妥当性確認
            assert seq['duration'] > 0
            assert seq['action_type'] in [
                'hover', 'takeoff', 'landing', 'waypoint', 
                'circle', 'square', 'manual'
            ]
    
    def test_drone_specs_yaml_validity(self):
        """drone_specs.yamlの妥当性テスト"""
        config_path = Path(__file__).parent.parent / 'config' / 'drone_specs.yaml'
        
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)
        
        # 基本構造の確認
        assert isinstance(data, dict)
        
        # ドローン仕様の確認
        if 'drone_specs' in data:
            specs = data['drone_specs']
            assert isinstance(specs, dict)
            
            # 必須フィールドの確認
            if 'airframe_name' in specs:
                assert isinstance(specs['airframe_name'], str)
            if 'weight' in specs:
                assert isinstance(specs['weight'], (int, float))
                assert specs['weight'] > 0
            if 'max_speed' in specs:
                assert isinstance(specs['max_speed'], (int, float))
                assert specs['max_speed'] > 0
    
    def test_package_structure(self):
        """パッケージ構造の確認"""
        package_dir = Path(__file__).parent.parent / 'src' / 'manual_control'
        
        # 必須ファイルの確認
        required_files = [
            'package.xml',
            'setup.py',
            '__init__.py',
            'manual_control/__init__.py',
            'manual_control/action_executor.py',
            'resource/manual_control'
        ]
        
        for file_name in required_files:
            file_path = package_dir / file_name
            assert file_path.exists(), f"Required package file not found: {file_path}"
    
    def test_package_xml_validity(self):
        """package.xmlの妥当性テスト"""
        package_xml_path = Path(__file__).parent.parent / 'src' / 'manual_control' / 'package.xml'
        
        with open(package_xml_path, 'r') as f:
            content = f.read()
        
        # 基本的なXML構造の確認
        assert '<?xml' in content
        assert '<package' in content
        assert '<name>manual_control</name>' in content
        assert '<version>0.1.0</version>' in content
        assert '<description>' in content
        assert '<maintainer>' in content
        assert '<license>MIT</license>' in content
        
        # 依存関係の確認
        required_deps = [
            'rclpy',
            'drone_msgs',
            'px4_msgs',
            'common'
        ]
        
        for dep in required_deps:
            assert f'<depend>{dep}</depend>' in content
    
    def test_setup_py_validity(self):
        """setup.pyの妥当性テスト"""
        setup_py_path = Path(__file__).parent.parent / 'src' / 'manual_control' / 'setup.py'
        
        with open(setup_py_path, 'r') as f:
            content = f.read()
        
        # 基本的なPythonコードの確認
        assert 'from setuptools import setup' in content
        assert "name='manual_control'" in content
        assert "version='0.1.0'" in content
        assert 'action_executor = manual_control.action_executor:main' in content
    
    def test_action_executor_import(self):
        """action_executorモジュールのインポートテスト"""
        # パッケージパスを追加
        package_path = Path(__file__).parent.parent / 'src'
        sys.path.insert(0, str(package_path))
        
        try:
            # モジュールのインポートテスト
            from manual_control.action_executor import ActionExecutorNode, ActionType, ActionSequence
            
            # クラスの存在確認
            assert ActionExecutorNode is not None
            assert ActionType is not None
            assert ActionSequence is not None
            
            # ActionTypeの値確認
            assert ActionType.HOVER.value == "hover"
            assert ActionType.TAKEOFF.value == "takeoff"
            assert ActionType.LANDING.value == "landing"
            assert ActionType.WAYPOINT.value == "waypoint"
            assert ActionType.CIRCLE.value == "circle"
            assert ActionType.SQUARE.value == "square"
            assert ActionType.MANUAL.value == "manual"
            
        except ImportError as e:
            pytest.fail(f"Failed to import action_executor module: {e}")
        finally:
            # パスを元に戻す
            sys.path.pop(0)
    
    def test_action_sequence_creation(self):
        """ActionSequenceの作成テスト"""
        from manual_control.action_executor import ActionSequence, ActionType
        
        # 基本的なActionSequenceの作成
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
    
    def test_config_file_loading_simulation(self):
        """設定ファイル読み込みのシミュレーションテスト"""
        config_path = Path(__file__).parent.parent / 'config' / 'action_sequences.yaml'
        
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)
        
        # 設定ファイルからActionSequenceを作成
        sequences = {}
        for seq_data in data.get('action_sequences', []):
            from manual_control.action_executor import ActionSequence, ActionType
            
            seq = ActionSequence(
                name=seq_data['name'],
                action_type=ActionType(seq_data['action_type']),
                duration=seq_data['duration'],
                parameters=seq_data.get('parameters', {}),
                next_action=seq_data.get('next_action')
            )
            sequences[seq.name] = seq
        
        # 作成されたシーケンスの確認
        assert len(sequences) > 0
        
        # 各シーケンスの妥当性確認
        for name, seq in sequences.items():
            assert isinstance(seq.name, str)
            assert isinstance(seq.action_type, ActionType)
            assert isinstance(seq.duration, (int, float))
            assert seq.duration > 0
            assert isinstance(seq.parameters, dict)


class TestManualControlBuild:
    """ビルド関連のテスト"""
    
    def test_colcon_build_simulation(self):
        """colcon buildのシミュレーションテスト"""
        # パッケージディレクトリの確認
        package_dir = Path(__file__).parent.parent / 'src' / 'manual_control'
        assert package_dir.exists()
        
        # 必須ファイルの確認
        required_files = [
            'package.xml',
            'setup.py',
            'manual_control/action_executor.py'
        ]
        
        for file_name in required_files:
            file_path = package_dir / file_name
            assert file_path.exists(), f"Build requires file not found: {file_path}"
    
    def test_entry_point_generation(self):
        """エントリーポイント生成のテスト"""
        setup_py_path = Path(__file__).parent.parent / 'src' / 'manual_control' / 'setup.py'
        
        with open(setup_py_path, 'r') as f:
            content = f.read()
        
        # エントリーポイントの確認
        assert 'action_executor = manual_control.action_executor:main' in content
        
        # main関数の存在確認
        action_executor_path = Path(__file__).parent.parent / 'src' / 'manual_control' / 'manual_control' / 'action_executor.py'
        
        with open(action_executor_path, 'r') as f:
            content = f.read()
        
        assert 'def main()' in content
        assert 'if __name__ == "__main__":' in content


if __name__ == "__main__":
    pytest.main([__file__, "-v"]) 