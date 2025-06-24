"""
Common utilities test suite for local GitHub Actions testing.
Tests the common utility functions and base classes.
"""
import pytest
import numpy as np

# Import common utilities
from src.common.common.utils import clamp
from src.common.common.bridge_base import BridgeBase


class TestCommonUtils:
    """Test suite for common utility functions."""
    
    def test_clamp_basic(self):
        """Test basic clamping functionality."""
        # Test within bounds
        assert clamp(5, 0, 10) == 5
        assert clamp(0, 0, 10) == 0
        assert clamp(10, 0, 10) == 10
        
        # Test below lower bound
        assert clamp(-5, 0, 10) == 0
        assert clamp(0.5, 1, 10) == 1
        
        # Test above upper bound
        assert clamp(15, 0, 10) == 10
        assert clamp(9.5, 0, 9) == 9
    
    def test_clamp_float(self):
        """Test clamping with float values."""
        assert clamp(3.14, 0.0, 5.0) == 3.14
        assert clamp(-1.5, 0.0, 5.0) == 0.0
        assert clamp(6.7, 0.0, 5.0) == 5.0
    
    def test_clamp_edge_cases(self):
        """Test clamping edge cases."""
        # Same min and max
        assert clamp(5, 10, 10) == 10
        assert clamp(15, 10, 10) == 10
        
        # Negative ranges
        assert clamp(0, -10, -5) == -5
        assert clamp(-15, -10, -5) == -10
        assert clamp(-7, -10, -5) == -7


class TestBridgeBase:
    """Test suite for BridgeBase class."""
    
    def test_bridge_base_creation(self):
        """Test BridgeBase can be created with parameters."""
        params = {
            'input_topic': '/test/input',
            'output_topic': '/test/output',
            'qos_depth': 10,
            'qos_reliability': 'reliable',
            'qos_history': 'keep_last',
            'log_level': 'info'
        }
        
        try:
            bridge = BridgeBase('test_bridge', params)
            assert bridge is not None
            assert bridge.get_name() == 'test_bridge'
            bridge.destroy_node()
        except Exception as e:
            pytest.fail(f"BridgeBase creation failed: {e}")
    
    def test_bridge_base_parameters(self):
        """Test BridgeBase parameter handling."""
        params = {
            'input_topic': '/test/input',
            'output_topic': '/test/output',
            'qos_depth': 5,
            'qos_reliability': 'best_effort',
            'qos_history': 'keep_last',
            'log_level': 'debug'
        }
        
        try:
            bridge = BridgeBase('test_bridge', params)
            
            # Check parameters are set correctly
            input_topic = bridge.get_parameter('input_topic').get_parameter_value().string_value
            output_topic = bridge.get_parameter('output_topic').get_parameter_value().string_value
            qos_depth = bridge.get_parameter('qos_depth').value
            log_level = bridge.get_parameter('log_level').get_parameter_value().string_value
            
            assert input_topic == '/test/input'
            assert output_topic == '/test/output'
            assert qos_depth == 5
            assert log_level == 'debug'
            
            bridge.destroy_node()
        except Exception as e:
            pytest.fail(f"BridgeBase parameter test failed: {e}")
    
    def test_bridge_base_qos_profile(self):
        """Test BridgeBase QoS profile creation."""
        params = {
            'input_topic': '/test/input',
            'output_topic': '/test/output',
            'qos_depth': 10,
            'qos_reliability': 'reliable',
            'qos_history': 'keep_last',
            'log_level': 'info'
        }
        
        try:
            bridge = BridgeBase('test_bridge', params)
            
            # Check QoS profile attributes
            assert hasattr(bridge, 'qos_profile')
            assert bridge.qos_profile.depth == 10
            
            bridge.destroy_node()
        except Exception as e:
            pytest.fail(f"BridgeBase QoS profile test failed: {e}")


if __name__ == "__main__":
    pytest.main([__file__]) 