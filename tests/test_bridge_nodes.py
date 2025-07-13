"""
Bridge nodes test suite for local GitHub Actions testing.
Tests the core functionality of all bridge nodes.
"""

import pytest
import rclpy

from drone_msgs.msg import DroneControlCommand

# Import message types
from px4_msgs.msg import ActuatorMotors, VehicleOdometry
from src.angle_bridge.angle_bridge.main import AngleBridgeNode
from src.command_bridge.command_bridge.main import CommandBridgeNode
from src.outer_motor_bridge.outer_motor_bridge.main import OuterMotorBridge

# Import bridge nodes
from src.state_bridge.state_bridge.state_bridge import StateBridgeNode


class TestBridgeNodes:
    """Test suite for bridge nodes functionality."""
    
    @pytest.fixture(autouse=True)
    def setup_teardown(self):
        """Setup and teardown for each test."""
        rclpy.init()
        yield
        rclpy.shutdown()
    
    def test_state_bridge_node_creation(self):
        """Test StateBridgeNode can be created and initialized."""
        try:
            node = StateBridgeNode()
            assert node is not None
            assert node.get_name() == 'state_bridge'
            node.destroy_node()
        except Exception as e:
            pytest.fail(f"StateBridgeNode creation failed: {e}")
    
    def test_angle_bridge_node_creation(self):
        """Test AngleBridgeNode can be created and initialized."""
        try:
            node = AngleBridgeNode()
            assert node is not None
            assert node.get_name() == 'angle_bridge'
            node.destroy_node()
        except Exception as e:
            pytest.fail(f"AngleBridgeNode creation failed: {e}")
    
    def test_command_bridge_node_creation(self):
        """Test CommandBridgeNode can be created and initialized."""
        try:
            node = CommandBridgeNode()
            assert node is not None
            assert node.get_name() == 'command_bridge_node'
            node.destroy_node()
        except Exception as e:
            pytest.fail(f"CommandBridgeNode creation failed: {e}")
    
    def test_outer_motor_bridge_node_creation(self):
        """Test OuterMotorBridge can be created and initialized."""
        try:
            node = OuterMotorBridge()
            assert node is not None
            assert node.get_name() == 'outer_motor_bridge'
            node.destroy_node()
        except Exception as e:
            pytest.fail(f"OuterMotorBridge creation failed: {e}")
    
    def test_state_bridge_message_conversion(self):
        """Test StateBridgeNode message conversion logic."""
        node = StateBridgeNode()
        
        # Create test input message
        input_msg = VehicleOdometry()
        input_msg.q = [0.0, 0.0, 0.0, 1.0]  # Identity quaternion
        input_msg.position = [1.0, 2.0, 3.0]
        input_msg.velocity = [0.1, 0.2, 0.3]
        input_msg.angular_velocity = [0.01, 0.02, 0.03]
        
        # Test callback
        try:
            node._cb(input_msg)
            # If no exception, conversion is working
            assert True
        except Exception as e:
            pytest.fail(f"StateBridgeNode message conversion failed: {e}")
        finally:
            node.destroy_node()
    
    def test_angle_bridge_message_conversion(self):
        """Test AngleBridgeNode message conversion logic."""
        node = AngleBridgeNode()
        
        # Create test input message
        input_msg = DroneControlCommand()
        input_msg.angle1 = 15.0  # degrees
        input_msg.angle2 = -10.0  # degrees
        
        # Test callback
        try:
            node._cb(input_msg)
            # If no exception, conversion is working
            assert True
        except Exception as e:
            pytest.fail(f"AngleBridgeNode message conversion failed: {e}")
        finally:
            node.destroy_node()
    
    def test_command_bridge_message_conversion(self):
        """Test CommandBridgeNode message conversion logic."""
        node = CommandBridgeNode()
        
        # Create test input message
        input_msg = DroneControlCommand()
        input_msg.throttle1 = 0.5
        input_msg.throttle2 = 0.6
        input_msg.angle1 = 10.0
        input_msg.angle2 = -5.0
        
        # Test callback
        try:
            node._cb(input_msg)
            # If no exception, conversion is working
            assert True
        except Exception as e:
            pytest.fail(f"CommandBridgeNode message conversion failed: {e}")
        finally:
            node.destroy_node()
    
    def test_outer_motor_bridge_message_conversion(self):
        """Test OuterMotorBridge message conversion logic."""
        node = OuterMotorBridge()
        
        # Create test input message
        input_msg = ActuatorMotors()
        input_msg.control = [0.1, 0.2, 0.3, 0.4] + [0.0] * 4  # 8 channels
        
        # Test callback
        try:
            node._cb(input_msg)
            # If no exception, conversion is working
            assert True
        except Exception as e:
            pytest.fail(f"OuterMotorBridge message conversion failed: {e}")
        finally:
            node.destroy_node()


if __name__ == "__main__":
    pytest.main([__file__]) 