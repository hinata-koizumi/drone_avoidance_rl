#!/usr/bin/env python3
"""State bridge node for drone simulation."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Vector3
from drone_msgs.msg import DroneState, DroneControlCommand


class StateBridgeNode(Node):
    """State bridge node for drone simulation."""

    def __init__(self):
        super().__init__('state_bridge_node')
        
        # Publishers
        self.state_pub = self.create_publisher(DroneState, '/drone0/state', 10)
        
        # Subscribers
        self.control_sub = self.create_subscription(
            DroneControlCommand, '/drone0/control', self.control_callback, 10)
        
        # Timer for publishing state
        self.timer = self.create_timer(0.1, self.publish_state)  # 10Hz
        
        self.get_logger().info('State bridge node started')

    def control_callback(self, msg):
        """Handle control commands."""
        self.get_logger().info(f'Received control command: {msg.control_mode}')

    def publish_state(self):
        """Publish drone state."""
        state_msg = DroneState()
        state_msg.header = Header()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.header.frame_id = 'drone0'
        
        # Set default values
        state_msg.position = Point(x=0.0, y=0.0, z=0.0)
        state_msg.orientation = Vector3(x=0.0, y=0.0, z=0.0)
        state_msg.velocity = Vector3(x=0.0, y=0.0, z=0.0)
        state_msg.angular_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        state_msg.battery_level = 1.0
        state_msg.flight_mode = 'IDLE'
        
        self.state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 