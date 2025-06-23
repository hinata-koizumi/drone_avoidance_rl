#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from drone_msgs.msg import DroneControlCommand
from px4_msgs.msg import ActuatorMotors
import rclpy
from rclpy.executors import MultiThreadedExecutor
import yaml

from src.common.bridge_base import BridgeBase


class CommandBridgeNode(BridgeBase):
    """
    Bridge node to convert DroneControlCommand to PX4 ActuatorMotors message.
    Subscribes to DroneControlCommand and publishes ActuatorMotors for PX4 actuator control.
    """
    def __init__(self) -> None:
        """
        Initialize CommandBridgeNode with parameters loaded from YAML config.
        Sets up ROS 2 subscriptions and publishers for input and output topics.
        """
        config_path = os.path.join(get_package_share_directory('sim_launch'), 'config', 'sim_params.yaml')
        with open(config_path, 'r') as f:
            params = yaml.safe_load(f)
        super().__init__('command_bridge_node', {
            'input_topic': params['input_topic'],
            'output_topic': params['output_topic'],
            'qos_depth': 10,
            'qos_reliability': 'reliable',
            'qos_history': 'keep_last',
            'log_level': 'debug'
        })
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.sub = self.create_subscription(
            DroneControlCommand,
            input_topic,
            self._cb,
            self.qos_profile,
        )
        self.pub = self.create_publisher(
            ActuatorMotors,
            output_topic,
            self.qos_profile,
        )
        if self.log_level == 'debug':
            self.get_logger().debug(
                f"Subscribed to: {input_topic}, Publishing to: {output_topic}, "
                f"QoS: {self.qos_profile}"
            )
    def _cb(self, msg: DroneControlCommand) -> None:
        """
        Callback for DroneControlCommand subscription. Converts to ActuatorMotors and publishes.
        Args:
            msg (DroneControlCommand): Incoming drone control command message.
        """
        # DroneControlCommand → ActuatorMotors 変換
        out = ActuatorMotors()
        # PX4 ActuatorMotorsは4ch想定（throttle1, throttle2, angle1, angle2を適切にマッピング）
        out.control = [msg.throttle1, msg.throttle2, msg.angle1, msg.angle2] + [0.0]*4  # 8ch分
        self.pub.publish(out)

def main() -> None:
    """
    Entry point for CommandBridgeNode. Initializes ROS 2, spins the node, and handles shutdown.
    """
    rclpy.init()
    node = CommandBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main() 