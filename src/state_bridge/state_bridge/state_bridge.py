#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from src.common.bridge_base import BridgeBase

def main():
    rclpy.init()
    node = StateBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

class StateBridgeNode(BridgeBase):
    def __init__(self):
        super().__init__('state_bridge_node', {
            'input_topic': '/px4/odom',
            'output_topic': '/drone/state',
            'qos_depth': 10,
            'qos_reliability': 'reliable',
            'qos_history': 'keep_last',
            'log_level': 'debug'
        })
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.sub = self.create_subscription(
            type(None),  # TODO: 適切なmsg型に置換
            input_topic,
            self._cb,
            self.qos_profile,
        )
        self.pub = self.create_publisher(
            type(None),  # TODO: 適切なmsg型に置換
            output_topic,
            self.qos_profile,
        )
        if self.log_level == 'debug':
            self.get_logger().debug(
                f"Subscribed to: {input_topic}, Publishing to: {output_topic}, "
                f"QoS: {self.qos_profile}"
            )
    def _cb(self, msg):
        pass  # TODO: 実装

if __name__ == "__main__":
    main() 