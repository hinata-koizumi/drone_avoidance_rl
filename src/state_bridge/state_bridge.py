import rclpy
from rclpy.node import Node


def main() -> None:
    rclpy.init()
    node = Node('state_bridge_node')
    rclpy.spin(node)
    rclpy.shutdown() 