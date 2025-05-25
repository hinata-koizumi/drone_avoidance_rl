import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


def test_tilt_topic() -> None:
    rclpy.init()
    node = Node("tilt_check")
    msgs = []
    def cb(msg: Float64) -> None: msgs.append(msg)
    fan1_topic = node.declare_parameter("fan1_topic", "/servo/fan1_tilt").value
    fan2_topic = node.declare_parameter("fan2_topic", "/servo/fan2_tilt").value
    node.create_subscription(Float64, fan1_topic, cb, 10)
    node.create_subscription(Float64, fan2_topic, cb, 10)

    start = time.time()
    while time.time() - start < 10.0 and len(msgs) < 4:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()
    assert len(msgs) >= 4, f"Tilt msgs received: {len(msgs)}"
