import time

import rclpy
from rclpy.node import Node

from drone_msgs.msg import DroneState

MSG_LIMIT = 5          # 5 通受信で OK
TIMEOUT   = 10.0       # 秒

def test_state_topic() -> None:
    rclpy.init()
    node = Node("state_check")
    msgs = []

    def cb(msg: DroneState) -> None:
        msgs.append(msg)

    node.create_subscription(DroneState, "/drone/state", cb, 10)

    start = time.time()
    while rclpy.ok() and time.time() - start < TIMEOUT and len(msgs) < MSG_LIMIT:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

    assert len(msgs) >= MSG_LIMIT, f"DroneState msgs received: {len(msgs)}"
