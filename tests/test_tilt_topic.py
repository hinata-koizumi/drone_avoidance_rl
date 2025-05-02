import rclpy, time, pytest
from rclpy.node import Node
from std_msgs.msg import Float64

def test_tilt_topic():
    rclpy.init()
    node = Node("tilt_check")
    msgs = []
    def cb(msg): msgs.append(msg)
    sub1 = node.create_subscription(Float64, "/servo/fan1_tilt", cb, 10)
    sub2 = node.create_subscription(Float64, "/servo/fan2_tilt", cb, 10)

    start = time.time()
    while time.time() - start < 10.0 and len(msgs) < 4:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node(); rclpy.shutdown()
    assert len(msgs) >= 4, f"Tilt msgs received: {len(msgs)}"
