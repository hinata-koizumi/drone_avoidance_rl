import rclpy, time
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

MSG_LIMIT = 4      # 4 通受信で合格
TIMEOUT   = 10.0   # 秒

def test_outer_pwm_topic():
    rclpy.init()
    node = Node("outer_pwm_check")
    msgs = []

    def cb(msg):
        msgs.append(msg)

    sub = node.create_subscription(
        Float32MultiArray, "/drone/outer_motor_pwm", cb, 10
    )

    start = time.time()
    while time.time() - start < TIMEOUT and len(msgs) < MSG_LIMIT:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node(); rclpy.shutdown()
    assert len(msgs) >= MSG_LIMIT, f"outer_motor_pwm msgs received: {len(msgs)}"
