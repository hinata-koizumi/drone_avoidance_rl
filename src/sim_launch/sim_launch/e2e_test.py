import rclpy
from rclpy.node import Node

class SmokeTest(Node):
    def __init__(self):
        super().__init__('smoke_test')
        self.create_timer(1.0, self.check)

    def check(self):
        topics = self.get_topic_names_and_types()
        assert any('/clock' in t for t, _ in topics), "/clock not found"
        self.get_logger().info("E2E test passed.")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = SmokeTest()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
