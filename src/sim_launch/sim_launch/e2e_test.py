import rclpy
from rclpy.node import Node

REQUIRED_TOPICS = [
    '/clock',
    '/servo/fan1_tilt',
    '/fmu/in/actuator_servos',
    '/drone/state',
    '/drone/outer_motor_pwm',
]

class SmokeTest(Node):
    def __init__(self):
        super().__init__('smoke_test')
        self.create_timer(2.0, self.check)
        self.checked = False

    def check(self) -> None:
        if self.checked:
            return
        topics = dict(self.get_topic_names_and_types())
        missing = [t for t in REQUIRED_TOPICS if t not in topics]
        if missing:
            self.get_logger().error(f"Missing topics: {missing}")
            assert not missing, f"E2E test failed: missing topics {missing}"
        else:
            self.get_logger().info("E2E test passed. All required topics found.")
            self.checked = True
            rclpy.shutdown()

def main():
    rclpy.init()
    node = SmokeTest()
    rclpy.spin(node)

def test_setup() -> None:
    pass

def test_launch() -> None:
    pass

def test_teardown() -> None:
    pass

if __name__ == '__main__':
    main()
