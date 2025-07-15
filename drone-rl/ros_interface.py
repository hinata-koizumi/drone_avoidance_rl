import threading
from typing import Optional, Callable

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Header
    from geometry_msgs.msg import Point, Vector3
    from drone_msgs.msg import DroneControlCommand, DroneState
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    # Define dummy message classes to avoid NameError during type checking/execution
    class Header:  # type: ignore
        pass

    class Point:  # type: ignore
        pass

    class Vector3:  # type: ignore
        pass

    class DroneState:  # type: ignore
        pass

    class DroneControlCommand:  # type: ignore
        pass

class DummyROSNode:
    def __init__(self, *args, **kwargs):
        self.state_cb = None
    def publish_control(self, cmd):
        pass
    def set_state_callback(self, cb):
        self.state_cb = cb
    def reset_sim(self):
        # Placeholder for simulation reset in non-ROS mode
        print("[DummyROSNode] reset_sim called (no-op)")

class DroneROSInterface:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._initialized = False
            return cls._instance

    def __init__(self, drone_ns: str = "/drone0"):
        if self._initialized:
            return
        self._initialized = True
        self.drone_ns = drone_ns
        self.state: Optional[DroneState] = None
        self.state_cb: Optional[Callable] = None
        if ROS_AVAILABLE:
            rclpy.init(args=None)
            self.node = rclpy.create_node(f"drone_rl_interface_{drone_ns.replace('/', '')}")
            self.pub = self.node.create_publisher(DroneControlCommand, f"{drone_ns}/control", 10)
            self.sub = self.node.create_subscription(DroneState, f"{drone_ns}/state", self._state_callback, 10)
            self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
            self.spin_thread.start()
        else:
            self.node = DummyROSNode()

    def _state_callback(self, msg):
        self.state = msg
        if self.state_cb:
            self.state_cb(msg)

    def publish_control(self, thrust, roll, pitch, yaw_rate, control_mode="MANUAL"):
        if ROS_AVAILABLE:
            cmd = DroneControlCommand()
            cmd.header = Header()
            cmd.header.stamp = self.node.get_clock().now().to_msg()
            cmd.thrust = float(thrust)
            cmd.roll = float(roll)
            cmd.pitch = float(pitch)
            cmd.yaw_rate = float(yaw_rate)
            cmd.control_mode = control_mode
            self.pub.publish(cmd)
        else:
            self.node.publish_control(None)

    def set_state_callback(self, cb: Callable):
        self.state_cb = cb

    def get_latest_state(self) -> Optional[DroneState]:
        return self.state

    def reset_sim(self):
        # TODO: Implement reset via ROS2 service or topic if available
        if ROS_AVAILABLE:
            # Attempt to trigger /sim/reset service of type std_srvs/Empty
            from std_srvs.srv import Empty  # type: ignore
            client = self.node.create_client(Empty, "/sim/reset")
            if not client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().error("/sim/reset service unavailable")
                return False
            req = Empty.Request()
            future = client.call_async(req)
            # Wait synchronously with short timeout to avoid blocking indefinitely
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            if future.result() is None:
                self.node.get_logger().error("Reset service call failed")
                return False
            return True
        else:
            self.node.reset_sim()
            return True 