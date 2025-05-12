#!/usr/bin/env python3
# ────────────────────────────────────────────
# state_bridge.py   ★CHANGE 2025-04-30 R3
#   * _cb をクラス内へ移動
#   * topic QoS, NaN ガードは現状維持
# ────────────────────────────────────────────
import math
from typing import Tuple, Any

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from drone_msgs.msg import DroneState
from px4_msgs.msg import VehicleOdometry

_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

try:
    from scipy.spatial.transform import Rotation as R
    _ROT_AVAILABLE = True
except ImportError:
    _ROT_AVAILABLE = False


def quat_to_euler(w: float, x: float, y: float, z: float) -> Tuple[float, float, float]:
    if _ROT_AVAILABLE:
        r = R.from_quat([x, y, z, w])      # scipy は [x, y, z, w]
        return r.as_euler("xyz", degrees=False)
    # 汎用式
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class StateBridge(Node):
    """PX4 VehicleOdometry → custom DroneState"""
    def __init__(self) -> None:
        super().__init__("state_bridge")
        self.pub = self.create_publisher(DroneState, "/drone/state", 10)
        self.sub = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self._cb,
            _QOS,
        )

    # -------------- callback --------------
    def _cb(self, odo: VehicleOdometry) -> None:
        msg = DroneState()
        msg.roll, msg.pitch, msg.yaw = quat_to_euler(odo.q[0], odo.q[1], odo.q[2], odo.q[3])

        msg.position = Vector3(x=odo.position[0], y=odo.position[1], z=odo.position[2])
        msg.velocity = Vector3(x=odo.velocity[0], y=odo.velocity[1], z=odo.velocity[2])
        msg.angular_velocity = Vector3(
            x=odo.angular_velocity[0],
            y=odo.angular_velocity[1],
            z=odo.angular_velocity[2],
        )
        msg.wind = Vector3()  # sensor 未実装

        if any(math.isnan(v) for v in (
            msg.roll, msg.pitch, msg.yaw,
            msg.position.x, msg.position.y, msg.position.z
        )):
            self.get_logger().warn("NaN in VehicleOdometry – skipped frame")
            return

        self.pub.publish(msg)

    def _get_xyz(self, odom: 'VehicleOdometry') -> Any:
        return (0.0, 0.0, 0.0)


def main() -> None:
    rclpy.init()
    rclpy.spin(StateBridge())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
