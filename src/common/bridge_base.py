import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class BridgeBase(Node):
    def __init__(self, node_name, params):
        super().__init__(node_name)
        # パラメータ宣言・取得を共通化
        for name, default in params.items():
            self.declare_parameter(name, default)
        # QoS共通化
        qos_depth = self.get_parameter('qos_depth').value
        qos_reliability = self.get_parameter('qos_reliability').value
        qos_history = self.get_parameter('qos_history').value
        self.qos_profile = QoSProfile(
            depth=qos_depth,
            reliability=ReliabilityPolicy.RELIABLE if qos_reliability == 'reliable' else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST if qos_history == 'keep_last' else HistoryPolicy.KEEP_ALL
        )
        # ログレベル
        self.log_level = self.get_parameter('log_level').value 