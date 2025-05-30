"""Bridge base for ROS 2 bridge nodes (typed for mypy)."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import importlib

class BridgeBase(Node):
    """
    Base class for all bridge nodes. Handles parameter declaration, QoS profile setup, and logging level.
    """
    def __init__(self, node_name: str, params: dict) -> None:
        """
        Initialize BridgeBase with node name and parameter dictionary.
        Declares parameters and sets up QoS profile and log level.
        Args:
            node_name (str): Name of the ROS 2 node.
            params (dict): Dictionary of parameter names and default values.
        """
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

    @staticmethod
    def create_node(class_path: str, node_name: str, params: dict) -> Node:
        """
        Factory method to create a node instance from a class path string.
        Args:
            class_path (str): Python path to the node class (e.g., 'angle_bridge.angle_bridge.main.AngleBridgeNode').
            node_name (str): Name of the ROS 2 node.
            params (dict): Dictionary of parameter names and default values.
        Returns:
            Node: Instantiated ROS 2 node object.
        """
        module_path, class_name = class_path.rsplit('.', 1)
        module = importlib.import_module(module_path)
        node_class = getattr(module, class_name)
        return node_class(node_name, params) 