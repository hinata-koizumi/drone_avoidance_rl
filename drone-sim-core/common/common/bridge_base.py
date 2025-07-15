"""Bridge base for ROS 2 bridge nodes (typed for mypy)."""

import importlib
import json
import logging
from functools import wraps

from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


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
        # ------------------------------------------------------------------
        # Logging setup (structured JSON)
        # ------------------------------------------------------------------
        _default_log_format = {
            "time": "%(asctime)s",
            "level": "%(levelname)s",
            "name": "%(name)s",
            "msg": "%(message)s"
        }
        logging.basicConfig(level=logging.INFO, format=json.dumps(_default_log_format))
        self.logger = logging.getLogger(node_name)

        # ------------------------------------------------------------------
        # Parameter declaration / retrieval
        # ------------------------------------------------------------------
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
        self.logger.setLevel(self.log_level.upper() if isinstance(self.log_level, str) else logging.INFO)

    # ------------------------------------------------------------------
    # Utility: safe callback wrapper to prevent hard crashes
    # ------------------------------------------------------------------
    @staticmethod
    def safe_callback(func):
        """Decorator that wraps ROS2 callbacks and catches/logs exceptions."""

        @wraps(func)
        def _wrapper(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except Exception as exc:  # pylint: disable=broad-except
                node: "BridgeBase | None" = None
                # Find self/node in args
                if args and isinstance(args[0], BridgeBase):
                    node = args[0]
                if node is not None and hasattr(node, "get_logger"):
                    node.get_logger().error(f"Callback {func.__name__} failed: {exc}")
                else:
                    logging.getLogger("BridgeBase").error("Callback %s failed: %s", func.__name__, exc)
        return _wrapper

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