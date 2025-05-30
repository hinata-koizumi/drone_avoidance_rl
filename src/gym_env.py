#!/usr/bin/env python3
# ────────────────────────────────────────────
# gym_env.py   ★2025-05-02 R7
#   * I-1  roll-pitch-yaw 順へ合わせる
#   * I-2  set_physics_properties → set_physics へ変更
# ────────────────────────────────────────────
import os
import random
import shutil
import socket
import subprocess
import time
import threading
import yaml
from ament_index_python.packages import get_package_share_directory

import gymnasium as gym
import numpy as np
import rclpy
from gymnasium import spaces

from drone_msgs.msg import DroneControlCommand as _DroneControlCommand
from drone_msgs.msg import DroneState as _DroneState
from src.common.utils import clamp


# ---------------- util ----------------
def _wait_udp(port: int, timeout: float = 5.0) -> None:
    """
    Wait until a UDP port is open (used for PX4 RTPS startup sync).
    Args:
        port (int): UDP port to check.
        timeout (float, optional): Timeout in seconds. Defaults to 5.0.
    Raises:
        RuntimeError: If the port is not open within the timeout.
    """
    start = time.time()
    while time.time() - start < timeout:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            if s.connect_ex(("127.0.0.1", port)) == 0:
                return
        time.sleep(0.1)
    raise RuntimeError(f"UDP {port} not open within {timeout}s")

class DroneSimEnv(gym.Env):
    """
    Gymnasium-compatible environment for drone RL using ROS 2 and Ignition Gazebo.
    Each instance uses a unique node name, topic, and UDP port for parallelization.
    Parameters are loaded from YAML and can be overridden via ROS 2 params.
    """
    metadata = {"render_modes": ["human"]}
    ORI_TARGET = np.zeros(3, dtype=np.float32)

    # パラメータドキュメント（自動生成用）
    PARAM_DOCS = {
        'reward': {
            'ori':   {'type': 'float', 'desc': '姿勢誤差に対するペナルティ重み'},
            'pos':   {'type': 'float', 'desc': '位置誤差に対するペナルティ重み'},
            'smooth':{'type': 'float', 'desc': '行動変化（スムーズさ）に対するペナルティ重み'},
        },
        'physics': {
            'mass':    {'type': 'float', 'desc': '機体質量[kg]'},
            'inertia': {'type': 'float', 'desc': '慣性モーメント（簡易）'},
        }
    }

    def __init__(
        self,
        episode_max_steps: int = 2_000,
        target_alt: float = 3.0,
        wind_max: float = 0.0,
        gust_max: float = 0.0,
        gust_prob: float = 0.0,
        instance_id: int = 0,
        reward_mode: str = "default",
    ) -> None:
        """
        Initialize DroneSimEnv with simulation and reward parameters.
        Args:
            episode_max_steps (int): Maximum steps per episode.
            target_alt (float): Target altitude for the drone.
            wind_max (float): Maximum wind speed.
            gust_max (float): Maximum gust speed.
            gust_prob (float): Probability of gust occurrence.
            instance_id (int): Unique instance ID for parallel environments.
            reward_mode (str): Reward calculation mode.
        """
        super().__init__()
        # --- ROS 2パラメータ優先で取得 ---
        if rclpy.ok():
            rclpy.shutdown()
        rclpy.init(args=None)
        node_name = f"inner_prop_rl_env_{instance_id}"
        self.node = rclpy.create_node(node_name)
        self.episode_max_steps = self.node.declare_parameter("episode_max_steps", episode_max_steps).value
        self.target_alt = self.node.declare_parameter("target_alt", target_alt).value
        self.wind_max = self.node.declare_parameter("wind_max", wind_max).value
        self.gust_max = self.node.declare_parameter("gust_max", gust_max).value
        self.gust_prob = self.node.declare_parameter("gust_prob", gust_prob).value
        self.reward_mode = self.node.declare_parameter("reward_mode", reward_mode).value
        self.POS_TARGET = np.array([0.0, 0.0, self.target_alt], dtype=np.float32)
        self.instance_id = instance_id

        self.action_space = spaces.Box(
            low=np.array([0.0, -30.0, 0.0, -30.0], dtype=np.float32),
            high=np.array([1.0, 30.0, 1.0, 30.0], dtype=np.float32),
            dtype=np.float32,
        )
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(15,), dtype=np.float32)

        self.state = np.zeros(15, dtype=np.float32)
        self.prev_action = np.zeros(4, dtype=np.float32)
        self.step_count = 0
        self.max_steps = self.episode_max_steps
        self.crashed = False

        config_path = os.path.join(get_package_share_directory('sim_launch'), 'config', 'sim_params.yaml')
        with open(config_path, 'r') as f:
            params = yaml.safe_load(f)
        # 報酬重みをYAMLから取得し型バリデーション
        try:
            self.rew_ori    = float(params['reward']['ori']['value'])
            self.rew_pos    = float(params['reward']['pos']['value'])
            self.rew_smooth = float(params['reward']['smooth']['value'])
        except (KeyError, ValueError, TypeError):
            raise RuntimeError('YAMLのreward重みが不正です')
        # 物理パラメータ例
        try:
            self.mass    = float(params['physics']['mass']['value'])
            self.inertia = float(params['physics']['inertia']['value'])
        except (KeyError, ValueError, TypeError):
            self.mass = 1.0
            self.inertia = 0.1
        self.cmd_topic = self.node.declare_parameter("cmd_topic", params["cmd_topic"]).value
        self.cmd_pub = self.node.create_publisher(
            _DroneControlCommand, self.cmd_topic, 10
        )
        self.state_sub = self.node.create_subscription(
            _DroneState, f"/drone{instance_id}/state", self._state_cb, 10
        )

        for tool in ("ign", "nc"):
            if shutil.which(tool) is None:
                raise RuntimeError(f"Required tool '{tool}' not found in RL-agent image")

        self._state_event = threading.Event()
        self._last_step_count = 0
        # rclpyスピンを別スレッドで常時回す
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self) -> None:
        """
        Spin the ROS 2 node in a background thread for message handling.
        """
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)

    # ----------------------- Callbacks -----------------------
    def _state_cb(self, msg: _DroneState) -> None:
        """
        Callback for drone state subscription. Updates internal state vector.
        Args:
            msg (_DroneState): Incoming drone state message.
        """
        # ★ roll, pitch, yaw の順で格納（state_bridge と一致）
        self.state = np.array([
            msg.roll, msg.pitch, msg.yaw,
            msg.position.x, msg.position.y, msg.position.z,
            msg.velocity.x, msg.velocity.y, msg.velocity.z,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.wind.x, msg.wind.y, msg.wind.z,
        ], dtype=np.float32)
        if msg.position.z < 0.3:
            self.crashed = True
        self._state_event.set()

    # ------------------ Gym API ------------------------------
    def reset(self, *, seed: int | None = None, options: dict | None = None) -> tuple[np.ndarray, dict]:
        """
        Reset the environment and randomize the world. Returns initial observation and info dict.
        Args:
            seed (int, optional): Random seed.
            options (dict, optional): Additional options.
        Returns:
            tuple[np.ndarray, dict]: Initial observation and info.
        """
        super().reset(seed=seed)
        # subprocess.runをPopenで並列化
        procs = self._randomize_world_async()
        procs += self._world_reset_async()
        for p in procs:
            p.wait()
        self.step_count = 0
        self.crashed = False
        self.prev_action.fill(0.0)
        _wait_udp(11345 + self.instance_id)
        self._state_event.clear()
        return self.state.copy(), {}

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict]:
        """
        Take an action in the environment. Publishes command and waits for new state.
        Args:
            action (np.ndarray): Action array [throttle1, angle1, throttle2, angle2].
        Returns:
            tuple: (observation, reward, terminated, truncated, info)
        """
        cmd = _DroneControlCommand()
        cmd.throttle1, cmd.angle1, cmd.throttle2, cmd.angle2 = map(float, action)
        self.cmd_pub.publish(cmd)
        # 新しい状態が来るまで待つ
        self._state_event.clear()
        # 0.05秒タイムアウト（必要に応じて調整）
        self._state_event.wait(timeout=0.05)
        reward = self._calc_reward(action)
        self.prev_action = action.copy()
        self.step_count += 1
        terminated = self.crashed
        truncated = self.step_count >= self.max_steps
        return self.state.copy(), reward, terminated, truncated, {}

    def close(self) -> None:
        """
        Shutdown the ROS 2 node and clean up resources.
        """
        rclpy.shutdown()

    # ------------------ helpers ------------------------------
    def _randomize_world_async(self) -> list[subprocess.Popen]:
        """
        Randomize world physics, wind, and battery voltage asynchronously.
        Returns:
            list[subprocess.Popen]: List of subprocesses for randomization commands.
        """
        mass_scale = random.uniform(0.92, 1.08)
        com_shift  = [random.uniform(-0.005, 0.005) for _ in range(3)]
        wind_vec   = [random.uniform(-3.0, 3.0) for _ in range(3)]
        gust_speed = random.uniform(0.0, 6.0)
        gust_dir   = [random.uniform(-1.0, 1.0) for _ in range(3)]
        batt_v     = random.uniform(10.8, 13.2)  # Li-ion 4S eq.
        procs = []
        req_str = (
            f"gravity_vector:{{x:0, y:0, z:-9.81}} mass_scale:{mass_scale} "
            f"center_of_mass:{{x:{com_shift[0]}, y:{com_shift[1]}, z:{com_shift[2]}}}"
        )
        procs.append(subprocess.Popen([
            "ign", "service", "-s", "/world/empty/physics/set_parameters",
            "--reqtype", "ignition.msgs.PhysicsParameters", "--reptype", "ignition.msgs.Boolean",
            "--timeout", "300",
            "--req",
            req_str
        ]))
        procs.append(subprocess.Popen([
            "ign", "service", "-s", "/world/empty/atmosphere/set_parameters",
            "--reqtype", "ignition.msgs.AtmosphereParameters", "--reptype", "ignition.msgs.Boolean",
            "--timeout", "300",
            "--req",
            f"wind_velocity:{{x:{wind_vec[0]}, y:{wind_vec[1]}, z:{wind_vec[2]}}}"
        ]))
        procs.append(subprocess.Popen([
            "ign", "service", "-s", "/world/empty/atmosphere/set_gust",
            "--reqtype", "ignition.msgs.AtmosphereGust", "--reptype", "ignition.msgs.Boolean",
            "--timeout", "300",
            "--req",
            f"velocity:{gust_speed} direction:{{x:{gust_dir[0]}, y:{gust_dir[1]}, z:{gust_dir[2]}}}"
        ]))
        procs.append(subprocess.Popen([
            "bash", "-c",
            f"printf 'param set BAT_V_CHARGED {batt_v:.2f}\\n' | nc -u -w1 localhost 14556"
        ]))
        return procs

    def _world_reset_async(self) -> list[subprocess.Popen]:
        """
        Reset the simulation world asynchronously.
        Returns:
            list[subprocess.Popen]: List of subprocesses for world reset commands.
        """
        procs = []
        procs.append(subprocess.Popen([
            "ign", "service", "-s", "/world/empty/control",
            "--reqtype", "ignition.msgs.WorldControl",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "300",
            "--req", "reset: { all: true }"
        ]))
        procs.append(subprocess.Popen([
            "bash", "-c",
            "printf 'commander reset all\\ncommander attitude set 0 0 0\\n' | nc -u -w1 localhost 14556"
        ]))
        return procs

    def _calc_reward(self, action: np.ndarray) -> float:
        """
        Calculate the reward for the current state and action.
        Args:
            action (np.ndarray): Action taken.
        Returns:
            float: Calculated reward.
        """
        roll, pitch, yaw = self.state[:3]
        pos = self.state[3:6]
        ori_err = np.linalg.norm(np.array([roll, pitch, yaw]) - self.ORI_TARGET)
        pos_err = np.linalg.norm(pos - self.POS_TARGET)
        smooth  = np.linalg.norm(action - self.prev_action)
        if self.reward_mode == "hover":
            pos_xy_err = np.linalg.norm(pos[:2] - self.POS_TARGET[:2])
            pos_z_err = abs(pos[2] - self.POS_TARGET[2])
            reward = (
                self.rew_ori * ori_err
                + 0.5 * self.rew_pos * pos_xy_err
                + 2.0 * self.rew_pos * pos_z_err
                + self.rew_smooth * smooth
            )
            if self.crashed:
                reward -= 10.0
            return reward
        elif self.reward_mode == "path_follow":
            reward = 2.0 * self.rew_pos * pos_err + 0.5 * self.rew_ori * ori_err + self.rew_smooth * smooth
            if self.crashed:
                reward -= 10.0
            return reward
        elif self.reward_mode == "obstacle_avoid":
            reward = self.rew_pos * pos_err + self.rew_ori * ori_err + self.rew_smooth * smooth
            if self.crashed:
                reward -= 20.0
            return reward
        else:
            return float(self.rew_ori * ori_err + self.rew_pos * pos_err + self.rew_smooth * smooth)

    def render(self) -> None:
        """
        Render the environment (no-op for headless simulation).
        """
        pass

    def _clamp(self, x: float, lo: float, hi: float) -> float:
        """
        Clamp a value between lo and hi (wrapper for utils.clamp).
        Args:
            x (float): Value to clamp.
            lo (float): Lower bound.
            hi (float): Upper bound.
        Returns:
            float: Clamped value.
        """
        return float(clamp(x, lo, hi))

    def _normalize_angle(self, angle: float) -> float:
        """
        Normalize an angle (placeholder).
        Args:
            angle (float): Angle to normalize.
        Returns:
            float: Normalized angle.
        """
        return float(angle)

    def _denormalize_angle(self, norm: float) -> float:
        """
        Denormalize an angle (placeholder).
        Args:
            norm (float): Normalized value.
        Returns:
            float: Denormalized angle.
        """
        return float(norm)
