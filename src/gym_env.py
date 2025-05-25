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

import gymnasium as gym
import numpy as np
import rclpy
from gymnasium import spaces

from drone_msgs.msg import DroneControlCommand as _DroneControlCommand
from drone_msgs.msg import DroneState as _DroneState
from src.common.utils import clamp


# ---------------- util ----------------
def _wait_udp(port: int, timeout: float = 5.0) -> None:
    """PX4 RTPS ポートが開くまで待機"""
    start = time.time()
    while time.time() - start < timeout:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            if s.connect_ex(("127.0.0.1", port)) == 0:
                return
        time.sleep(0.1)
    raise RuntimeError(f"UDP {port} not open within {timeout}s")

_REW_ORI    = float(os.getenv("REWARD_ORI",    "-1.0"))
_REW_POS    = float(os.getenv("REWARD_POS",    "-1.0"))
_REW_SMOOTH = float(os.getenv("REWARD_SMOOTH", "-0.02"))

class DroneSimEnv(gym.Env):
    """
    ROS 2 + Ignition Gazeboベースのドローン強化学習用Gym環境。
    - SubprocVecEnv等のベクトル化環境での利用を想定し、instance_idでリソース分離可能。
    - 各インスタンスは独立したノード名・トピック名・UDPポートを使用。
    """
    metadata = {"render_modes": ["human"]}
    ORI_TARGET = np.zeros(3, dtype=np.float32)

    def __init__(
        self,
        episode_max_steps: int = 2_000,
        target_alt: float = 3.0,
        wind_max: float = 0.0,
        gust_max: float = 0.0,
        gust_prob: float = 0.0,
        instance_id: int = 0,
    ) -> None:
        super().__init__()
        self.POS_TARGET = np.array([0.0, 0.0, target_alt], dtype=np.float32)
        self.wind_max = wind_max
        self.gust_max = gust_max
        self.gust_prob = gust_prob
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
        self.max_steps = episode_max_steps
        self.crashed = False

        if rclpy.ok():
            rclpy.shutdown()
        rclpy.init(args=None)
        node_name = f"inner_prop_rl_env_{instance_id}"
        self.node = rclpy.create_node(node_name)
        self.cmd_topic = self.node.declare_parameter("cmd_topic", f"/drone{instance_id}/inner_propeller_cmd").value
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

    def _spin(self):
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)

    # ----------------------- Callbacks -----------------------
    def _state_cb(self, msg: _DroneState) -> None:
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
        rclpy.shutdown()

    # ------------------ helpers ------------------------------
    def _randomize_world_async(self):
        mass_scale = random.uniform(0.92, 1.08)
        com_shift  = [random.uniform(-0.005, 0.005) for _ in range(3)]
        wind_vec   = [random.uniform(-3.0, 3.0) for _ in range(3)]
        gust_speed = random.uniform(0.0, 6.0)
        gust_dir   = [random.uniform(-1.0, 1.0) for _ in range(3)]
        batt_v     = random.uniform(10.8, 13.2)  # Li-ion 4S eq.
        procs = []
        procs.append(subprocess.Popen([
            "ign", "service", "-s", "/world/empty/set_physics",
            "--reqtype", "ignition.msgs.Physics", "--reptype", "ignition.msgs.Boolean",
            "--timeout", "300",
            "--req",
            f"mass_scale:{mass_scale} center_of_mass:{{x:{com_shift[0]}, y:{com_shift[1]}, z:{com_shift[2]}}}"
        ]))
        procs.append(subprocess.Popen([
            "ign", "service", "-s", "/world/empty/set_wind",
            "--reqtype", "ignition.msgs.Wind", "--reptype", "ignition.msgs.Boolean",
            "--timeout", "300",
            "--req",
            f"linear_velocity:{{x:{wind_vec[0]}, y:{wind_vec[1]}, z:{wind_vec[2]}}}"
        ]))
        procs.append(subprocess.Popen([
            "ign", "service", "-s", "/world/empty/set_wind_gust",
            "--reqtype", "ignition.msgs.Wind", "--reptype", "ignition.msgs.Boolean",
            "--timeout", "300",
            "--req",
            f"speed:{gust_speed} direction:{{x:{gust_dir[0]}, y:{gust_dir[1]}, z:{gust_dir[2]}}}"
        ]))
        procs.append(subprocess.Popen([
            "bash", "-c",
            f"printf 'param set BAT_V_CHARGED {batt_v:.2f}\\n' | nc -u -w1 localhost 14556"
        ]))
        return procs

    def _world_reset_async(self):
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
        roll, pitch, yaw = self.state[:3]
        pos = self.state[3:6]
        ori_err = np.linalg.norm(np.array([roll, pitch, yaw]) - self.ORI_TARGET)
        pos_err = np.linalg.norm(pos - self.POS_TARGET)
        smooth  = np.linalg.norm(action - self.prev_action)
        return float(_REW_ORI * ori_err + _REW_POS * pos_err + _REW_SMOOTH * smooth)

    def render(self) -> None:
        pass

    def _clamp(self, x: float, lo: float, hi: float) -> float:
        return clamp(x, lo, hi)

    def _normalize_angle(self, angle: float) -> float:
        return angle

    def _denormalize_angle(self, norm: float) -> float:
        return norm
