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
from typing import Dict, Tuple

import gymnasium as gym
import numpy as np
import rclpy
from gymnasium import spaces

from drone_msgs.msg import DroneControlCommand, DroneState


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
    metadata = {"render_modes": ["human"]}
    ORI_TARGET = np.zeros(3, dtype=np.float32)

    def __init__(self, episode_max_steps: int = 2_000, target_alt: float = 3.0):
        super().__init__()
        self.POS_TARGET = np.array([0.0, 0.0, target_alt], dtype=np.float32)

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
        self.node = rclpy.create_node("inner_prop_rl_env")
        self.cmd_pub = self.node.create_publisher(
            DroneControlCommand, "/drone/inner_propeller_cmd", 10
        )
        self.state_sub = self.node.create_subscription(
            DroneState, "/drone/state", self._state_cb, 10
        )

        for tool in ("ign", "nc"):
            if shutil.which(tool) is None:
                raise RuntimeError(f"Required tool '{tool}' not found in RL-agent image")

    # ----------------------- Callbacks -----------------------
    def _state_cb(self, msg: DroneState) -> None:
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

    # ------------------ Gym API ------------------------------
    def reset(self, *, seed=None, options=None) -> Tuple[np.ndarray, Dict]:
        super().reset(seed=seed)
        self._randomize_world()
        self._world_reset()
        self.step_count = 0
        self.crashed = False
        self.prev_action.fill(0.0)
        _wait_udp(11345)
        return self.state.copy(), {}

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict]:
        cmd = DroneControlCommand()
        cmd.throttle1, cmd.angle1, cmd.throttle2, cmd.angle2 = map(float, action)
        self.cmd_pub.publish(cmd)
        rclpy.spin_once(self.node, timeout_sec=0.03)

        reward = self._calc_reward(action)
        self.prev_action = action.copy()
        self.step_count += 1
        terminated = self.crashed
        truncated = self.step_count >= self.max_steps
        return self.state.copy(), reward, terminated, truncated, {}

    def close(self):
        rclpy.shutdown()

    # ------------------ helpers ------------------------------
    def _randomize_world(self) -> None:
        mass_scale = random.uniform(0.92, 1.08)
        com_shift  = [random.uniform(-0.005, 0.005) for _ in range(3)]
        wind_vec   = [random.uniform(-3.0, 3.0) for _ in range(3)]
        gust_speed = random.uniform(0.0, 6.0)
        gust_dir   = [random.uniform(-1.0, 1.0) for _ in range(3)]
        batt_v     = random.uniform(10.8, 13.2)  # Li-ion 4S eq.

        # 1) Physics
        subprocess.run(
            ["ign", "service", "-s", "/world/empty/set_physics",
             "--reqtype", "ignition.msgs.Physics", "--reptype", "ignition.msgs.Boolean",
             "--timeout", "300",
             "--req",
             f"mass_scale:{mass_scale} center_of_mass:{{x:{com_shift[0]}, y:{com_shift[1]}, z:{com_shift[2]}}}"],
            check=True,
        )
        # 2) Wind steady
        subprocess.run(
            ["ign", "service", "-s", "/world/empty/set_wind",
             "--reqtype", "ignition.msgs.Wind", "--reptype", "ignition.msgs.Boolean",
             "--timeout", "300",
             "--req",
             f"linear_velocity:{{x:{wind_vec[0]}, y:{wind_vec[1]}, z:{wind_vec[2]}}}"],
            check=True,
        )
        # 3) Wind gust
        subprocess.run(
            ["ign", "service", "-s", "/world/empty/set_wind_gust",
             "--reqtype", "ignition.msgs.Wind", "--reptype", "ignition.msgs.Boolean",
             "--timeout", "300",
             "--req",
             f"speed:{gust_speed} direction:{{x:{gust_dir[0]}, y:{gust_dir[1]}, z:{gust_dir[2]}}}"],
            check=True,
        )
        # 4) Battery voltage (PX4 param)
        subprocess.run(
            ["bash", "-c",
             f"printf 'param set BAT_V_CHARGED {batt_v:.2f}\\n' | "
             "nc -u -w1 localhost 14556"],
            check=True,
        )

    def _world_reset(self) -> None:
        subprocess.run(
            ["ign", "service", "-s", "/world/empty/control",
             "--reqtype", "ignition.msgs.WorldControl",
             "--reptype", "ignition.msgs.Boolean",
             "--timeout", "300",
             "--req", "reset: { all: true }"],
            check=True,
        )
        subprocess.run(
            ["bash", "-c",
             "printf 'commander reset all\\ncommander attitude set 0 0 0\\n' | "
             "nc -u -w1 localhost 14556"],
            check=True,
        )

    def _calc_reward(self, action: np.ndarray) -> float:
        roll, pitch, yaw = self.state[:3]          # ★順番更新
        pos = self.state[3:6]

        ori_err = np.linalg.norm(np.array([roll, pitch, yaw]) - self.ORI_TARGET)
        pos_err = np.linalg.norm(pos - self.POS_TARGET)
        smooth  = np.linalg.norm(action - self.prev_action)

        return _REW_ORI * ori_err + _REW_POS * pos_err + _REW_SMOOTH * smooth

    def render(self) -> None:
        pass

    def _clamp(self, x: float, lo: float, hi: float) -> float:
        return x

    def _normalize_angle(self, angle: float) -> float:
        return angle

    def _denormalize_angle(self, norm: float) -> float:
        return norm
