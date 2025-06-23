import gymnasium as gym
import numpy as np
from typing import Any

class DroneSimEnv(gym.Env):
    def __init__(self, instance_id: int = 0, reward_mode: str = "default", episode_max_steps: int = 2000) -> None:
        super().__init__()
        self.instance_id = instance_id
        self.reward_mode = reward_mode
        self.episode_max_steps = episode_max_steps
        self.current_step = 0
        # 例: 4次元連続状態、2次元連続アクション
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32)
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[np.ndarray, dict[str, Any]]:
        self.current_step = 0
        shape = self.observation_space.shape
        if shape is None:
            shape = (4,)  # デフォルト値
        obs = np.zeros(shape, dtype=np.float32)
        return obs, {}

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        self.current_step += 1
        shape = self.observation_space.shape
        if shape is None:
            shape = (4,)  # デフォルト値
        obs = np.zeros(shape, dtype=np.float32)
        reward = 0.0
        terminated = self.current_step >= self.episode_max_steps
        truncated = False
        info: dict[str, Any] = {}
        return obs, reward, terminated, truncated, info

    def render(self) -> None:
        pass

    def close(self) -> None:
        pass 