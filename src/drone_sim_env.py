import gymnasium as gym
import numpy as np

class DroneSimEnv(gym.Env):
    def __init__(self, instance_id=0, reward_mode="default", episode_max_steps=2000):
        super().__init__()
        self.instance_id = instance_id
        self.reward_mode = reward_mode
        self.episode_max_steps = episode_max_steps
        self.current_step = 0
        # 例: 4次元連続状態、2次元連続アクション
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32)
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        self.current_step = 0
        obs = np.zeros(self.observation_space.shape, dtype=np.float32)
        return obs, {}

    def step(self, action):
        self.current_step += 1
        obs = np.zeros(self.observation_space.shape, dtype=np.float32)
        reward = 0.0
        terminated = self.current_step >= self.episode_max_steps
        truncated = False
        info = {}
        return obs, reward, terminated, truncated, info

    def render(self):
        pass

    def close(self):
        pass 