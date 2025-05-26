import numpy as np
import pytest

from gym_env import DroneSimEnv

def test_rl_longrun_stability():
    env = DroneSimEnv(episode_max_steps=1000)
    obs, info = env.reset()
    for _ in range(1000):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        assert isinstance(obs, np.ndarray)
        assert isinstance(reward, float)
        if terminated or truncated:
            obs, info = env.reset()
    env.close() 