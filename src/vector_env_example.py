# vector_env_example.py
"""
SubprocVecEnvでDroneSimEnvをベクトル化して複数同時実行するサンプル。
"""
import numpy as np
from gymnasium.vector import AsyncVectorEnv
from gym_env import DroneSimEnv

def make_env(instance_id):
    def _init():
        return DroneSimEnv(instance_id=instance_id)
    return _init

if __name__ == "__main__":
    num_envs = 2  # 必要に応じて増減
    env_fns = [make_env(i) for i in range(num_envs)]
    env = AsyncVectorEnv(env_fns)

    obs = env.reset()
    for _ in range(10):
        actions = np.stack([env.single_action_space.sample() for _ in range(num_envs)])
        obs, rewards, terminated, truncated, infos = env.step(actions)
        print(f"obs: {obs}, rewards: {rewards}")
    env.close() 