# vector_env_example.py
"""
SubprocVecEnvでDroneSimEnvをベクトル化して複数同時実行するサンプル。
"""
import numpy as np
from gymnasium.vector import AsyncVectorEnv
from gym_env import DroneSimEnv
from typing import Callable
from torch.utils.tensorboard import SummaryWriter

def make_env(
    instance_id: int,
    reward_mode: str = "default",
    episode_max_steps: int = 2000
) -> Callable[[], DroneSimEnv]:
    """
    Factory function to create DroneSimEnv initializer for vectorized environments.
    Args:
        instance_id (int): Unique instance ID for the environment.
        reward_mode (str, optional): Reward mode. Defaults to "default".
        episode_max_steps (int, optional): Max steps per episode. Defaults to 2000.
    Returns:
        Callable[[], DroneSimEnv]: Initializer function for AsyncVectorEnv.
    """
    def _init() -> DroneSimEnv:
        """
        Initialize a DroneSimEnv instance with the given parameters.
        Returns:
            DroneSimEnv: The initialized environment instance.
        """
        return DroneSimEnv(instance_id=instance_id, reward_mode=reward_mode, episode_max_steps=episode_max_steps)
    return _init

if __name__ == "__main__":
    num_envs = 2  # 必要に応じて増減
    # 例: hoverモードで2環境
    env_fns = [make_env(i, reward_mode="hover", episode_max_steps=1000) for i in range(num_envs)]
    env = AsyncVectorEnv(env_fns)

    writer = SummaryWriter(log_dir="runs/drone_vector_env")
    num_episodes = 10
    episode_rewards = [[] for _ in range(num_envs)]
    obs = env.reset()
    for ep in range(num_episodes):
        terminated = [False] * num_envs
        truncated = [False] * num_envs
        rewards = [0.0] * num_envs
        obs = env.reset()
        while not all(terminated) and not all(truncated):
            actions = np.stack([env.single_action_space.sample() for _ in range(num_envs)])
            obs, step_rewards, terminated, truncated, infos = env.step(actions)
            for i in range(num_envs):
                rewards[i] += step_rewards[i]
        for i in range(num_envs):
            episode_rewards[i].append(rewards[i])
            writer.add_scalar(f"Reward/Episode_env{i}", rewards[i], ep)
    # 各環境の平均reward
    for i in range(num_envs):
        mean_reward = np.mean(episode_rewards[i])
        writer.add_scalar(f"Reward/Mean_env{i}", mean_reward, num_episodes)
    writer.close()
    env.close() 