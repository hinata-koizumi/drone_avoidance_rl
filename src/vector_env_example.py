# vector_env_example.py
"""
SubprocVecEnvでDroneSimEnvをベクトル化して複数同時実行するサンプル。
"""
import numpy as np
from gymnasium.vector import AsyncVectorEnv
from gym_env import DroneSimEnv

def make_env(
    instance_id: int,
    reward_mode: str = "default",
    episode_max_steps: int = 2000
) -> callable[[], DroneSimEnv]:
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

    obs = env.reset()
    for _ in range(10):
        actions = np.stack([env.single_action_space.sample() for _ in range(num_envs)])
        obs, rewards, terminated, truncated, infos = env.step(actions)
        print(f"obs: {obs}, rewards: {rewards}")
    env.close() 