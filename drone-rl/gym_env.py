"""gym_env.py
Unified Gymnasium interface & registration utilities for Drone RL.
"""
from __future__ import annotations

from typing import Any

import gymnasium as gym
from gymnasium.envs.registration import register

from drone_sim_env import DroneSimEnv

__all__ = [
    "register_drone_env",
    "make_drone_env",
]


ENV_ID = "DroneSimEnv-v0"


def register_drone_env(max_episode_steps: int = 2000) -> None:
    """Register the DroneSimEnv with Gymnasium registry.

    This should be executed once (e.g. at the top-level of a training script)
    before `gym.make(ENV_ID)` is called.
    """
    # Unregister if already exists to avoid conflicts
    if ENV_ID in gym.registry:
        gym.registry.pop(ENV_ID)

    # Register with the correct format for gymnasium 1.0.0
    try:
        register(
            id=ENV_ID,
            entry_point=DroneSimEnv,
            max_episode_steps=max_episode_steps,
        )
        print(f"Successfully registered {ENV_ID}")
    except Exception as e:
        print(f"Failed to register {ENV_ID}: {e}")
        # Fallback: try with string entry point
        try:
            register(
                id=ENV_ID,
                entry_point="drone_sim_env:DroneSimEnv",
                max_episode_steps=max_episode_steps,
            )
            print(f"Successfully registered {ENV_ID} with string entry point")
        except Exception as e2:
            print(f"Failed to register {ENV_ID} with string entry point: {e2}")


def make_drone_env(**kwargs: Any) -> gym.Env:
    """Convenience factory that returns a freshly initialised **DroneSimEnv**.

    Parameters are forwarded to :class:`DroneSimEnv`.
    """
    return DroneSimEnv(**kwargs)
