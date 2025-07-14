"""test_gym_api.py
Basic tests for the Gymnasium environment API.
"""
import pytest
import numpy as np
import gymnasium as gym

from gym_env import register_drone_env, create_env, ENV_ID


def test_environment_registration():
    """Test that the environment can be registered and created."""
    register_drone_env()
    
    # Should be able to make the environment using create_env
    env = create_env()
    assert env is not None
    assert hasattr(env, 'reset')
    assert hasattr(env, 'step')
    assert hasattr(env, 'observation_space')
    assert hasattr(env, 'action_space')
    
    env.close()


def test_environment_reset():
    """Test that environment reset works correctly."""
    register_drone_env()
    env = create_env()
    
    obs, info = env.reset()
    
    # Check observation shape
    assert isinstance(obs, np.ndarray)
    assert obs.shape == env.observation_space.shape
    assert obs.dtype == env.observation_space.dtype
    
    env.close()


def test_environment_step():
    """Test that environment step works correctly."""
    register_drone_env()
    env = create_env()
    
    obs, info = env.reset()
    
    # Take a random action
    action = env.action_space.sample()
    next_obs, reward, terminated, truncated, info = env.step(action)
    
    # Check return types
    assert isinstance(next_obs, np.ndarray)
    assert isinstance(reward, (int, float))
    assert isinstance(terminated, bool)
    assert isinstance(truncated, bool)
    assert isinstance(info, dict)
    
    # Check observation shape
    assert next_obs.shape == env.observation_space.shape
    
    env.close()


def test_action_space():
    """Test that action space is correctly defined."""
    register_drone_env()
    env = create_env()
    
    # Check action space properties
    assert isinstance(env.action_space, gym.spaces.Box)
    assert env.action_space.shape == (2,)  # thrust, yaw_rate
    assert env.action_space.dtype == np.float32
    
    # Test sampling
    action = env.action_space.sample()
    assert action.shape == (2,)
    assert np.all(action >= env.action_space.low)
    assert np.all(action <= env.action_space.high)
    
    env.close()


def test_observation_space():
    """Test that observation space is correctly defined."""
    register_drone_env()
    env = create_env()
    
    # Check observation space properties
    assert isinstance(env.observation_space, gym.spaces.Box)
    assert env.observation_space.shape == (4,)  # x, y, z, yaw
    assert env.observation_space.dtype == np.float32
    
    env.close()


if __name__ == "__main__":
    pytest.main([__file__]) 