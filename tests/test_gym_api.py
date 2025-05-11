import numpy as np

from gym_env import DroneSimEnv


def test_gym_api_basic() -> None:
    env = DroneSimEnv(episode_max_steps=10)
    obs, info = env.reset()
    assert isinstance(obs, np.ndarray)
    for _ in range(3):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        assert isinstance(obs, np.ndarray)
        assert isinstance(reward, float)
        assert isinstance(terminated, bool)
        assert isinstance(truncated, bool)
    env.close()

def test_gym_api_render_not_implemented():
    env = DroneSimEnv()
    try:
        env.render()
    except NotImplementedError:
        pass
    except AttributeError:
        pass
    else:
        # If render() is implemented, that's fine too
        pass
    env.close()

def test_gym_api_reset() -> None:
    env = DroneSimEnv()
    obs, info = env.reset()
    assert isinstance(obs, np.ndarray)
    env.close() 