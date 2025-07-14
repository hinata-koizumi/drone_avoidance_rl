import numpy as np
import pytest

pytest.importorskip("gymnasium")

# Gym環境のimport
try:
    from drone_sim_env import DroneSimEnv
except ImportError:
    pytest.skip(
        "DroneSimEnvがimportできません。src/gym_env.pyのパスや依存を確認してください。",
        allow_module_level=True
    )


def test_reset_step():
    env = DroneSimEnv(episode_max_steps=10)
    obs, info = env.reset()
    assert isinstance(obs, np.ndarray)
    assert isinstance(info, dict)
    action = np.zeros(4, dtype=np.float32)
    obs2, reward, terminated, truncated, info2 = env.step(action)
    assert isinstance(obs2, np.ndarray)
    assert isinstance(reward, float)
    assert isinstance(terminated, bool)
    assert isinstance(truncated, bool)
    assert isinstance(info2, dict)
    env.close() 