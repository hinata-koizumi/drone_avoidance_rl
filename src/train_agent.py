#!/usr/bin/env python3
# ────────────────────────────────────────────
# train_agent.py   ★CHANGE A-3
#   * SAC に policy/env/device を明示
# ────────────────────────────────────────────
import signal

import torch
from gymnasium.wrappers import RecordEpisodeStatistics
from stable_baselines3 import SAC

from gym_env import DroneSimEnv

if __name__ == "__main__":
    env = DroneSimEnv(
        wind_max=3.0,
        gust_max=6.0,
        gust_prob=0.05,
        episode_max_steps=2_000,
    )
    env = RecordEpisodeStatistics(env)

    device = (
        "cuda" if torch.cuda.is_available() else
        "mps"  if torch.backends.mps.is_available() else
        "cpu"
    )
    print(f"[train_agent] device = {device}")

    model = SAC("MlpPolicy", env, verbose=1, device=device)   # ★Fix

    def _term_handler(*_: object) -> None:
        print("[train_agent] SIGTERM – saving checkpoint")
        model.save("sac_inner_prop_model_partial")
        env.close()
        exit(0)

    signal.signal(signal.SIGTERM, _term_handler)

    model.learn(
        total_timesteps=1_000_000,
        log_interval=1,
        progress_bar=True,
        callback=lambda _l, _g: model.save("sac_inner_prop_model_ckpt")
        if (_l["n_calls"] % 100_000 == 0) else True,
    )
    model.save("sac_inner_prop_model")
    env.close()
