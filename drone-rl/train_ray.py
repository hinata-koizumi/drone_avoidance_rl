"""train_ray.py
Minimal Ray RLlib training script for the registered DroneSimEnv.

Usage (local):
    python train_ray.py --num-workers 4 --env-id DroneSimEnv-v0 --train-iterations 10

The script expects that DroneSimEnv has been registered via gym_env.register_drone_env().
"""
from __future__ import annotations

import argparse
from pathlib import Path

import ray
from ray import air, tune
from ray.rllib.algorithms.ppo import PPOConfig

# Local imports
from gym_env import register_drone_env, ENV_ID


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train PPO on DroneSimEnv with Ray RLlib.")
    parser.add_argument("--num-workers", type=int, default=2, help="Number of rollout workers.")
    parser.add_argument("--train-iterations", type=int, default=10, help="Number of training iterations.")
    parser.add_argument("--env-id", type=str, default=ENV_ID, help="Gymnasium Env ID (already registered)")
    parser.add_argument(
        "--output", type=Path, default=Path("./rllib_results"), help="Directory to store checkpoints & logs."
    )
    parser.add_argument("--stop-reward", type=float, default=None, help="Stop when mean reward reaches value.")
    parser.add_argument("--target-pos", type=float, nargs=3, default=[5.0,0.0,4.0], help="Target position [x y z]")
    parser.add_argument("--init-pos", type=float, nargs=3, default=[0.0,0.0,4.0], help="Initial position [x y z]")
    parser.add_argument("--randomize", action="store_true", help="Enable domain randomization")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    # Register environment and initialise Ray.
    register_drone_env()
    ray.init(ignore_reinit_error=True)

    env_config = {
        "target_pos": args.target_pos,
        "init_pos": args.init_pos,
    }
    if args.randomize:
        env_config["randomization_params"] = {
            "init_pos_range": [[-2, -2, 3.5], [2, 2, 4.5]],
            "target_pos_range": [[4, -2, 3.5], [6, 2, 4.5]],
            "mass_range": [0.6, 0.7],
            "wind_range": [[-1, -1, 0], [1, 1, 0]],
        }

    config = (
        PPOConfig()
        .environment(env=args.env_id, env_config=env_config)
        .framework("torch")
        .rollouts(num_rollout_workers=args.num_workers)
    )

    stopping_criteria = {
        "training_iteration": args.train_iterations,
    }
    if args.stop_reward is not None:
        stopping_criteria["episode_reward_mean"] = args.stop_reward

    tuner = tune.Tuner(
        "PPO",
        param_space=config.to_dict(),
        run_config=air.RunConfig(local_dir=str(args.output), stop=stopping_criteria),
    )

    tuner.fit()
    print("Training complete. Results stored in", args.output)


if __name__ == "__main__":
    main() 