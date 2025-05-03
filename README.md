# README.md (English version)
[![CI](https://github.com/yourname/drone_avoidance_rl/actions/workflows/ci.yml/badge.svg)](https://github.com/yourname/drone_avoidance_rl/actions)  
📘 [日本語版はこちら](README.ja.md)
## Overview

* **Plug‑and‑play** sample stack for PX4 SITL, ROS 2, and reinforcement learning – drop your `model.sdf` + airframe JSON and start training.
* **Reproducible builds** – Ubuntu 22.04 snapshot, ROS 2 Humble, Ignition Fortress, PX4‑SITL v1.15 are fixed by explicit dates.
* **Multi‑stage Docker** splits the stack into *simulator*, *bridge*, and *RL agent*, enabling hot‑reload and Apple Silicon GPU (`--profile gpu`).
* **Continuous Integration** – GitHub Actions boots the full stack and runs `pytest` in < 10 min on every PR.

## Directory Layout

```
drone_avoidance_rl/
├── docker/          # Dockerfiles & entrypoints
├── src/             # ROS 2 nodes & Gym environment
├── drone_msgs/      # Custom ROS 2 message definitions
├── custom_model/    # Replaceable SDF model
├── custom_airframes/# PX4 airframe JSON
├── tests/           # Minimal integration tests (pytest)
└── logs/            # PX4 ulog / TensorBoard output
```

## Prerequisites

* Docker Desktop 4.30+ with BuildKit
* ≥ 12 GB RAM
* macOS 12+, Linux, or Windows (WSL2)
* Optional GPU: Apple Silicon (M‑series) or NVIDIA with CUDA 12

## Quick Start

```bash
# 1) Clone
$ git clone https://github.com/yourname/drone_avoidance_rl.git
$ cd drone_avoidance_rl

# 2) (optional) Insert your own drone model & airframe
$ cp -r ~/my_drone_sdf      custom_model/drone_model
$ cp    ~/4500_my_drone.json custom_airframes/

# 3) (Apple Silicon) create arm64 buildx
$ docker buildx create --name arm_builder --driver docker-container --use || true

# 4) Forward UDP 14556 & 11345 in Docker Desktop › Settings › Resources › Networking

# 5) Build & launch
$ docker compose --profile cpu up -d --build      # CPU‑only
$ docker compose --profile gpu up -d --build      # Apple GPU

# 6) Stop
$ docker compose down
```

All logs (PX4 ulog, TensorBoard, models) are stored in `logs/`.

## Customization

* **Reward weights** – set `REWARD_ORI`, `REWARD_POS`, `REWARD_SMOOTH` env vars (see `src/gym_env.py`).
* **Domain randomization** – extend `DroneSimEnv._randomize_world()` for mass/wind/battery variations.
* **PX4 parameters** – edit or add JSON files in `custom_airframes/` and adapt `SYS_AUTOSTART`.
* **Telemetry** – forward UDP 14550 to connect QGroundControl.

## Testing & CI

```bash
$ docker compose exec rl-agent pytest -q /work/tests
```

See `.drone.github/workflows/ci.yml` for the GitHub Actions workflow.

## License

Apache License 2.0 — see `LICENSE`.

---

*Contributions and issues are welcome!*
