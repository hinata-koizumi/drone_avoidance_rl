# Drone Avoidance RL Stack

[![CI](https://github.com/yourname/drone_avoidance_rl/actions/workflows/ci.yml/badge.svg)](https://github.com/yourname/drone_avoidance_rl/actions)  
📘 [日本語版はこちら](README.ja.md)

---

## Overview

- **PX4 SITL + ROS 2 Humble + Gazebo Garden + RL (Gym API) Unified Stack**
- **Reproducibility**: Ubuntu 22.04, ROS 2 Humble, PX4 v1.15, Gazebo Garden, multi-stage Docker
- **CI/CD Automation**: GitHub Actions for build, test, static analysis, E2E, coverage, security, release notes
- **Custom model/airframe support**
- **Type safety, code quality gates, auto-generated docs**

---

## Directory Layout

```
drone_avoidance_rl/
├── docker/          # Dockerfiles & entrypoints
├── src/             # ROS 2 nodes, Gym env, custom msgs
├── custom_model/    # Replaceable SDF model
├── custom_airframes/# PX4 airframe JSON
├── tests/           # Integration/E2E tests (pytest)
├── docs/            # mkdocs/Sphinx auto-docs
├── .github/         # CI/CD, PR templates, etc.
└── tools/           # Dev helper scripts
```

---

## Prerequisites

- Docker Desktop 4.30+ (BuildKit enabled)
- 12GB+ RAM
- macOS 12+, Linux, Windows (WSL2)
- Apple Silicon (arm64) / x86_64 supported
- (Optional) Apple M-series or NVIDIA CUDA 12

---

## Quick Start

```bash
# 1. Clone & submodules
git clone https://github.com/yourname/drone_avoidance_rl.git
cd drone_avoidance_rl
git submodule update --init --recursive

# 2. (Optional) Insert your own drone model & airframe
cp -r ~/my_drone_sdf      custom_model/drone_model
cp    ~/4500_my_drone.json custom_airframes/

# 3. Build & launch (CPU)
docker compose --profile cpu up -d --build

# 4. (Optional) Apple GPU
docker compose --profile gpu up -d --build

# 5. Stop
docker compose down
```

---

## Customization

- **Reward weights**: `REWARD_ORI`, `REWARD_POS`, `REWARD_SMOOTH` env vars (see `src/gym_env.py`)
- **Domain randomization**: extend `DroneSimEnv._randomize_world()`
- **PX4 parameters**: edit JSON in `custom_airframes/`
- **Telemetry**: forward UDP 14550 to QGroundControl

---

## Testing & CI

- **Full CI/CD**: GitHub Actions for build, test, static analysis, E2E, coverage, security scan, release notes
- **Local test example**:
  ```bash
  tools/clean_workspace.sh
  docker build -t drone_rl:unified -f docker/Dockerfile.unified .
  docker run --rm drone_rl:unified ros2 launch sim_launch sim_all.launch.py
  python3 -m pip install pytest gymnasium numpy pyyaml lark
  cd src && PYTHONPATH=$PYTHONPATH:$(pwd) pytest ../tests/test_gym_api.py
  python3 -m pip install ruff mypy
  ruff src/ tests/
  mypy src/ tests/
  ```

---

## Documentation

- **Auto-generated docs**: [docs/](docs/) (mkdocs structure)
- **GitHub Pages auto-publish**
- Architecture, dev flow, FAQ, troubleshooting, etc.

---

## Contribution

- PR template & CONTRIBUTING.md required
- Code quality gates (ruff, mypy, ament_lint_auto) required
- Semantic Versioning
- See [docs/](docs/) for details

---

## License

Apache License 2.0 — see `LICENSE`.

---

*Contributions and issues are welcome!*
