# Drone Avoidance RL Stack

[![CI](https://github.com/hinata-koizumi/drone_avoidance_rl/actions/workflows/ci.yml/badge.svg)](https://github.com/hinata-koizumi/drone_avoidance_rl/actions)  
📘 [日本語版はこちら](README.ja.md)

---

## Overview

PX4 SITL + ROS 2 Humble + Gazebo Garden + Reinforcement Learning (Gym API) unified stack.

**This project aims to realize safe and reliable drone deployment in disaster scenes in Japan, a disaster-prone country.** We develop autonomous flight technology through reinforcement learning, targeting the practical application of drones that contribute to search and rescue activities and damage assessment during disasters.

### Features
- **Reproducibility**: Ubuntu 22.04, ROS 2 Humble, PX4 v1.15, Gazebo Garden (LTS)
- **CI/CD**: Automated build, test, and static analysis with GitHub Actions
- **Customization**: Custom drone model and airframe support
- **Type Safety**: Code quality management with mypy and ruff
- **Multi-stage Docker**: Efficient build and deployment

---

## Project Structure

```
drone_avoidance_rl/
├── docker/              # Dockerfiles & entrypoints
├── src/                 # ROS 2 nodes, Gym env, custom msgs
│   ├── drone_sim_env.py # Gym API compliant drone environment
│   ├── common/          # Common utilities & base classes
│   └── [bridge_nodes]/  # Various bridge nodes
├── custom_model/        # Custom SDF models
├── custom_airframes/    # PX4 airframe configurations
├── tests/               # Integration & E2E tests
├── docs/                # Auto-generated documentation
└── tools/               # Development helper scripts
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

### 1. Clone Repository
```bash
git clone https://github.com/hinata-koizumi/drone_avoidance_rl.git
cd drone_avoidance_rl
git submodule update --init --recursive
```

### 2. (Optional) Add Custom Models
```bash
# Add your own drone model
cp -r ~/my_drone_sdf      custom_model/drone_model
cp    ~/4500_my_drone.json custom_airframes/
```

### 3. Build & Launch
```bash
# CPU version
docker compose --profile cpu up -d --build

# Apple GPU version (M1/M2)
docker compose --profile gpu up -d --build
```

### 4. Stop
```bash
docker compose down
```

---

## Local Development Environment

### Python Environment Setup
```bash
# Install dependencies
python3 -m pip install --upgrade pip
pip install -r requirements.txt
```

### Run Tests
```bash
# Set environment variables
export PYTHONPATH=$(pwd):$(pwd)/src

# Run tests
pytest tests/test_gym_api.py
pytest tests/test_rl_longrun.py
pytest tests/test_gym_env.py
```

### Static Analysis
```bash
ruff src/ tests/
mypy src/ tests/
```

---

## Gym API Specification

### Environment Specification
- **Observation Space**: 15-dimensional (attitude, position, velocity, angular velocity, wind)
- **Action Space**: 4-dimensional (throttle and angle for 2 motors)
- **Reward Function**: Weighted sum of REWARD_ORI, REWARD_POS, REWARD_SMOOTH

### Usage Example
```python
from drone_sim_env import DroneSimEnv
from stable_baselines3 import SAC

# Create environment
env = DroneSimEnv(reward_mode="hover", episode_max_steps=1000)

# Training
model = SAC("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=100_000)
```

### Reward Modes
- `hover`: Hovering focused
- `path_follow`: Path following
- `obstacle_avoid`: Obstacle avoidance
- `default`: Traditional (adjust weights via environment variables)

---

## Customization

### Adjust Reward Weights
Modify reward weights using environment variables:
```bash
export REWARD_ORI=1.0
export REWARD_POS=0.5
export REWARD_SMOOTH=0.1
```

### Domain Randomization
Extend `DroneSimEnv._randomize_world()` for environment randomization.

### PX4 Parameters
Edit JSON files in `custom_airframes/` to adjust PX4 parameters.

---

## Testing with CI/CD Environment

### Docker Environment Testing
```bash
# Test with same environment and commands as CI
docker compose build --no-cache
bash tools/setup_rosdep_local.sh
docker compose -f tests/ci-compose.yml up --abort-on-container-exit
```

### Local Environment Testing
```bash
# Recommend Python 3.10.12
pyenv install 3.10.12
pyenv local 3.10.12

# Install dependencies
python3 -m pip install --upgrade pip
python3 -m pip install pytest gymnasium numpy pyyaml lark ruff mypy types-PyYAML

# Run tests
cd src
PYTHONPATH=$PYTHONPATH:$(pwd) pytest ../tests/test_gym_api.py
ruff src/ tests/
mypy src/ tests/
```

---

## Main ROS 2 Nodes & Topics

- `/drone{N}/inner_propeller_cmd` (DroneControlCommand)
- `/drone{N}/state` (DroneState)

Topic names can be changed via launch files or node parameters.

---

## Version Management

- ROS 2 and Gazebo/Ignition versions are centrally managed in `.env` files
- Package.xml version consistency is automatically checked by `check_package_versions.sh`
- Dependabot automatically monitors and creates PRs for dependencies

---

## Documentation

- **Auto-generated docs**: [docs/](docs/) (mkdocs structure)
- **GitHub Pages auto-publish**
- Architecture, development flow, FAQ, troubleshooting

---

## Contributing

- PR template & CONTRIBUTING.md required
- Code quality gates (ruff, mypy, ament_lint_auto) required
- Semantic Versioning
- See [docs/](docs/) for details

---

## License

Apache License 2.0 — see `LICENSE`.

---

*Contributions and issues are welcome!*