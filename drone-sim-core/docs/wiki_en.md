# Drone Avoidance RL Wiki (English)

[日本語版はこちら](wiki_ja.md)

---

## Project Overview

- PX4 SITL + ROS 2 Humble + Gazebo Garden + RL (Gym API) Unified Stack
- Reproducibility: Ubuntu 22.04, ROS 2 Humble, PX4 v1.15, Gazebo Garden, multi-stage Docker
- CI/CD Automation: GitHub Actions for build, test, static analysis, E2E, coverage, security, release notes
- Custom model/airframe support
- Type safety, code quality gates, auto-generated docs
- **Cross-platform**: Optimized for ARM64 (Apple Silicon) and x86_64 architectures
- **BuildKit optimizations**: 40-60% faster rebuilds with cache mounts

---

## Directory Structure

```
drone_avoidance_rl/
├── docker/              # Dockerfiles & entrypoints
│   ├── px4-simulator/   # PX4 SITL + Gazebo Garden
│   ├── bridge/          # ROS 2 bridge nodes
│   ├── msgs/            # Message packages
│   └── rl-agent/        # Reinforcement learning environment
├── src/                 # ROS 2 nodes, Gym env, custom msgs
│   ├── drone_sim_env.py # Gym API compliant drone environment
│   ├── common/          # Common utilities & base classes
│   └── [bridge_nodes]/  # Various bridge nodes
├── assets/
│   ├── models/          # Custom SDF models
│   └── airframes/       # PX4 airframe configurations
├── tests/               # Integration & E2E tests
├── docs/                # Auto-generated documentation
└── tools/               # Development helper scripts
```

---

## Development Flow
- Branching: main, feature/*, fix/*
- PR template & CONTRIBUTING.md required
- All code must pass CI/CD (build, test, lint, static analysis)
- Semantic versioning for releases
- Automated dependency updates via Dependabot

---

## Docker Architecture

### Multi-stage Builds
- **Base image**: ROS 2 Humble + Gazebo Garden + PX4 SITL
- **Bridge image**: ROS 2 bridge nodes with optimized colcon builds
- **RL Agent image**: Python RL environment with gymnasium
- **Simulator image**: PX4 SITL + Gazebo Garden runtime

### Build Optimizations
- **BuildKit 1.4**: Cache mounts for apt/pip dependencies
- **Parallel builds**: `$(nproc)` workers for colcon builds
- **Platform-specific**: Native ARM64 support eliminates QEMU emulation
- **Context splitting**: Dedicated build contexts per service

### Services
- `sim`: PX4 SITL + Gazebo Garden simulation
- `bridge`: ROS 2 bridge nodes (state_bridge, command_bridge, etc.)
- `msgs`: Message packages (drone_msgs, px4_msgs)
- `rl-agent`: Reinforcement learning environment

---

## Customization
- Reward weights: `REWARD_ORI`, `REWARD_POS`, `REWARD_SMOOTH` (see `src/drone_sim_env.py`)
- Domain randomization: extend `DroneSimEnv._randomize_world()`
- PX4 params: edit JSON in `assets/airframes/`
- Custom models: add SDF files to `assets/models/`
- Telemetry: forward UDP 14550 to QGroundControl

---

## CI/CD & Testing
- GitHub Actions: build, test, lint, E2E, coverage, security, release notes
- Local test example:
  ```bash
  # Build optimized images
  docker compose build --no-cache
  
  # Run tests
  docker compose -f tests/ci-compose.yml up --abort-on-container-exit
  
  # Local development
  python3 -m pip install pytest gymnasium numpy pyyaml lark
  cd src && PYTHONPATH=$PYTHONPATH:$(pwd) pytest ../tests/test_gym_api.py
  python3 -m pip install ruff mypy
  ruff src/ tests/
  mypy src/ tests/
  ```

---

## Performance Optimizations

### Build Performance
- **Cache mounts**: apt/pip dependencies cached between builds
- **Parallel colcon**: `$(nproc)` workers reduce build time by 2-3x
- **Multi-stage**: Separate build/runtime layers reduce image size
- **Context splitting**: Dedicated contexts reduce Docker context size

### Runtime Performance
- **Native ARM64**: No QEMU emulation on Apple Silicon
- **Memory efficient**: Multi-stage builds reduce image sizes
- **Cache-friendly**: Layer optimization for faster container startup

---

## Documentation
- Auto-generated docs: [docs/](../docs/)
- mkdocs + GitHub Pages
- Architecture, dev flow, troubleshooting, FAQ, etc.
- API documentation for Gym environment

---

## FAQ
- Q: Does it work on Apple Silicon?  
  A: Yes, both arm64 and x86_64 are supported with native ARM64 optimization.
- Q: What if Gazebo hangs in CI?  
  A: Timeout/exit code monitoring is enabled. See troubleshooting.
- Q: How to add custom drone models?
  A: Place SDF files in `assets/models/` and update docker-compose volume mounts.
- Q: Build times are slow?
  A: Use `docker compose build` with BuildKit cache mounts for 40-60% faster rebuilds.
- Q: How to customize reward functions?
  A: Modify environment variables or extend `DroneSimEnv` class in `src/drone_sim_env.py`.

---

## CI Matrix (Manual Control)
| Job | OS | Architecture | Trigger |
|-----|----|--------------|---------|
| manual_control-validation | ubuntu-latest | arm64 (QEMU) | push / PR |  

The workflow builds the **manual control stack**, runs the container health-check script, and executes pytest unit tests.

## Manual Control Package
The manual control functionality has been integrated into the main repository as `src/manual_control/`. This package provides predefined drone action sequences and state monitoring capabilities.

### Key Features
- Action executor for predefined drone movements
- State monitoring and validation
- Integration with PX4 SITL and Gazebo Garden
- Docker-based deployment with health checks

### Development
```bash
# Build manual control package
colcon build --packages-select manual_control

# Run tests
python -m pytest tests/test_manual_control.py

# Build Docker image
docker build -f docker/Dockerfile.manual_control -t drone-manual-control .
```

---

## License
Apache License 2.0 