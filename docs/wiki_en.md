# Drone Avoidance RL Wiki (English)

[日本語版はこちら](wiki_ja.md)

---

## Project Overview

- Unified stack: PX4 SITL, ROS 2 Humble, Gazebo Garden, RL (Gym API)
- Reproducible, multi-stage Docker, CI/CD, custom model/airframe support

---

## Directory Structure

(see README.md)

---

## Development Flow
- Branching: main, feature/*, fix/*
- PR template & CONTRIBUTING.md required
- All code must pass CI/CD (build, test, lint, static analysis)

---

## Customization
- Reward weights: `REWARD_ORI`, `REWARD_POS`, `REWARD_SMOOTH` (see `src/gym_env.py`)
- Domain randomization: extend `DroneSimEnv._randomize_world()`
- PX4 params: edit JSON in `custom_airframes/`
- Telemetry: forward UDP 14550 to QGroundControl

---

## CI/CD & Testing
- GitHub Actions: build, test, lint, E2E, coverage, security, release notes
- Local test example:
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
- Auto-generated docs: [docs/](../docs/)
- mkdocs + GitHub Pages
- Architecture, dev flow, troubleshooting, FAQ, etc.

---

## FAQ
- Q: Does it work on Apple Silicon?  
  A: Yes, both arm64 and x86_64 are supported (depends on official binaries).
- Q: What if Gazebo hangs in CI?  
  A: Timeout/exit code monitoring is enabled. See troubleshooting.

---

## License
Apache License 2.0 