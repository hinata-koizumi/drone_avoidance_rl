# drone-sim-core

PX4 SITL + Ignition Gazebo Garden simulation environment for drone reinforcement learning.

## Overview

This repository contains all components required to run a PX4-based drone simulation with ROS 2 Humble and Ignition Gazebo Garden. It provides the simulation foundation that RL agents can interact with via ROS 2 topics.

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   PX4 SITL     │    │  Ignition      │    │  Bridge Nodes   │
│   (Flight      │◄──►│  Gazebo        │◄──►│  (ROS 2 ↔ PX4) │
│   Controller)   │    │  (Physics)     │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                │
                                ▼
                       ┌─────────────────┐
                       │  ROS 2 Topics  │
                       │  (Interface)    │
                       └─────────────────┘
```

## Components

### Simulation Core
- **PX4 SITL**: Flight controller simulation
- **Ignition Gazebo Garden**: Physics engine and world simulation
- **Custom Models**: Drone and environment assets

### Bridge Nodes
- `angle_bridge`: Angle command translation
- `command_bridge`: High-level command routing
- `state_bridge`: PX4 state to ROS 2 conversion
- `outer_motor_bridge`: Motor control interface
- `common`: Shared bridge utilities

### Launch & Configuration
- `sim_launch`: ROS 2 launch files for simulation
- `config/`: Simulation parameters and settings
- `custom_airframes/`: PX4 airframe definitions
- `custom_model/`, `models/`: SDF model assets

### Manual Control
- `manual_control`: Web-based manual control interface
- Web UI accessible at `http://localhost:8080`

## Quick Start

### Prerequisites
- Docker and Docker Compose
- NVIDIA GPU (optional, for GPU acceleration)

### Basic Simulation
```bash
# Clone and navigate
git clone <this-repo>
cd drone-sim-core

# Build images
make build

# Start simulation
make sim

# Check status
make status
```

### Manual Control
```bash
# Start with manual control
make manual

# Access web UI
open http://localhost:8080
```

### Development
```bash
# Run tests
make test

# Clean up
make clean
```

## Docker Profiles

### Default Profile
- PX4 SITL + Gazebo + Bridge nodes
- Manual control (optional)

### Test Profile
- Lightweight simulation for CI/CD
- Reduced physics and shorter timeouts

### GPU Profile (Experimental)
- GPU-accelerated simulation
- Requires NVIDIA Docker

## Configuration

### Environment Variables
Create `.env` file:
```bash
ROS_DISTRO=humble
IGNITION_VERSION=garden
GPU_COUNT=1
CUDA_VISIBLE_DEVICES=0
```

### Simulation Parameters
Edit `config/sim_params.yaml`:
```yaml
simulation:
  headless: true
  physics_rate: 1000
  real_time_factor: 1.0
```

## Development

### Adding New Bridge Nodes
1. Create package in `src/`
2. Inherit from `common.bridge_base.BridgeBase`
3. Add to launch files
4. Update Docker build

### Custom Models
1. Add SDF files to `models/` or `custom_model/`
2. Update launch files to reference new models
3. Test with `make sim`

### Testing
```bash
# Unit tests
make test-fast

# Integration tests
make test

# Performance tests
make perf-test
```

## Troubleshooting

### Common Issues
- **Gazebo not starting**: Check GPU drivers and Docker permissions
- **PX4 connection failed**: Verify network configuration and ports
- **Bridge nodes not found**: Ensure workspace is sourced correctly

### Logs
```bash
# View all logs
make logs

# View specific service
docker compose logs sim
docker compose logs bridge
```

## Dependencies

### External
- ROS 2 Humble
- PX4 Firmware (SITL)
- Ignition Gazebo Garden

### Internal
- `drone-msgs`: Message definitions
- `drone-rl`: Reinforcement learning agents (optional)

## Contributing

1. Fork the repository
2. Create feature branch
3. Add tests for new functionality
4. Submit pull request

## License

[LICENSE](../LICENSE) - Same as parent repository 