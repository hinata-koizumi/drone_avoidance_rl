# drone-rl

Reinforcement learning framework for drone avoidance using Ray RLlib.

## Overview

This repository contains the reinforcement learning stack for training drone avoidance agents. It provides a scalable RL environment that can communicate with the simulation core via ROS 2 topics.

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Ray RLlib    │    │  Gymnasium     │    │  ROS 2 Bridge   │
│   (Training)   │◄──►│  Environment   │◄──►│  (Topics)       │
│                 │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Components

### Core Environment
- `drone_sim_env.py`: Base Gymnasium environment for drone control
- `gym_env.py`: Environment registration and factory functions
- `vector_env_example.py`: Multi-environment vectorization example

### Training Framework
- `train_ray.py`: Ray RLlib training script with PPO
- `sample_agent/`: Example agents and evaluation scripts
- `docker/rl-agent/`: GPU-optimized training container
- `docker/rl-agent-cpu/`: Lightweight CPU container for CI

### Algorithms Supported
- **PPO** (Proximal Policy Optimization) - Primary algorithm
- **SAC** (Soft Actor-Critic) - Available via Ray RLlib
- **DQN** - Available via Ray RLlib
- **Custom algorithms** - Extensible framework

## Quick Start

### Prerequisites
- Python 3.10+
- Docker (for containerized training)
- NVIDIA GPU (optional, for GPU acceleration)

### Local Development
```bash
# Clone and setup
git clone <this-repo>
cd drone-rl

# Install dependencies
pip install -r requirements.txt

# Register environment
python -c "from gym_env import register_drone_env; register_drone_env()"

# Run training
python train_ray.py --num-workers 2 --train-iterations 10
```

### Docker Training
```bash
# Build GPU image
docker build -f docker/rl-agent/Dockerfile -t drone-rl:gpu .

# Run training
docker run --gpus all -v $(pwd)/results:/workspace/results drone-rl:gpu \
  python train_ray.py --num-workers 4 --train-iterations 100
```

### CI Testing
```bash
# Build CPU image for testing
docker build -f docker/rl-agent-cpu/Dockerfile -t drone-rl:cpu .

# Run mini test
docker run drone-rl:cpu
```

## Environment Interface

### Observation Space
```python
gym.spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32)
# [x, y, z, yaw] - drone position and orientation
```

### Action Space
```python
gym.spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
# [thrust, yaw_rate] - control commands
```

### Reward Function
Currently implements a simple reward structure:
- **Positive reward**: Staying within safe zones
- **Negative reward**: Collisions or boundary violations
- **Episode termination**: Max steps or collision

## Training Configuration

### Ray RLlib Settings
```python
config = (
    PPOConfig()
    .environment(env="DroneSimEnv-v0")
    .framework("torch")
    .rollouts(num_rollout_workers=4)
    .training(
        train_batch_size=4000,
        lr=3e-4,
        gamma=0.99,
        lambda_=0.95,
        kl_coeff=0.2,
        num_sgd_iter=30,
        sgd_minibatch_size=128,
    )
)
```

### Hyperparameter Tuning
```bash
# Use Ray Tune for hyperparameter optimization
python -c "
from ray import tune
from train_ray import main
tune.run(main, config={'lr': tune.loguniform(1e-4, 1e-3)})
"
```

## Docker Images

### GPU Training Image (`docker/rl-agent/`)
- Base: `drone-avoidance-base:2.0.1`
- CUDA 11.8 + PyTorch GPU
- Ray RLlib with GPU support
- NCCL for distributed training

### CPU Testing Image (`docker/rl-agent-cpu/`)
- Base: `python:3.10-slim`
- Minimal dependencies
- Fast CI testing
- 5-step validation

## Integration with Simulation

### ROS 2 Topics
The environment communicates with `drone-sim-core` via:
- `/drone0/state` - Drone state (position, velocity, etc.)
- `/drone0/command` - Control commands
- `/drone0/action` - High-level actions

### Message Types
```python
from drone_msgs.msg import DroneState, DroneControlCommand
from px4_msgs.msg import VehicleCommand, VehicleStatus
```

## Development

### Adding New Algorithms
1. Extend `train_ray.py` with new algorithm config
2. Add algorithm-specific hyperparameters
3. Update Docker images if needed
4. Add tests

### Custom Environments
1. Inherit from `gym.Env`
2. Implement `reset()`, `step()`, `render()`
3. Register with `gym.register()`
4. Update training scripts

### Testing
```bash
# Unit tests
python -m pytest tests/ -v

# Integration tests
docker compose --profile test up --abort-on-container-exit

# Performance tests
python vector_env_example.py
```

## Monitoring & Logging

### TensorBoard
```bash
# Start TensorBoard
tensorboard --logdir ./rllib_results

# View at http://localhost:6006
```

### Ray Dashboard
```bash
# Start Ray dashboard
ray start --head --dashboard-port=8265

# View at http://localhost:8265
```

## Performance Optimization

### GPU Training
- Use `CUDA_VISIBLE_DEVICES` to specify GPUs
- Set `NCCL_DEBUG=INFO` for network debugging
- Monitor GPU memory with `nvidia-smi`

### Distributed Training
- Use `ray start --head` for single-node cluster
- Use `ray start --address=<head-ip>` for multi-node
- Configure `num_rollout_workers` based on CPU cores

## Troubleshooting

### Common Issues
- **CUDA out of memory**: Reduce batch size or use gradient accumulation
- **Ray connection failed**: Check firewall and network configuration
- **Environment not found**: Ensure `register_drone_env()` was called

### Debugging
```bash
# Enable Ray debug logging
export RAY_BACKEND_LOG_LEVEL=debug

# Check GPU availability
python -c "import torch; print(torch.cuda.is_available())"

# Test environment
python -c "import gymnasium as gym; env=gym.make('DroneSimEnv-v0'); print(env.reset())"
```

## Dependencies

### External
- Ray RLlib 2.10.0
- PyTorch 2.0+
- Gymnasium 1.1.1
- ROS 2 Humble (via simulation)

### Internal
- `drone-msgs`: Message definitions
- `drone-sim-core`: Simulation environment

## Contributing

1. Fork the repository
2. Create feature branch
3. Add tests for new functionality
4. Submit pull request

## License

[LICENSE](../LICENSE) - Same as parent repository 