# Drone Manual Control Environment

> **日本語**: [README.md](README.md)

This environment provides a manual control system for executing predefined actions on drones.
It is built by migrating reusable components from the original reinforcement learning environment.

## Dedicated Drone Specifications

### Aether-SL

A high-performance drone airframe specifically designed for the manual control environment.

#### **Basic Specifications**
- **Airframe Type**: Streamlined Quadcopter
- **Weight**: 0.65kg
- **Dimensions**: 32.5cm × 32.5cm × 7cm
- **Max Speed**: 23.6 m/s (85 km/h)
- **Max Altitude**: 50m
- **Flight Time**: ~25 minutes (simulation)
- **Max Range**: 20km

#### **Propulsion System**
- **Main Rotors**: 4 units (6-inch propellers, max 11000rpm)
- **Control Channels**: 4 channels (4 rotors)
- **Thrust Constant**: 1.6 (high-efficiency design)

#### **Sensor System**
- **IMU**: 9-axis (500Hz update rate, high precision)
- **GPS**: Dual-band GNSS (10Hz update rate)
- **Barometer**: Altitude measurement (60Hz update rate)
- **Magnetometer**: 3-axis (50Hz update rate)

#### **Safety Features**
- **Geofence**: 20km horizontal, 50m vertical limits
- **Failsafe**: Automatic return on RC signal loss
- **Speed Limits**: 23.6m/s horizontal, 8m/s vertical
- **Acceleration Limits**: Max 12m/s²

#### **Control Parameters**
- **Attitude Control**: PID control (Roll/Pitch: P=4.8, I=0.18, D=0.09)
- **Position Control**: PID control (XY: P=1.0, I=0.1, D=0.05)
- **Manual Control**: 80% tilt angle limit, 80% yaw angle limit

#### **Flight Modes**
- **STABILIZE**: Attitude stabilization mode
- **ALT_HOLD**: Altitude hold mode
- **LOITER**: Position hold mode
- **AUTO**: Autonomous flight mode
- **RTL**: Return to launch mode
- **MANUAL**: Manual control mode

See `config/drone_specs.yaml` for detailed specifications.

## Features

- **Predefined Action Execution**: Basic movements such as hovering, takeoff, landing, and trajectory following
- **ROS 2 Humble Support**: Uses the latest ROS 2 framework
- **Ignition Gazebo (Garden)**: High-precision physics simulation
- **Docker Integration**: Reproducible development environment
- **Modular Design**: Reuses existing bridge components

## Migrated Components

### Core Components
- `common/` - BridgeBase class and utilities
- `drone_msgs/` - Custom message definitions
- `px4_msgs/` - PX4 message definitions

### Simulation Environment
- `sim_launch/` - Gazebo Sim launch configurations
- `models/` - Drone models
- `custom_airframes/` - Airframe configurations

### Bridge Nodes
- `command_bridge/` - Control command conversion
- `state_bridge/` - State information conversion
- `angle_bridge/` - Angle control
- `outer_motor_bridge/` - External motor control

## New Components

### Manual Control System
- `manual_control/` - Predefined action execution node
- `action_sequences/` - Action sequence definitions
- `control_interface/` - Control interface

## Setup Instructions

### 1. Environment Initialization
```bash
# Copy components from existing environment
./scripts/setup_environment.sh
```

### 2. Environment Build
```bash
# Build Docker containers
docker-compose build
```

### 3. Demo Execution
```bash
# Run automatic demo
./scripts/run_demo.sh
```

### 4. Manual Execution (Optional)
```bash
# Start simulation
docker-compose up -d simulator

# Start bridge nodes
docker-compose up -d bridge

# Start manual control node
docker-compose up -d manual_control
```

## Usage

### Basic Usage
1. **Environment Setup**
```bash
cd drone_manual_control
./scripts/setup_environment.sh
docker-compose build
```

2. **Start Simulation**
```bash
docker-compose up -d
```

3. **Execute Manual Control**
```bash
docker-compose up -d manual_control
```

### Log Monitoring
```bash
# Manual control node logs
docker-compose logs -f manual_control

# All node logs
docker-compose logs -f
```

### Environment Shutdown
```bash
docker-compose down
```

## Predefined Actions

### Basic Movements
- **Hover**: Maintain hovering (10 seconds)
- **Takeoff**: Takeoff sequence (5 seconds)
- **Landing**: Landing sequence (8 seconds)

### Movement Actions
- **Waypoint Forward**: Forward movement 5m (15 seconds)
- **Waypoint Backward**: Backward movement 5m (15 seconds)
- **Waypoint Right**: Rightward movement 5m (12 seconds)
- **Waypoint Left**: Leftward movement 5m (12 seconds)

### Pattern Flight
- **Circle Flight**: Circular flight (radius 5m, 20 seconds)
- **Square Pattern**: Square pattern flight (40 seconds)

### Complex Sequences
- **Takeoff and Hover**: Takeoff → Hovering
- **Exploration Sequence**: Takeoff → Forward movement
- **Return to Base**: Return to base → Landing

## Configuration

### Modifying Action Sequences
Edit `config/action_sequences.yaml` to customize actions:

```yaml
action_sequences:
  - name: "custom_hover"
    action_type: "hover"
    duration: 15.0  # 15 seconds
    parameters:
      target_altitude: 5.0  # 5m altitude
    next_action: "landing"  # Next action
```

### Control Parameter Adjustment
```yaml
control_parameters:
  position_p_gain: 1.0
  position_i_gain: 0.1
  position_d_gain: 0.05
  max_velocity: 5.0  # m/s
```

### Safety Settings
```yaml
safety_parameters:
  min_altitude: 0.5  # m
  max_altitude: 50.0  # m
  max_distance_from_base: 100.0  # m
```