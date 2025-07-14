# drone-msgs

ROS 2 message definitions for drone avoidance reinforcement learning.

## Overview

This repository contains ROS 2 message packages that define the interface between simulation components and reinforcement learning agents.

## Packages

### drone_msgs
High-level command and state messages used by bridge nodes and RL agents.

- `DroneControlCommand.msg` - Commands for drone control (thrust, roll, pitch, yaw)
- `DroneState.msg` - Current drone state (position, velocity, attitude, etc.)

### px4_msgs  
PX4-specific uORB ↔ ROS 2 message mappings (220+ messages).

Contains all PX4 uORB message definitions converted to ROS 2 format for seamless integration with PX4 SITL.

## Building

```bash
# Clone and build
git clone <this-repo>
cd drone-msgs
colcon build

# Source the workspace
source install/setup.bash
```

## Dependencies

- ROS 2 Humble
- ament_cmake
- ament_cmake_auto

## Versioning

This repository follows [Semantic Versioning](https://semver.org/):

- **v1.0.0** - Initial release with drone_msgs and px4_msgs
- Future releases will maintain backward compatibility for patch/minor versions

## Usage

### As a Dependency

Add to your `package.xml`:
```xml
<depend>drone_msgs</depend>
<depend>px4_msgs</depend>
```

### In Python
```python
from drone_msgs.msg import DroneControlCommand, DroneState
from px4_msgs.msg import VehicleCommand, VehicleStatus
```

### In C++
```cpp
#include "drone_msgs/msg/drone_control_command.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
```

## Development

### Adding New Messages

1. Create `.msg` file in appropriate package
2. Update `CMakeLists.txt` and `package.xml`
3. Build and test
4. Update version and release

### Testing

```bash
# Build and run tests
colcon build --packages-select drone_msgs px4_msgs
colcon test --packages-select drone_msgs px4_msgs
```

## Contributing

1. Fork the repository
2. Create feature branch
3. Make changes with tests
4. Submit pull request

## License

[LICENSE](../LICENSE) - Same as parent repository 