# DroneControlCommand.msg
# High-level control commands for drone

# Header with timestamp
std_msgs/Header header

# Thrust (0.0 to 1.0)
float32 thrust

# Roll rate (-1.0 to 1.0)
float32 roll_rate

# Pitch rate (-1.0 to 1.0)
float32 pitch_rate

# Yaw rate (-1.0 to 1.0)
float32 yaw_rate 