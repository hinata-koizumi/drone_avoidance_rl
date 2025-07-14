# DroneControlCommand.msg
# Drone control command

Header header

# Thrust command (0.0 to 1.0)
float32 thrust

# Roll command (-1.0 to 1.0)
float32 roll

# Pitch command (-1.0 to 1.0)
float32 pitch

# Yaw rate command (-1.0 to 1.0)
float32 yaw_rate

# Control mode
string control_mode 