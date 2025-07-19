# DroneState.msg
# Current state of the drone

# Position (x, y, z)
geometry_msgs/Point position

# Orientation (roll, pitch, yaw)
geometry_msgs/Vector3 orientation

# Velocity (vx, vy, vz)
geometry_msgs/Vector3 velocity

# Battery level (0.0 to 1.0)
float32 battery_level

# Timestamp
time stamp 