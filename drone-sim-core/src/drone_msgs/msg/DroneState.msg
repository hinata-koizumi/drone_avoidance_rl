# DroneState.msg
# Drone state information
import std_msgs/Header.msg

Header header

# Position (x, y, z) in meters
geometry_msgs/Point position

# Orientation (roll, pitch, yaw) in radians
geometry_msgs/Vector3 orientation

# Velocity (vx, vy, vz) in m/s
geometry_msgs/Vector3 velocity

# Angular velocity (wx, wy, wz) in rad/s
geometry_msgs/Vector3 angular_velocity

# Battery level (0.0 to 1.0)
float32 battery_level

# Flight mode
string flight_mode 