# Drone state message
float32 roll    # Roll angle [rad]
float32 pitch   # Pitch angle [rad]
float32 yaw     # Yaw angle [rad]
geometry_msgs/Vector3 position         # Position in world frame [m]
geometry_msgs/Vector3 velocity         # Velocity in world frame [m/s]
geometry_msgs/Vector3 angular_velocity # Angular velocity [rad/s]
geometry_msgs/Vector3 wind            # Wind vector [m/s]
