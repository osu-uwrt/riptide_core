# Minimal IMU Message
#
# We won't take the estimated orientation from IMU because that data
# comes from the accelerometer and gyroscope measurments. Using that data
# would overrepresent the IMU and potentially cause issues when optimizing.
#
# Also minimizing message size should help with ROS latency issues that we have been having.

float64 delta_time

geometry_msgs/Vector3 angular_velocity

geometry_msgs/Vector3 linear_acceleration
