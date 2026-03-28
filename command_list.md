Launch rover control:

ros2 launch robo_rover rover_launch.py

Echo Accelerometer from IMU:
ros2 topic echo /imu/accel

Echo Gyroscope from IMU:
ros2 topic echo /imu/gyro

Run Keyboard Control Node:
ros2 run teleop_twist_keyboard teleop_twist_keyboard

Run LIDaR:
ros2 launch rplidar_ros rplidar_a1_launch.py

Check LIDar Scan:
ros2 topic echo /scan

Run Realsense: (https://github.com/realsenseai/realsense-ros)
ros2 run realsense2_camera realsense2_camera_node

Follow Green Nodes:
ros2 run green_control green_control
ros2 run green_vision green_vision

ros2 launch launch/green_follow_launch.py


