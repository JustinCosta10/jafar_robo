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
