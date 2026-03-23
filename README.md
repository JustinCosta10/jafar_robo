# Advanced Robotics Final Project

**TEAM UNDERQUALIFIED**

jafar_robo

## How to Run

**Launch rover control:**
`ros2 launch robo_rover rover_launch.py`

**Echo Accelerometer from IMU:**
`ros2 topic echo /imu/accel`

**Echo Gyroscope from IMU:**
`ros2 topic echo /imu/gyro`

**Run Keyboard Control Node:**
`ros2 run teleop_twist_keyboard teleop_twist_keyboard`

**Run LIDaR:**
`ros2 launch rplidar_ros rplidar_a1_launch.py`

## Packages Used

1. `ROBO_rover`: integrated control and IMU data publishing for a Pixhawk 4 Mini running ArduPilot Rover firmware.
1. `ack_control`: Ackermann steering control for the rover
1. `rplidar_ros`: LIDAR integration
