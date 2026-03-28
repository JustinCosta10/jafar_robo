#!/usr/bin/env python3
"""
Launch file for green paper following behavior.

Launches: realsense camera -> green_vision -> green_control -> rover_node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Rover args
    connection_string = LaunchConfiguration("connection_string", default="/dev/ttyACM1")
    baud_rate = LaunchConfiguration("baud_rate", default="115200")

    # RealSense camera launch
    realsense_share = get_package_share_directory("realsense2_camera")
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "pointcloud.enable": "false",
        }.items(),
    )

    return LaunchDescription([
        # --- Launch arguments ---
        DeclareLaunchArgument(
            "connection_string",
            default_value="/dev/ttyACM1",
            description="MAVLink connection string for the rover",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="115200",
            description="Baud rate for the rover serial connection",
        ),

        # --- Nodes ---
        # RealSense D435 camera
        realsense_launch,

        # Green paper detection
        Node(
            package="green_vision",
            executable="green_vision",
            name="green_vision",
            output="screen",
        ),

        # Green paper following controller
        Node(
            package="green_control",
            executable="green_control",
            name="green_control",
            output="screen",
        ),

        # Rover driver
        Node(
            package="robo_rover",
            executable="rover_node",
            name="rover_node",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "connection_string": connection_string,
                "baud_rate": baud_rate,
                "control_frequency": 20.0,
                "imu_frequency": 20.0,
            }],
        ),
    ])
