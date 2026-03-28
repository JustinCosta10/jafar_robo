#!/usr/bin/env python3
"""
Launch file for green paper following behavior.

Launches: rs_stream -> green_vision -> green_control -> rover_node
Rover starts only after RealSense pipeline is ready (stdout event).
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessIO
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Rover args
    connection_string = LaunchConfiguration("connection_string", default="/dev/ttyACM1")
    baud_rate = LaunchConfiguration("baud_rate", default="115200")

    # --- Node definitions ---
    rs_node = Node(
        package="rs_stream",
        executable="rs_stream_node",
        name="rs_stream_node",
        output="screen",
    )

    rover_node = Node(
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
    )

    # Start rover only after RS prints "RealSense pipeline started."
    wait_for_rs = RegisterEventHandler(
        OnProcessIO(
            target_action=rs_node,
            on_stdout=lambda event: [rover_node]
            if b"RealSense pipeline started." in event.text
            else [],
        )
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
        # RealSense starts first on a quiet USB bus
        rs_node,

        # Vision and control can start immediately (no hardware)
        Node(
            package="green_vision",
            executable="green_vision",
            name="green_vision",
            output="screen",
        ),
        Node(
            package="green_control",
            executable="green_control",
            name="green_control",
            output="screen",
        ),

        # Rover starts only after RealSense pipeline is ready
        wait_for_rs,
    ])
