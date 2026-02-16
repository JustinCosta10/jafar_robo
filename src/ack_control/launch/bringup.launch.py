from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('ack_control')
    
    # Get URDF via xacro
    robot_description = Command(['xacro ', os.path.join(pkg_path, 'description', 'robot.urdf.xacro')])

    return LaunchDescription([
        # 1. Robot State Publisher
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),

        # 2. Controller Manager
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[{'robot_description': robot_description},
                         os.path.join(pkg_path, 'config', 'ack_controllers.yaml')]),

        # 3. Spawner for Ackermann Controller
        Node(package='controller_manager', executable='spawner',
             arguments=['ackermann_controller'])
    ])
