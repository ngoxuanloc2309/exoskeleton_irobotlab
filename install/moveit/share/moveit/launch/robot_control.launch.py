import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start controller manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[os.path.join(get_package_share_directory('moveit'), 'config', 'ros2_controllers.yaml')],
            output='screen'
        ),
        # Spawn joint state broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),
        # Spawn controllers
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['chan_trai_controller'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['chan_phai_controller'],
            output='screen'
        ),
    ])
