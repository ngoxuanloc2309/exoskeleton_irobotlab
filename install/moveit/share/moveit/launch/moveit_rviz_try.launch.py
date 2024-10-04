from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("tongquan_sldasm", package_name="moveit").to_moveit_configs()
    
    return LaunchDescription([
        # Khởi động RViz với MoveIt
        generate_demo_launch(moveit_config),
        
        # Khởi động robot_state_publisher để phát các trạng thái khớp
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[moveit_config.robot_description]
        ),
        
        # Khởi động joint_state_broadcaster để phát các trạng thái khớp
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),
    ])
