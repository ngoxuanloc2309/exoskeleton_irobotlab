import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    pkg_share = get_package_share_directory('tongquan_sldasm')
    moveit_config_path = get_package_share_directory('moveit')
    urdf_file_name = 'urdf/tongquan_sldasm.urdf'
    urdf_path = os.path.join(pkg_share, 'urdf', 'tongquan_sldasm.urdf')
    ros2_controllers_yaml = os.path.join(moveit_config_path, 'config', 'ros2_controllers.yaml')
    moveit_controllers_yaml = os.path.join(moveit_config_path, 'config', 'moveit_controllers.yaml')
    srdf_file = os.path.join(moveit_config_path, 'config', 'tongquan_sldasm.srdf')

    # Đọc nội dung file URDF
    with open(urdf_path, 'r') as file:
        urdf = file.read()

    # Tạo tên entity duy nhất
    entity_name = f'tongquan_sldasm_{int(time.time())}'

    # Thiết lập biến môi trường GAZEBO_MODEL_PATH
    gazebo_models_path = os.path.join(pkg_share, 'meshes')
    env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_models_path
    )

    # Khai báo các launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        env_var,

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'model',
            default_value=urdf_path,
            description='Absolute path to robot URDF file'
        ),

        # Khởi động Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Node robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': urdf,
                'use_sim_time': use_sim_time,
                'robot_description_semantic': srdf_file,
            }]
        ),

        # Spawn URDF trong Gazebo sau 2 giây
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='urdf_spawner',
                    output='screen',
                    arguments=['-topic', '/robot_description', '-entity', entity_name],
                    parameters=[{'use_sim_time': use_sim_time}],
                ),
            ]
        ),

        # Spawn các controllers sau 4 giây
        TimerAction(
            period=4.0,
            actions=[
                # Spawn joint_state_broadcaster
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', "--controller-manager", "/controller_manager"],
                    parameters=[ros2_controllers_yaml, {'use_sim_time': use_sim_time}],
                    output='screen'
                ),

                # Spawn chan_trai_controller
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['chan_trai_controller', "--controller-manager", "/controller_manager"],
                    parameters=[ros2_controllers_yaml, {'use_sim_time': use_sim_time}],
                    output='screen'
                ),

                # Spawn chan_phai_controller
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['chan_phai_controller', "--controller-manager", "/controller_manager"],
                    parameters=[ros2_controllers_yaml, {'use_sim_time': use_sim_time}],
                    output='screen'
                ),
            ]
        ),
    ])