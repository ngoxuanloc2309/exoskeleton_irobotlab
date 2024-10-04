import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, RegisterEventHandler, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit
from launch.actions import TimerAction

def generate_launch_description():
    # Đường dẫn tới các gói cần thiết
    pkg_share = get_package_share_directory('tongquan_sldasm')  # Chỉnh lại đường dẫn hợp lý
    pkg_share1 = get_package_share_directory('moveit')  # Chỉnh lại đường dẫn hợp lý
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'urdf/tongquan_sldasm.urdf'

    urdf_path = os.path.join(pkg_share, urdf_file_name)

    with open(urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    entity_name = 'tongquan_sldasm_' + str(int(time.time()))

    # Thiết lập biến môi trường GAZEBO_MODEL_PATH
    env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_share, 'meshes')
    )

    return LaunchDescription([
        env_var,

        # Argument để sử dụng thời gian mô phỏng
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'model',
            default_value=os.path.join(pkg_share, 'urdf', 'tongquan_sldasm.urdf'),
            description='Absolute path to robot URDF file'
        ),

        # Khởi động Gazebo với ROS integration
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Khởi động node robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(robot_description_content, value_type=str)
            }]
        ),

        # Static transform giữa base_link và base_footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0.00', '0.00', '0.00', '0.00', '0.00', '0.00', 'base_link', 'base_footprint']
        ),

        # Thêm delay để đảm bảo /robot_description được nạp trước khi spawn robot
        TimerAction(
            period=2.0,  # 2 giây delay
            actions=[

                # Spawn URDF trong Gazebo
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='urdf_spawner',
                    output='screen',
                    arguments=['-topic', '/robot_description', '-entity', entity_name]
                )
            ]
        ),

        # Khởi động controller_manager với thông số từ file yaml
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[PathJoinSubstitution([pkg_share1, 'config', 'ros2_controllers.yaml'])],
            output='screen'
        ),

        # Tải danh sách các controllers sau khi robot được spawn
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='urdf_spawner',
                    arguments=['-topic', '/robot_description', '-entity', entity_name]
                ),
                on_exit=[
                    # Spawn joint_state_broadcaster và các controllers
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        arguments=['joint_state_broadcaster'],
                        output='screen'
                    ),
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
                    )
                ]
            )
        ),
    ])
