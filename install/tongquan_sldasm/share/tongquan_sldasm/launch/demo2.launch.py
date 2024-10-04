import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = '/home/kienpvt/exos_new_ws/src/tongquan_sldasm'
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'urdf/tongquan_sldasm.urdf'

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf_path = os.path.join(
        get_package_share_directory('tongquan_sldasm'),
        urdf_file_name)

    with open(urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    entity_name = 'tongquan_sldasm_' + str(int(time.time()))

    # Thiết lập biến môi trường GAZEBO_MODEL_PATH
    env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(get_package_share_directory('tongquan_sldasm'), 'meshes')
    )

    return LaunchDescription([
        env_var,
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'model',
            default_value=os.path.join(get_package_share_directory('tongquan_sldasm'), 'urdf', 'tongquan_sldasm.urdf'),
            description='Absolute path to robot URDF file'),

        # Khởi động Gazebo
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
                'robot_description': ParameterValue(
                    robot_description_content,
                    value_type=str
                )
            }]
        ),
        # Add
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0.00', '0.00', '0.00', '0.00', '0.00', '0.00', 'base_link', 'base_footprint']
        ),
        

        # Khởi động node spawn_entity.py để thêm URDF vào Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=['-topic', '/robot_description', '-entity', entity_name]
        ),

        #add
        ExecuteProcess(
            cmd=['ros2', 'param', 'load', '/controller_manager', PathJoinSubstitution([pkg_share, 'config', 'joint_trajectory_controller.yaml'])],
            output='screen'
        ),

        #add
        Node(
            package='controller_manager', executable='spawner',
            arguments=['joint_state_controller', 'robot_arm_controller', 'hand_ee_controller'],
            output='screen'
        )

 #       Node(
 #           package='controller_manager',
 #           executable='spawner',
 #           name='controller_spawner',
 #           output='screen',
 #           parameters=['/home/kienpvt/exos_new_ws/src/tongquan_sldasm/config/joint_trajectory_controller.yaml'],
 #           arguments=['joint_state_controller', 'chan_trai_controller', 'chan_phai_controller']
 #       )

    ])