import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Sử dụng thời gian giả lập trong Gazebo
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Đường dẫn đến file URDF của robot
    urdf_file_name = 'urdf/tongquan_sldasm.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('tongquan_sldasm'),
        urdf_file_name
    )

    # Đọc nội dung file URDF
    with open(urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    # Tạo tên thực thể cho robot trong Gazebo
    entity_name = 'tongquan_sldasm_' + str(int(time.time()))

    # Thiết lập biến môi trường GAZEBO_MODEL_PATH để tìm kiếm các file mesh
    env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(get_package_share_directory('tongquan_sldasm'), 'meshes')
    )

    return LaunchDescription([
        env_var,

        # Khai báo tham số use_sim_time để sử dụng thời gian của simulation (nếu có)
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Khai báo tham số model chỉ ra đường dẫn đến file URDF
        DeclareLaunchArgument(
            'model',
            default_value=os.path.join(get_package_share_directory('tongquan_sldasm'), 'urdf', 'tongquan_sldasm.urdf'),
            description='Absolute path to robot URDF file'
        ),

        # Khởi động Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Khởi động node robot_state_publisher để phát đi thông tin về robot state
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

        # Khởi động node spawn_entity để thêm URDF vào Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=['-topic', '/robot_description', '-entity', entity_name]
        )
    ])
