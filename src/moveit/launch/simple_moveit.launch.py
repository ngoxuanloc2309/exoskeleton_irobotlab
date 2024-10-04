import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction

def generate_launch_description():
    # Đường dẫn tới các file cấu hình controllers và URDF
    moveit_config_path = get_package_share_directory('moveit')
    moveit_controllers_yaml = os.path.join(moveit_config_path, 'config', 'moveit_controllers.yaml')  # Sửa lại biến này
    ros2_controllers_yaml = os.path.join(moveit_config_path, 'config', 'ros2_controllers.yaml')

    urdf_file = os.path.join(get_package_share_directory('tongquan_sldasm'), 'urdf', 'tongquan_sldasm.urdf')
    srdf_file = os.path.join(get_package_share_directory('moveit'),'config','tongquan_sldasm.srdf')

    # Argument để sử dụng thời gian mô phỏng
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Tải URDF từ file và nạp vào robot_state_publisher
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        # Nạp URDF cho robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time,'robot_description_semantic': srdf_file,}]
        ),

        # Định nghĩa plugin MoveIt controller manager để thực thi quỹ đạo
#        Node(
#            package='controller_manager',
#            executable='ros2_control_node',
#            parameters=[moveit_controllers_yaml],  # Sử dụng đúng tên biến
#            output='screen'
#        ),

        # Tải danh sách controllers vào parameter server
#        Node(
#            package='controller_manager',
#            executable='ros2_control_node',
#            parameters=[ros2_controllers_yaml, {'use_sim_time': use_sim_time}],  # Sử dụng đúng tên biến
#            remappings=[
#                ("/controller_manager/robot_description", "/robot_description"),
#            ],
#            output='screen'

#        ),

        

        # Khởi động node move_group để thực hiện các chuyển động MoveIt
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('moveit'), 'launch', 'move_group.launch.py')
            ),

            #launch_arguments={'allow_trajectory_execution': 'true'}.items(),


            launch_arguments={'allow_trajectory_execution': 'true', 'use_sim_time': 'True'}.items(),
        ),

        # Khởi động RViz để hiển thị trạng thái MoveIt
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('moveit'), 'launch', 'moveit_rviz.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )
    ])
