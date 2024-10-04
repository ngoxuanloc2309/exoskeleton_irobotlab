import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Đường dẫn tới các file cần thiết
    urdf_file_name = 'urdf/tongquan_sldasm.urdf'
    srdf_file_name = 'config/tongquan_sldasm.srdf'
    moveit_config_path = get_package_share_directory('moveit')

    urdf_file = os.path.join(moveit_config_path, urdf_file_name)
    srdf_file = os.path.join(moveit_config_path, srdf_file_name)

    # Gazebo specific options
    use_gui = LaunchConfiguration('gazebo_gui', default='true')
    paused = LaunchConfiguration('paused', default='false')

    return LaunchDescription([
        # Argument for pausing simulation
        DeclareLaunchArgument('paused', default_value='false', description='Pause simulation on start'),
        
        # Argument for GUI
        DeclareLaunchArgument('gazebo_gui', default_value='true', description='Run gazebo with GUI'),

        # Launch Gazebo and spawn the robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'paused': paused,
                'gui': use_gui,
            }.items(),
        ),

        # Load the robot description from URDF and SRDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_file}]
        ),

        # Publish joint states (fake for simulation)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            parameters=[{'use_gui': use_gui}]
        ),

        # Start MoveIt move_group
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(os.path.dirname(__file__), 'move_group.launch.py')
        ),
        launch_arguments={
            'allow_trajectory_execution': 'true',
            'fake_execution': 'false'
        }.items(),
        ),


        # Start Rviz for MoveIt
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(os.path.dirname(__file__), 'demo.launch.py')
            ),
            launch_arguments={'config': 'true'}.items(),
        )
    ])
