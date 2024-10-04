from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        # Start Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('tongquan_sldasm'), 'launch', 'gazebo.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # Delay để đảm bảo Gazebo khởi động trước MoveIt
        TimerAction(
            period=5.0,  # 5 giây delay
            actions=[
                # Start MoveIt (bao gồm cả RViz và controllers)
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('moveit'), 'launch', 'demo.launch.py')),
                    launch_arguments={'use_sim_time': use_sim_time}.items()
                ),
            ]
        )
    ])
