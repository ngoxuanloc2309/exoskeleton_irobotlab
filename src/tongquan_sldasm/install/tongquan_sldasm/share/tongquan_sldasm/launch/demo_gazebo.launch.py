import os #Thu vien os vdu: os.path.join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription #include 1 file launch khac vao file launch hien tai
from launch.substitutions import LaunchConfiguration, Command #thu vien nay dung de trich xuat gtri cua 1 tham so vdu arg x..
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    #Khai bao bien va nhap toa do
    return LaunchDescription([
        DeclareLaunchArgument('arg_x', default_value='0.00'),
        DeclareLaunchArgument('arg_y', default_value='0.00'),
        DeclareLaunchArgument('arg_z', default_value='0.00'),
        DeclareLaunchArgument('arg_R', default_value='0.00'),
        DeclareLaunchArgument('arg_P', default_value='0.00'),
        DeclareLaunchArgument('arg_Y', default_value='0.00'),

        Node(
            package = 'robot_state_publisher',
            executable ='robot_state_publisher', #tep thuc thi
            name = 'robot_state_publisher', #Dat ten cho node
            output = 'screen',
            parameters=[{
                'robot_description': ParameterValue(
                    open('/home/kienpvt/exos_new_ws/src/tongquan_sldasm/urdf/tongquan_sldasm.urdf').read(), 
                    value_type=str
                )
            }]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    FindPackageShare('gazebo_ros').find('gazebo_ros'),'launch', 'gazebo.launch.py'
                )
            ])
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0.00', '0.00', '0.00', '0.00', '0.00', '0.00', 'base_link', 'base_footprint']
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name="spawn_urdf",
            arguments=[
                    '-file', '/home/kienpvt/exos_new_ws/src/tongquan_sldasm/urdf/tongquan_sldasm.urdf',
                    '-entity', 'exos_urdf',
                    '-x', LaunchConfiguration('arg_x'),
                    '-y', LaunchConfiguration('arg_y'),
                    '-z', LaunchConfiguration('arg_z'),
                    '-Y', LaunchConfiguration('arg_Y'),
                    '-J', 'joint_1', '0.0',
                    '-J', 'joint_2', '0.0',
                    '-J', 'joint_3', '0.0',
                    '-J', 'joint_4', '0.0'

            ]
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            output='screen',
            parameters=['/home/kienpvt/exos_new_ws/src/tongquan_sldasm/config/joint_trajectory_controller.yaml'],
            arguments=['joint_state_controller', 'chan_trai_controller', 'chan_phai_controller']
        ),
    ])