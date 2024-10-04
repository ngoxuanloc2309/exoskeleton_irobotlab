import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction

def generate_launch_description():
    pkg_share = get_package_share_directory('tongquan_sldasm')
    moveit_config_path = get_package_share_directory('moveit')
    use_sim_time = {'use_sim_time': True}
    urdf_file_name = 'urdf/tongquan_sldasm.urdf'
    ros2_controllers_yaml = os.path.join(moveit_config_path, 'config', 'ros2_controllers.yaml')
    urdf_path = os.path.join(pkg_share, 'urdf', 'tongquan_sldasm.urdf')
    ros2_controllers_yaml1=os.path.join(get_package_share_directory('tongquan_sldasm'),'config','ros2_controllers.yaml')
    ros2_controllers_yaml2=os.path.join(get_package_share_directory('moveit'),'config','ros2_controllers.yaml')
    moveit_controllers_yaml = os.path.join(get_package_share_directory('moveit'),'config','moveit_controllers.yaml')
    srdf_file = os.path.join(get_package_share_directory('moveit'),'config','tongquan_sldasm.srdf')
    # Đọc file URDF để lấy nội dung robot_description
#    with open(urdf_path, 'r') as urdf_file:
#        robot_description_content = urdf_file.read()
    urdf = open(urdf_path).read()
#    robot_description = {'robot_description': robot_description_content}  # Khởi tạo robot_description

    entity_name = 'tongquan_sldasm_' + str(int(time.time()))
#     entity_name = 'tongquan_sldasm'

     #Set GAZEBO_MODEL_PATH environment variable
    env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_share, 'meshes')
    )

    return LaunchDescription([
        env_var,

        DeclareLaunchArgument(
           # 'use_sim_time',
            #default_value='true',
            use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'model',
            default_value=os.path.join(pkg_share, 'urdf', 'tongquan_sldasm.urdf'),
            description='Absolute path to robot URDF file',
            use_sim_time,
        ),

        # Khởi động Gazebo với ROS integration
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Khởi động node robot_state_publisher và truyền robot_description

         Node(
             package='robot_state_publisher',
             executable='robot_state_publisher',
             name='robot_state_publisher',
             output='screen',
             parameters=[{                
                 'robot_description': urdf,
                 'use_sim_time': True,
                 'robot_description_semantic': srdf_file, 
             },
             ]
         ),

        # Add delay để đảm bảo /robot_description được publish trước khi spawn robot trong Gazebo
        TimerAction(
            period=2.0,  # 2 giây delay
            actions=[

                # Spawn URDF trong Gazebo, sử dụng /robot_description topic
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='urdf_spawner',
                    output='screen',
                    arguments=['-topic', '/robot_description', '-entity', entity_name],
                    parameters=[{'use_sim_time': True}],
                ),
            ]
        ),

        # Start ros2_control_node with ros2_controllers.yaml and truyền robot_description
        # Node(
        #     # name='ros2_control_node',
        #     package='controller_manager',
        #     executable='ros2_control_node',
        #     # arguments=['chan_trai_controller', 'chan_phai_controller'],
        #     # parameters=[ros2_controllers_yaml2],  # Truyền robot_description và file yaml
        #     parameters=[ros2_controllers_yaml2],
        #     output='screen',
        #     remappings=[
        #         ("/controller_manager/robot_description", "/robot_description"),
        #     ],
        # ),

       


        # Delay để đảm bảo ros2_control_node khởi động trước khi spawn controllers
       TimerAction(
           period=4.0,  # 4 giây delay để đảm bảo ros2_control_node khởi chạy trước
           actions=[

                # Spawn joint_state_broadcaster
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', "--controller-manager", "/controller_manager"],
                    parameters=[ros2_controllers_yaml2, {'use_sim_time': use_sim_time}],
                    output='screen'
                ),

                # Spawn chan_trai_controller
                Node(
                    package='controller_manager',
                    executable='spawner',
                   arguments=['chan_trai_controller', "--controller-manager", "/controller_manager"],
                    parameters=[ros2_controllers_yaml2, {'use_sim_time': use_sim_time}],
                    #parameters=[moveit_controllers_yaml],
                    output='screen'
                ),

                # Spawn chan_phai_controller
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['chan_phai_controller', "--controller-manager", "/controller_manager"],
                    parameters=[ros2_controllers_yaml2, {'use_sim_time': use_sim_time}],
                    #parameters=[moveit_controllers_yaml],
                    output='screen'
                ),
           ]
       ),
    ])
