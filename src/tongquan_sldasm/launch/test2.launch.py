import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml

# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None

# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # Đường dẫn package
    pkg_share = get_package_share_directory('tongquan_sldasm')
    moveit_config_path = get_package_share_directory('moveit_true')

    # Các file cấu hình
    urdf_path = os.path.join(pkg_share, 'urdf', 'tongquan_sldasm.urdf')
    srdf_path = os.path.join(moveit_config_path, 'config', 'tongquan_sldasm.srdf')
    ros2_controllers_yaml = os.path.join(moveit_config_path, 'config', 'ros2_controllers.yaml')
    moveit_controllers_yaml = os.path.join(moveit_config_path, 'config', 'moveit_controllers.yaml')
    kinematics_yaml = load_yaml("moveit_true", "config/kinematics.yaml")

    # Đọc URDF và SRDF
    robot_description = load_file("tongquan_sldasm", "urdf/tongquan_sldasm.urdf")
    robot_description_semantic = load_file("moveit_true", "config/tongquan_sldasm.srdf")

    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Cấu hình MoveIt
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }

    # Cấu hình controllers
    moveit_controllers = load_yaml("moveit_true", "config/moveit_controllers.yaml")
    
    # Các tham số khác
    use_sim_time = {'use_sim_time': True}
    robot_description_param = {'robot_description': robot_description}
    robot_description_semantic_param = {'robot_description_semantic': robot_description_semantic}

    # Cấu hình Gazebo parameters
    gazebo_params = {
        'max_step_size': 0.001,
        'real_time_factor': 1.0,
        'real_time_update_rate': 1000.0,
        'max_update_rate': 1000.0,
        'physics': {
            'type': 'ode',
            'max_contacts': 20,
            'ode': {
                'solver': {
                    'type': 'quick',
                    'min_step_size': 0.0001,
                    'iters': 50,
                    'sor': 1.3
                },
                'constraints': {
                    'cfm': 0.0,
                    'erp': 0.2,
                    'contact_max_correcting_vel': 100.0,
                    'contact_surface_layer': 0.001
                }
            }
        }
    }

    # Node robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param, use_sim_time],
    )

    # Node joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[ros2_controllers_yaml, use_sim_time],
        output='screen',
    )

    # Nodes cho các controller
    chan_trai_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['chan_trai_group_controller', '--controller-manager', '/controller_manager'],
        parameters=[ros2_controllers_yaml, use_sim_time],
        output='screen'
    )

    chan_phai_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['chan_phai_group_controller', '--controller-manager', '/controller_manager'],
        parameters=[ros2_controllers_yaml, use_sim_time],
        output='screen'
    )

    # Node nhận dữ liệu
    receive_data_node = Node(
        package="control_exos",
        executable='thaythe.py',
        name='exos',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # Node MoveIt
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description_param,
            robot_description_semantic_param,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            moveit_controllers,
            use_sim_time,
        ],
    )

    # Node Rviz
    rviz_config_file = os.path.join(get_package_share_directory("moveit_true"), "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description_param,
            robot_description_semantic_param,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            use_sim_time,
        ],
    )

    # Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        additional_env={'GAZEBO_MODEL_PATH': os.path.join(pkg_share, 'meshes')},
        arguments=['-physics', 'ode', '--params-file', '/tmp/gazebo_physics.yaml']
    )

    # Write gazebo params to yaml file
    with open('/tmp/gazebo_physics.yaml', 'w') as f:
        yaml.dump(gazebo_params, f)

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'tongquan_sldasm'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=os.path.join(pkg_share, 'meshes')
        ),

        gazebo,
        spawn_entity,
        robot_state_publisher_node,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[chan_trai_controller_spawner, chan_phai_controller_spawner]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=chan_phai_controller_spawner,
                on_exit=[run_move_group_node, rviz_node]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=rviz_node,
                on_exit=[receive_data_node],
            )
        ),
    ])