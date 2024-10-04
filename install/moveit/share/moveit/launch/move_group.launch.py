from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    moveit_config = MoveItConfigsBuilder("tongquan_sldasm", package_name="moveit").to_moveit_configs()
    move_group_launch = generate_move_group_launch(moveit_config)
    for entity in move_group_launch.entities:
        if hasattr(entity, 'parameters'):
            entity.parameters.append({'use_sim_time': use_sim_time})
    

    return move_group_launch
