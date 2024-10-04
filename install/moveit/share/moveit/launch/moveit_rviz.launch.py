#from moveit_configs_utils import MoveItConfigsBuilder
#from moveit_configs_utils.launches import generate_moveit_rviz_launch


#def generate_launch_description():
#    moveit_config = MoveItConfigsBuilder("tongquan_sldasm", package_name="moveit").to_moveit_configs()
#    return generate_moveit_rviz_launch(moveit_config)

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    # Khai báo tham số 'use_sim_time'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Load MoveIt config
    moveit_config = MoveItConfigsBuilder("tongquan_sldasm", package_name="moveit").to_moveit_configs()

    # Tạo launch từ MoveIt RViz
    moveit_rviz_launch = generate_moveit_rviz_launch(moveit_config)
    for entity in moveit_rviz_launch.entities:
        if hasattr(entity, 'parameters'):
            entity.parameters.append({'use_sim_time': use_sim_time})
    return moveit_rviz_launch

