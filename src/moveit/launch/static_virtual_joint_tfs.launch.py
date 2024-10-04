#from moveit_configs_utils import MoveItConfigsBuilder
#from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


#def generate_launch_description():
#    moveit_config = MoveItConfigsBuilder("tongquan_sldasm", package_name="moveit").to_moveit_configs()
#    return generate_static_virtual_joint_tfs_launch(moveit_config)

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    # Khai báo tham số 'use_sim_time'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Load MoveIt config
    moveit_config = MoveItConfigsBuilder("tongquan_sldasm", package_name="moveit").to_moveit_configs()

    # Tạo launch từ MoveIt RViz
    static_virtual_joint_tfs_launch = generate_static_virtual_joint_tfs_launch(moveit_config)
    for entity in static_virtual_joint_tfs_launch.entities:
        if hasattr(entity, 'parameters'):
            entity.parameters.append({'use_sim_time': use_sim_time})
    return static_virtual_joint_tfs_launch
    