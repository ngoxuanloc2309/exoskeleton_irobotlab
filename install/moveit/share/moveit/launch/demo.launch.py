from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Tạo cấu hình MoveIt
    moveit_config = MoveItConfigsBuilder("tongquan_sldasm", package_name="moveit").to_moveit_configs()
    
    # Cấu hình use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Tạo launch demo
    demo_launch = generate_demo_launch(moveit_config)
    
    # Đảm bảo use_sim_time được áp dụng cho tất cả các node
    for entity in demo_launch.entities:
        if hasattr(entity, 'parameters'):
            entity.parameters.append({'use_sim_time': use_sim_time})

    return demo_launch
