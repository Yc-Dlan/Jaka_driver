import os
from ament_index_python.packages import get_package_prefix
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_gazebo_launch 

def generate_launch_description():
    jaka_share = os.path.join(get_package_prefix('jaka_description'), 'share')
    robotiq_share = os.path.join(get_package_prefix('robotiq_description'), 'share')
    models_path = f"{jaka_share}:{robotiq_share}"

    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] += f":{models_path}"
    else:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] = models_path

    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += f":{models_path}"
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = models_path
    
    moveit_config = MoveItConfigsBuilder("jaka_zu20", package_name="jaka_zu20_moveit_config").to_moveit_configs()
    return generate_demo_gazebo_launch(moveit_config)