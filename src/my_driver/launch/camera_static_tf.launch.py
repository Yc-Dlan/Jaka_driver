import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_driver' 
    yaml_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'calibrate.yaml'
    )

    with open(yaml_file_path, 'r') as f:
        config = yaml.safe_load(f)['handeye_parameters']

    t = config['translation']
    r = config['rotation']

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='handeye_static_tf_publisher',
        arguments=[
            str(t['x']), str(t['y']), str(t['z']), 
            str(r['yaw']), str(r['pitch']), str(r['roll']), # 顺序为 Y P R
            config['parent_frame'], 
            config['child_frame']
        ]
    )

    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
    )

    return LaunchDescription([
        static_tf_node,
        foxglove_node
    ])