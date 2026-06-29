import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    fiesta_share_dir = get_package_share_directory('fiesta')
    rviz_config_file = os.path.join(fiesta_share_dir, 'demo.rviz')

    # Fiesta 核心节点
    fiesta_node = Node(
        package='fiesta',
        executable='test_fiesta',
        name='fiesta',
        output='screen',
        parameters=[{
            'resolution': 0.05,
            'update_esdf_every_n_sec': 0.1,
            'reserved_size': 1000000,
            
            # Array implementation limits
            'lx': -10.0,
            'ly': -10.0,
            'lz': -1.0,
            'rx': 10.0,
            'ry': 10.0,
            'rz': 3.0,
            
            # Raycasting parameters
            'min_ray_length': 0.5,
            'max_ray_length': 5.0,
            'ray_cast_num_thread': 0,
            
            # Probabilistic grid map
            'p_hit': 0.70,
            'p_miss': 0.35,
            'p_min': 0.12,
            'p_max': 0.97,
            'p_occ': 0.80,
            
            # Global / Local settings
            'global_map': True,
            'global_update': True,
            'global_vis': True,
            'radius_x': 3.0,
            'radius_y': 3.0,
            'radius_z': 1.5,
            
            # Depth filter
            'use_depth_filter': True,
            'depth_filter_tolerance': 0.1,
            'depth_filter_max_dist': 10.0,
            'depth_filter_min_dist': 0.1,
            'depth_filter_margin': 0,
            
            # Visualization
            'visualize_every_n_updates': 10,
            'slice_vis_max_dist': 2.0,
            'slice_vis_level': 1.6,
            'vis_lower_bound': 0.0,
            'vis_upper_bound': 10.0,
        }],
        remappings=[
            ('depth', '/camera/depth_registered/points'),
            ('transform', '/kinect/vrpn_client/estimated_transform')
        ]
    )

    # RViz2 可视化节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rvizvisualisation',
        output='log',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        fiesta_node,
        rviz_node
    ])