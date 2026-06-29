import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Fiesta 核心节点
    fiesta_node = Node(
        package='fiesta',
        executable='test_fiesta',
        name='fiesta',
        output='screen',
        parameters=[{
            'resolution': 0.1,
            'update_esdf_every_n_sec': 0.1,
            'reserved_size': 1000000,
            
            # Array implementation limits
            'lx': -20.0,
            'ly': -20.0,
            'lz': -1.6,
            'rx': 20.0,
            'ry': 20.0,
            'rz': 2.0,
            
            # Raycasting parameters
            'min_ray_length': 0.5,
            'max_ray_length': 5.0,
            'ray_cast_num_thread': 0,
            
            # Camera intrinsics (depth camera)
            'center_x': 428.411,
            'center_y': 237.285,
            'focal_x': 428.938,
            'focal_y': 428.938,
            
            # Probabilistic grid map
            'p_hit': 0.70,
            'p_miss': 0.35,
            'p_min': 0.12,
            'p_max': 0.97,
            'p_occ': 0.80,
            
            # Global / Local settings
            'global_map': False,
            'global_update': False,
            'global_vis': False,
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
            ('depth', '/camera/depth/image_rect_raw'),
            ('transform', '/vins_estimator/camera_pose')
        ]
    )

    return LaunchDescription([
        fiesta_node
    ])