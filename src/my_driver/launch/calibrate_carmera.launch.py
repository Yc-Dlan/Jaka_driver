import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # ArUco 标定板参数
    marker_id = 582            # 替换为你打印的 ArUco 码 ID
    marker_size = 0.1          # 替换为标定板真实尺寸（单位：米）
    
    # TF 坐标系参数
    robot_base_frame = 'base_link'
    robot_effector_frame = 'Link_06'
    tracking_base_frame = 'camera_color_optical_frame'
    tracking_marker_frame = 'aruco_marker_frame'
    calibration_name = 'jaka_cali'

    realsense_pkg = get_package_share_directory('my_driver')
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_pkg, 'launch', 'realsense_bringup.launch.py')
        ),
    )
    
    aruco_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        parameters=[{
            'marker_id': marker_id,
            'marker_size': marker_size,
            'image_is_rectified': True,
            'marker_frame': tracking_marker_frame,
            'camera_frame': tracking_base_frame,
        }],
        remappings=[
            ('/image', '/camera/color/image_raw'),
            ('/camera_info', '/camera/color/camera_info')
        ]
    )

    handeye_pkg = get_package_share_directory('easy_handeye2')
    handeye_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(handeye_pkg, 'launch', 'calibrate.launch.py')
        ),
        launch_arguments={
            'name': calibration_name,
            'calibration_type': 'eye_in_hand',
            'robot_base_frame': robot_base_frame,
            'robot_effector_frame': robot_effector_frame,
            'tracking_base_frame': tracking_base_frame,
            'tracking_marker_frame': tracking_marker_frame,
            'freehand_robot_movement': 'true',
        }.items()
    )

    return LaunchDescription([
        realsense_launch,
        aruco_node,
        handeye_launch
    ])