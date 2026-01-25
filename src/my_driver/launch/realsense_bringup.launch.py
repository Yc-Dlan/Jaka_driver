import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 路径获取
    my_pkg_path = get_package_share_directory('my_driver')
    realsense_pkg_path = get_package_share_directory('realsense2_camera')
    
    # 2. 参数定义
    log_level = LaunchConfiguration('log_level', default='INFO')
    # 指向我们刚刚创建的高清配置文件
    config_path = os.path.join(my_pkg_path, 'config', 'realsense.yaml')

    # 3. RealSense 启动节点
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_pkg_path, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'config_file': config_path,
            'log_level': log_level,
            'camera_namespace': '/'  # 带电作业通常不使用复杂命名空间，保持根目录方便 TF 对齐
        }.items()
    )

    # 4. 你的 Python 融合/处理节点 (假设包名为 my_fusion_pkg)
    fusion_node = Node(
        package='my_fusion_pkg',
        executable='fusion_node',
        name='high_res_fusion_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        # 如果需要将 D435i 坐标系转换到机械臂，可以在这里传参
    )

    # 5. 延迟启动处理节点，确保相机已经初始化
    delayed_fusion_node = TimerAction(
        period=3.0,
        actions=[fusion_node]
    )

    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='INFO'),
        realsense_node,
        #delayed_fusion_node
    ])