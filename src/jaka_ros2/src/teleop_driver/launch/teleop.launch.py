import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # ================= 配置区域 =================
    # 1. 你的 MoveIt 配置包名 (请确认名字是否正确)
    moveit_config_pkg = "jaka_zu20_moveit_config"
    
    # 2. 你的 C++ 节点所在的包名
    cpp_pkg_name = "jaka_planner"
    
    # 3. 你的 Python 脚本所在的包名
    py_pkg_name = "teleop_driver" 
    # ===========================================

    # --- A. 加载 MoveIt 配置 (为 C++ 节点提供 IK 解算参数) ---
    moveit_config = MoveItConfigsBuilder("jaka_zu20", package_name=moveit_config_pkg).to_moveit_configs()

    # --- B. 声明参数 ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        # 1. 启动仿真环境 (Gazebo + MoveIt + Rviz)
        # 假设你原本是用 demo_gazebo.launch.py 启动仿真的
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(moveit_config_pkg), 'launch', 'demo_gazebo.launch.py')
            ]),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        # 2. 启动 Joy 驱动节点 (读取硬件 USB 数据)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'deadzone': 0.05,
                'autorepeat_rate': 50.0, # 提高频率以匹配我们的 50Hz 控制
                'use_sim_time': use_sim_time
            }]
        ),

        # 3. 启动 Python 转换节点 (你的 jaka_teleop_final.py)
        Node(
            package=py_pkg_name,
            executable='logitech_control', # 确保 setup.py 里注册了这个名字，或者是文件名
            name='logitech_control',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 4. 启动 C++ 执行节点 (你的 main.cpp 编译出的可执行文件)
        Node(
            package=cpp_pkg_name,
            executable='jaka_test', # CMakeLists.txt 中 add_executable 的名字
            name='jaka_test',
            output='screen',
            # 关键：加载 Kinematics.yaml 才能做防乱飞解算
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {'use_sim_time': use_sim_time}
            ]
        ),
    ])