import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    moveit_config_pkg = "jaka_zu20_moveit_config"

    cpp_pkg_name = "jaka_planner"
    
    py_pkg_name = "teleop_driver" 

    moveit_config = MoveItConfigsBuilder("jaka_zu20", package_name=moveit_config_pkg).to_moveit_configs()

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(moveit_config_pkg), 'launch', 'demo_gazebo.launch.py')
            ]),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'deadzone': 0.05,
                'autorepeat_rate': 50.0,
                'use_sim_time': use_sim_time
            }]
        ),

        Node(
            package=py_pkg_name,
            executable='logitech_control', 
            name='logitech_control',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package=cpp_pkg_name,
            executable='jaka_test', 
            name='jaka_test',
            output='screen',
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {'use_sim_time': use_sim_time}
            ]
        ),
    ])