import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ip_arg = DeclareLaunchArgument('ip', default_value='192.168.x.x', description='JAKA Robot IP')
    ip_val = LaunchConfiguration('ip')

    jaka_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('jaka_driver'), 'launch', 'robot_start.launch.py')
        ]),
        launch_arguments={'ip': ip_val}.items(),
    )

    delayed_teleop_nodes = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                output='screen',
                parameters=[{
                    'deadzone': 0.05,
                    'autorepeat_rate': 50.0
                }]
            ),
            Node(
                package='teleop_driver',
                executable='hybrid_control', 
                name='hybrid_control',
                output='screen'
            ),
            Node(
                package='jaka_driver',
                executable='jaka_hybrid_R', 
                name='jaka_hybrid_R',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        ip_arg,
        jaka_driver_launch,
        delayed_teleop_nodes
    ])