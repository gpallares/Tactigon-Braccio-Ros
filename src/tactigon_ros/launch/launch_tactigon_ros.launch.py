from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tactigon_ros',
            executable='tactigon_turtlesim_controller',
            name='tactigon_turtlesim_controller',
            output='screen'
        ),
        Node(
            package='tactigon_ros',
            executable='tactigon_data',
            name='tactigon_data',
            output='screen'
        ),
        Node(
            package='tactigon_ros',
            executable='tactigon_logger',
            name='tactigon_logger',
            output='screen'
        ),
    ])