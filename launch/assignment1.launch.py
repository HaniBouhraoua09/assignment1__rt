from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='assignment1_rt',
            executable='spawner',
            name='spawner'
        ),
        Node(
            package='assignment1_rt',
            executable='distance_node',
            name='distance_monitor',
            output='screen'
        ),
    ])
