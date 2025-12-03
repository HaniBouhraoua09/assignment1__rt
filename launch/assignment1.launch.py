from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Start the Simulator (Turtlesim)
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        # 2. Run the Spawner (Sets up the two turtles)
        Node(
            package='assignment1_rt',
            executable='spawner',
            name='spawner'
        ),
	# 3. Start the Distance Monitor
	Node(
		package='assignment1_rt',
		executable='distance_node',
		name='distance_monitor',
		output='screen',
		parameters=[{
		    'dist_threshold': 2.0,  # We can change this easily now!
		    'wall_min': 1.5,
		    'wall_max': 9.5
		}]
	),
    ])
