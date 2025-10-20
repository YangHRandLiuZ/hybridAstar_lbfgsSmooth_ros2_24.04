from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Grid map generator node
        Node(
            package='create_maps',
            executable='simple_demo_node',
            name='grid_map_generator',
            parameters=[
                {'map_frame': 'map'},
                {'resolution': 0.1}
            ],
            output='screen'
        ),
        # Hybrid A* test node
        Node(
            package='create_maps',
            executable='hybrid_astar_test',
            name='hybrid_astar_planner',
            output='screen'
        )
    ])