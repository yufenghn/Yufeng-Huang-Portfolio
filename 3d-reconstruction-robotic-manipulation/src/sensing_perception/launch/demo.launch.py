from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensing_perception',
            executable='main',
            name='sensing_perception_node',
            output='screen'
        ),
    ])
