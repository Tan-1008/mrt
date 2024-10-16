from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sauron',
            executable='publisher1',
        ),
        Node(
            package = 'sauron',
            executable = 'publisher2',
        ),
        Node(
            package='sauron',
            executable='subscriber',
        ),
    ])