from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="diagnostics_analyzer",
            executable="my_publisher",
            name="my_publisher",
            output='screen',
            emulate_tty=True,
            parameters=[os.path.join(os.getcwd(), 'my_publisher_params.yaml')]
        )
    ])