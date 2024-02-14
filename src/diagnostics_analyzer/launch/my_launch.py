from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('diagnostics_analyzer'),
        'config',
        'my_publisher_params.yaml'
        )
        
    node=Node(
        package = 'diagnostics_analyzer',
        name = 'my_publisher',
        executable = 'my_publisher',
        parameters = [config]
    )

    ld.add_action(node)
    return ld

