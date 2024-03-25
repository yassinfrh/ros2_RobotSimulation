from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():

    # Include services.launch.py
    services_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros2_imitation'),
                'services.launch.py'))
    )

    # Environment node
    environment = Node(
            package='ros2_imitation',
            executable='environment',
            name='environment',
            output='screen'
        )

    return LaunchDescription([
        services_launch,
        environment
    ])