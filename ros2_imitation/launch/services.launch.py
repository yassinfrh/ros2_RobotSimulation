from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Object detection node
    object_detection = Node(
            package='ros2_imitation',
            executable='object_detection',
            output='screen'
        )
    # Spawn objects service node
    spawn_objects_service = Node(
            package='ros2_imitation',
            executable='spawn_objects_service',
            output='screen'
        )
    # Pick and place action server node
    pick_place_action_server = Node(
            package='ros2_imitation',
            executable='pick_place_action_server',
            output='screen'
        )

    return LaunchDescription([
        object_detection,
        spawn_objects_service,
        pick_place_action_server
    ])