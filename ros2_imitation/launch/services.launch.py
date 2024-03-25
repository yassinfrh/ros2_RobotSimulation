from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Object detection node
    object_detection = Node(
            package='ros2_imitation',
            executable='object_detection',
            name='object_detection',
            output='screen'
        )
    # Spawn boxes service node
    spawn_boxes_service = Node(
            package='ros2_imitation',
            executable='spawn_boxes_service',
            name='spawn_boxes_service',
            output='screen'
        )
    # Pick and place service node
    pick_place_service = Node(
            package='ros2_imitation',
            executable='pick_place_service',
            name='pick_place_service',
            output='screen'
        )

    return LaunchDescription([
        object_detection,
        spawn_boxes_service,
        pick_place_service
    ])