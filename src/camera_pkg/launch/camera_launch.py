from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera = Node(
        package='camera_pkg',
        executable='camera',
        output='screen',
    )

    detect_circle_node = Node(
        package='camera_pkg',
        executable='detect_circle',
        output='screen',
    )

    detect_color_node = Node(
        package='camera_pkg',
        executable='detect_color',
        output='screen',
    )

    return LaunchDescription([camera, detect_circle_node, detect_color_node])
