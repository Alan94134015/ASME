from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera = Node(
        package='camera_pkg',
        executable='camera',
        output='screen',
    )

    detect_ball_node = Node(
        package='camera_pkg',
        executable='detect_ball',
        output='screen',
    )

    return LaunchDescription([camera, detect_ball_node])
