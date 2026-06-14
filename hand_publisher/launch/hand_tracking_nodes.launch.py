from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="hand_publisher",
                executable="hand_points_node",
                name="hand_points_node",
                output="screen",
            ),
            Node(
                package="hand_publisher",
                executable="hand_publisher_node",
                name="hand_publisher_node",
                output="screen",
            ),
            Node(
                package="hand_publisher",
                executable="hand_frame_node",
                name="hand_frame_node",
                output="screen",
            ),
        ]
    )
