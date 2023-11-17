from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="ball_moving_robot", executable="localizer", name="localizer"),
            Node(package="ball_moving_robot", executable="planner", name="planner"),
            Node(package="ball_moving_robot", executable="driver", name="driver"),
        ]
    )
