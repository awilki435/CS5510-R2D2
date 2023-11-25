import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory("ball_moving_robot"),
        "rviz",
        "ball_moving_robot.rviz",
    )
    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
            ),
            Node(
                package="ball_moving_robot", executable="transforms", name="transforms"
            ),
            # Node(package="ball_moving_robot", executable="localizer", name="localizer"),
            Node(package="ball_moving_robot", executable="planner", name="planner"),
            Node(package="ball_moving_robot", executable="driver", name="driver"),
        ]
    )
