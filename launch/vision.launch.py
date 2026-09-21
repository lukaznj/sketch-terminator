"""Vision and planning only -- no motors, no GUI.

Useful for checking the camera calibration and watching the planner work
without powering up the arm:

    ros2 launch sketch_terminator vision.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE_NAME = 'sketch_terminator'


def generate_launch_description():
    vision_config = os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'config', 'vision_config.yaml'
    )

    return LaunchDescription([
        Node(
            package=PACKAGE_NAME,
            executable='yolo_workspace_processing_node.py',
            name='yolo_workspace_processing_node',
            parameters=[vision_config],
            output='screen',
        ),
        Node(
            package=PACKAGE_NAME,
            executable='path_planner_node.py',
            name='path_planner_node',
            output='screen',
        ),
    ])
