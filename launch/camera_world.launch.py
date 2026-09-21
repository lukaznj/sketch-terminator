"""Step 2 of calibration: find where the camera sits relative to the robot.

Starts the camera and ``camera_to_world.py``, which spots the checkerboard and
prints the camera-to-robot transform. Copy the rotation and translation it
prints into ``config/vision_config.yaml``.

Run ``camera_calib.launch.py`` first -- this step needs the intrinsics.

    ros2 launch sketch_terminator camera_world.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE_NAME = 'sketch_terminator'


def generate_launch_description():
    share_dir = get_package_share_directory(PACKAGE_NAME)

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera',
            parameters=[os.path.join(share_dir, 'config', 'camera_params.yaml')],
            output='screen',
        ),
        Node(
            package=PACKAGE_NAME,
            executable='camera_to_world.py',
            name='cam2world',
            parameters=[os.path.join(share_dir, 'config', 'calib_config.yaml')],
            output='screen',
        ),
    ])
