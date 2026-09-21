"""Step 1 of calibration: find the camera's intrinsic parameters.

Starts the camera and the standard ``camera_calibration`` GUI. Wave the
checkerboard around until the sample bars fill up, hit CALIBRATE, then SAVE,
and copy the resulting values into ``config/camera_calibration_params.yaml``.

The board geometry is read from ``config/calib_config.yaml`` so the two
calibration steps cannot drift apart.

    ros2 launch sketch_terminator camera_calib.launch.py
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE_NAME = 'sketch_terminator'


def generate_launch_description():
    share_dir = get_package_share_directory(PACKAGE_NAME)
    camera_config = os.path.join(share_dir, 'config', 'camera_params.yaml')
    calib_config = os.path.join(share_dir, 'config', 'calib_config.yaml')

    with open(calib_config, 'r') as handle:
        params = yaml.safe_load(handle)['cam2world']['ros__parameters']

    # camera_calibration counts *inner corners* and wants them as HxW.
    board_size = f"{params['checkerboard_height']}x{params['checkerboard_width']}"

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera',
            parameters=[camera_config],
            output='screen',
        ),
        Node(
            package='camera_calibration',
            executable='cameracalibrator',
            name='camera_calib',
            arguments=['--size', board_size, '--square', str(params['square_size'])],
            remappings=[('image', '/image_raw')],
            output='screen',
        ),
    ])
