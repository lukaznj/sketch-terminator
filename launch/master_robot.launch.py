"""Brings up the whole robot: hardware, vision, planning, motion and both UIs.

This is the entry point for actually running the thing:

    ros2 launch sketch_terminator master_robot.launch.py

Start order does not matter -- every node either waits for its inputs or is
idle until something arrives -- so everything comes up in parallel.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro

PACKAGE_NAME = 'sketch_terminator'
ROBOT_NAME = 'sketch_terminator'


def generate_launch_description():
    share_dir = get_package_share_directory(PACKAGE_NAME)

    rviz_config = os.path.join(share_dir, 'rviz', f'{ROBOT_NAME}.rviz')
    controller_config = os.path.join(share_dir, 'controllers', 'controllers.yaml')
    vision_config = os.path.join(share_dir, 'config', 'vision_config.yaml')
    camera_config = os.path.join(share_dir, 'config', 'camera_params.yaml')
    dashboard = os.path.join(share_dir, 'gui', 'dashboard.py')

    # The URDF carries the ros2_control hardware definition, so both the
    # controller manager and robot_state_publisher need the expanded xacro.
    robot_description = xacro.process_file(
        os.path.join(share_dir, 'urdf', f'{ROBOT_NAME}.urdf.xacro')
    ).toxml()

    return LaunchDescription([
        # --- Hardware interface and controllers --------------------------
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description}, controller_config],
            output='screen',
        ),
        *[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[controller, '-c', '/controller_manager'],
                output='screen',
            )
            for controller in (
                'joint_state_broadcaster',
                'velocity_controller',
                'joint_trajectory_controller',
            )
        ],

        # --- Robot model and visualisation -------------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),

        # --- Vision ------------------------------------------------------
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera',
            parameters=[camera_config],
            output='screen',
        ),
        Node(
            package=PACKAGE_NAME,
            executable='yolo_node.py',
            name='yolo_node',
            parameters=[{
                'model': 'yolo12m.pt',
                'device': 'cuda:0',
                'threshold': 0.5,
                'iou': 0.5,
                'input_image_topic': '/image_raw',
                'detections_topic': '/yolo/detections',
                'debug_image_topic': 'dbg_image',
                'use_debug': True,
            }],
            output='screen',
        ),
        Node(
            package=PACKAGE_NAME,
            executable='yolo_workspace_processing_node.py',
            name='yolo_workspace_processing_node',
            parameters=[vision_config],
            output='screen',
        ),

        # --- Planning and motion -----------------------------------------
        # path_planner_node -> generate_smooth_path -> move_robot_node,
        # with kinematics_node closing the loop back from /joint_states.
        Node(
            package=PACKAGE_NAME,
            executable='path_planner_node.py',
            name='path_planner_node',
            output='screen',
        ),
        Node(
            package=PACKAGE_NAME,
            executable='generate_smooth_path.py',
            name='smooth_path_generator',
            output='screen',
        ),
        Node(
            package=PACKAGE_NAME,
            executable='move_robot.py',
            name='move_robot_node',
            output='screen',
        ),
        Node(
            package=PACKAGE_NAME,
            executable='kinematics_node.py',
            name='kinematics_node',
            output='screen',
        ),

        # --- User interfaces ---------------------------------------------
        Node(
            package=PACKAGE_NAME,
            executable='agent_node.py',
            name='agent_node',
            output='screen',
        ),
        # Headless, so it does not steal focus; open http://localhost:8501.
        ExecuteProcess(
            cmd=['streamlit', 'run', dashboard, '--server.headless', 'true'],
            output='screen',
        ),
    ])
