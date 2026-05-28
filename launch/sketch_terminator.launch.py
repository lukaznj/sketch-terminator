#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 1. Path Planner Node
    planner_node = Node(
        package='sketch_terminator',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen'
    )

    # 2. Integration Coordinator Node
    integration_node = Node(
        package='sketch_terminator',
        executable='integration_node',
        name='integration_node',
        output='screen',
        parameters=[{
            'detections_topic': 'detections',
            'vision_positions_topic': '/vision/object_positions',
            'planning_path_topic': '/planning/path',
            'joint_states_topic': '/joint_states',
            'joint_trajectory_topic': '/joint_trajectory_controller/joint_trajectory'
        }]
    )

    # 3. Autonomous ROSA Agent Node
    agent_node = Node(
        package='sketch_terminator',
        executable='agent_node',
        name='agent_node',
        output='screen'
    )

    # 4. Streamlit GUI Panel
    package_share = get_package_share_directory('sketch_terminator')
    gui_script = os.path.join(package_share, 'gui', 'dashboard.py')
    streamlit_gui = ExecuteProcess(
        cmd=['streamlit', 'run', gui_script, '--server.headless', 'true'],
        output='screen'
    )

    return LaunchDescription([
        planner_node,
        integration_node,
        agent_node,
        streamlit_gui
    ])
