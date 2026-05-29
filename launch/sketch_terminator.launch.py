#!/usr/bin/env python3

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # =========================================================================
    # 1. Low-Level Control & Hardware Bringup (Self-contained in sketch_terminator)
    # =========================================================================
    package_name = "sketch_terminator"
    package_share = get_package_share_directory(package_name)
    
    # Paths to local resources
    rviz_config = os.path.join(package_share, "launch", "sketch_terminator.rviz")
    robot_description = os.path.join(package_share, "urdf", "prarob_manipulator.urdf.xacro")
    robot_description_config = xacro.process_file(robot_description)
    controller_config = os.path.join(package_share, "config", "controllers.yaml")

    # ros2_control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_config.toxml()}, 
            controller_config
        ],
        output="screen",
    )

    # Spawner: joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawner: velocity_controller
    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Spawner: joint_trajectory_controller
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description_config.toxml()}],
        output="screen",
    )

    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    # =========================================================================
    # 2. High-Level AI, Planning & Integration Nodes (sketch_terminator)
    # =========================================================================
    
    # Path Planner Node
    planner_node = Node(
        package='sketch_terminator',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen'
    )

    # Integration Coordinator Node
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

    # Autonomous ROSA Agent Node
    agent_node = Node(
        package='sketch_terminator',
        executable='agent_node',
        name='agent_node',
        output='screen'
    )

    # Streamlit GUI Panel
    gui_script = os.path.join(package_share, 'gui', 'dashboard.py')
    streamlit_gui = ExecuteProcess(
        cmd=['streamlit', 'run', gui_script, '--server.headless', 'true'],
        output='screen'
    )

    return LaunchDescription([
        control_node,
        joint_state_broadcaster_spawner,
        velocity_controller_spawner,
        joint_trajectory_controller_spawner,
        robot_state_publisher,
        rviz_node,
        planner_node,
        integration_node,
        agent_node,
        streamlit_gui
    ])
