#!/usr/bin/env python3

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # =========================================================================
    # 1. Low-Level Control & Hardware Bringup (from ros2_prarob)
    # =========================================================================
    robot_name = "prarob_manipulator"
    prarob_package = "ros2_prarob"
    
    try:
        prarob_share = get_package_share_directory(prarob_package)
        rviz_config = os.path.join(prarob_share, "launch", "prarob_manipulator.rviz")
        robot_description = os.path.join(prarob_share, "urdf", robot_name + ".urdf.xacro")
        robot_description_config = xacro.process_file(robot_description)
        controller_config = os.path.join(prarob_share, "controllers", "controllers.yaml")
        has_prarob = True
    except Exception:
        # Fallback if ros2_prarob is not installed (e.g. clean simulation environment)
        has_prarob = False

    ld = LaunchDescription()

    if has_prarob:
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
        ld.add_action(control_node)

        # Spawner: joint_state_broadcaster
        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        )
        ld.add_action(joint_state_broadcaster_spawner)

        # Spawner: velocity_controller
        velocity_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["velocity_controller", "-c", "/controller_manager"],
            output="screen",
        )
        ld.add_action(velocity_controller_spawner)

        # Spawner: joint_trajectory_controller
        joint_trajectory_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
            output="screen",
        )
        ld.add_action(joint_trajectory_controller_spawner)

        # robot_state_publisher
        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description_config.toxml()}],
            output="screen",
        )
        ld.add_action(robot_state_publisher)

        # RViz2
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        )
        ld.add_action(rviz_node)

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
    ld.add_action(planner_node)

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
    ld.add_action(integration_node)

    # Autonomous ROSA Agent Node
    agent_node = Node(
        package='sketch_terminator',
        executable='agent_node',
        name='agent_node',
        output='screen'
    )
    ld.add_action(agent_node)

    # Streamlit GUI Panel
    package_share = get_package_share_directory('sketch_terminator')
    gui_script = os.path.join(package_share, 'gui', 'dashboard.py')
    streamlit_gui = ExecuteProcess(
        cmd=['streamlit', 'run', gui_script, '--server.headless', 'true'],
        output='screen'
    )
    ld.add_action(streamlit_gui)

    return ld
