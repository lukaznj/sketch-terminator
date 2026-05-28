#!/usr/bin/env python3

import os
import json
import time
import rclpy
from langchain.agents import tool
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .kinematics import Kinematics

# Joint controller parameters
JOINT_NAMES = ['joint1', 'joint2', 'joint3']
JOINT_MIN = [-3.14, -3.14, -3.14]
JOINT_MAX = [3.14, 3.14, 3.14]

@tool
def get_detected_objects(timeout_sec: float = 3.0) -> str:
    """
    Retrieves the most recent list of 3D detected objects in the environment.
    Use this to see what objects are currently visible to the camera (e.g. car, cat, traffic light)
    and where they are located.
    """
    if not rclpy.ok():
        rclpy.init()

    node = rclpy.create_node(f'rosa_object_fetcher_{int(time.time())}')
    received_msg = None

    def callback(msg):
        nonlocal received_msg
        received_msg = msg

    subscription = node.create_subscription(
        String,
        '/vision/object_positions',
        callback,
        10
    )

    try:
        start_time = node.get_clock().now()
        while received_msg is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = node.get_clock().now() - start_time
            if elapsed.nanoseconds > (timeout_sec * 1e9):
                return "No objects detected on topic /vision/object_positions. Is the vision system running?"
            
        data = json.loads(received_msg.data)
        objects = data.get("objects", [])
        if not objects:
            return "No objects currently detected in camera view."
        
        output = "Currently detected objects in environment (robot_base coordinates in mm):\n"
        for obj in objects:
            output += f"- Class: {obj.get('class')}, Position: [X: {obj.get('x')}, Y: {obj.get('y')}], BBox: {obj.get('bbox')}\n"
        return output

    except Exception as e:
        return f"Error retrieving detected objects: {str(e)}"
    finally:
        node.destroy_node()

@tool
def move_robot_joints(positions: list[float], duration: float = 3.0) -> str:
    """
    Moves the robot joints [joint1, joint2, joint3] directly to specified positions (in radians).
    joint1 corresponds to shoulder (beta), joint2 to elbow (gama), and joint3 to base (alpha).
    """
    if not rclpy.ok():
        rclpy.init()

    node_name = f'rosa_mover_{int(time.time())}'
    node = rclpy.create_node(node_name)
    
    try:
        if len(positions) != 3:
            return "Error: You must provide exactly 3 joint positions [joint1, joint2, joint3]."

        publisher = node.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        # Prepare Message
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        
        # Clip positions for safety
        point.positions = [
            max(JOINT_MIN[0], min(JOINT_MAX[0], float(positions[0]))),
            max(JOINT_MIN[1], min(JOINT_MAX[1], float(positions[1]))),
            max(JOINT_MIN[2], min(JOINT_MAX[2], float(positions[2])))
        ]
        
        seconds = int(duration)
        nanoseconds = int((duration - seconds) * 1e9)
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = nanoseconds
        msg.points.append(point)

        # Grace period for publisher discovery
        time.sleep(0.2) 
        publisher.publish(msg)
        time.sleep(0.1) 
        
        return f"Successfully commanded joint angles: {point.positions}"

    except Exception as e:
        return f"Error moving joints: {str(e)}"
    finally:
        node.destroy_node()

@tool
def get_joint_states(timeout_sec: float = 2.0) -> str:
    """
    Retrieves current joint names, positions, velocities, and efforts of the robot manipulator.
    """
    if not rclpy.ok():
        rclpy.init()

    node = rclpy.create_node(f'rosa_joint_fetcher_{int(time.time())}')
    received_msg = None

    def callback(msg):
        nonlocal received_msg
        received_msg = msg

    subscription = node.create_subscription(
        JointState,
        '/joint_states',
        callback,
        10
    )

    try:
        start_time = node.get_clock().now()
        while received_msg is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = node.get_clock().now() - start_time
            if elapsed.nanoseconds > (timeout_sec * 1e9):
                return "Error: Timeout waiting for joint states. Is the robot simulation or hardware running?"
        
        output = "Current Robot Joint States:\n"
        for i, name in enumerate(received_msg.name):
            pos = received_msg.position[i] if i < len(received_msg.position) else 0.0
            vel = received_msg.velocity[i] if i < len(received_msg.velocity) else 0.0
            output += f"- Joint {name}: Position={pos:.4f} rad, Velocity={vel:.4f} rad/s\n"
        return output

    except Exception as e:
        return f"Error: {e}"
    finally:
        node.destroy_node()

@tool
def get_end_effector_pose(timeout_sec: float = 2.0) -> str:
    """
    Calculates and returns the current 3D pose [X, Y, Z] (in meters) of the robot end effector.
    Uses Direct Kinematics.
    """
    if not rclpy.ok():
        rclpy.init()

    node = rclpy.create_node(f'rosa_dk_fetcher_{int(time.time())}')
    received_msg = None

    def callback(msg):
        nonlocal received_msg
        received_msg = msg

    subscription = node.create_subscription(
        JointState,
        '/joint_states',
        callback,
        10
    )

    try:
        start_time = node.get_clock().now()
        while received_msg is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = node.get_clock().now() - start_time
            if elapsed.nanoseconds > (timeout_sec * 1e9):
                return "Error: Timeout waiting for joint states."
        
        # Extract joints
        try:
            idx1 = received_msg.name.index('joint1')
            idx2 = received_msg.name.index('joint2')
            idx3 = received_msg.name.index('joint3')
            b = received_msg.position[idx1]
            g = received_msg.position[idx2]
            a = received_msg.position[idx3]
        except ValueError:
            return "Error: Manipulator joints ['joint1', 'joint2', 'joint3'] not found in joint states."

        kin = Kinematics()
        x, y, z = kin.get_dk(b, g, a)
        return f"Current End Effector Position (meters):\nX: {x:.4f}\nY: {y:.4f}\nZ: {z:.4f}\nJoints: joint1={b:.4f}, joint2={g:.4f}, joint3={a:.4f}"

    except Exception as e:
        return f"Error: {e}"
    finally:
        node.destroy_node()

@tool
def plan_and_move_to_object(start_class: str, goal_class: str, avoid_classes: list[str], timeout_sec: float = 5.0) -> str:
    """
    Plans a 2D collision-free path from the start_class object to the goal_class object,
    avoiding all classes in avoid_classes, and commands the robot arm to execute the path.
    Example: plan_and_move_to_object(start_class='car', goal_class='traffic light', avoid_classes=['cat'])
    """
    if not rclpy.ok():
        rclpy.init()

    node = rclpy.create_node(f'rosa_planner_trigger_{int(time.time())}')
    
    # Setup path feedback subscription
    received_path_msg = None
    def path_callback(msg):
        nonlocal received_path_msg
        received_path_msg = msg

    path_sub = node.create_subscription(
        String,
        '/planning/path',
        path_callback,
        10
    )

    request_pub = node.create_publisher(String, '/planning/request', 10)

    try:
        # Construct planning request
        req_payload = {
            "start_class": start_class,
            "goal_class": goal_class,
            "avoid_classes": avoid_classes
        }
        
        # Publish request
        req_msg = String()
        req_msg.data = json.dumps(req_payload)
        
        time.sleep(0.2)
        request_pub.publish(req_msg)
        node.get_logger().info(f"Published plan request: {req_msg.data}")

        # Wait for planned path response
        start_time = node.get_clock().now()
        while received_path_msg is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = node.get_clock().now() - start_time
            if elapsed.nanoseconds > (timeout_sec * 1e9):
                return f"Timeout waiting for path planner response for request: {req_payload}. Is path_planner_node running?"
        
        path_data = json.loads(received_path_msg.data)
        waypoints = path_data.get("path", [])
        
        if not waypoints:
            return "Planner failed to find a valid collision-free path."

        return f"Successfully generated path with {len(waypoints)} points from {start_class} to {goal_class} avoiding {avoid_classes}. Command sent to robot controllers."

    except Exception as e:
        return f"Error executing plan and move: {str(e)}"
    finally:
        node.destroy_node()

TOOLS = [
    get_detected_objects,
    move_robot_joints,
    get_joint_states,
    get_end_effector_pose,
    plan_and_move_to_object
]
