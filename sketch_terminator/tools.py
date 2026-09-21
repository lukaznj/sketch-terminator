#!/usr/bin/env python3
"""LangChain tools that let the ROSA agent inspect and drive the robot.

Each tool is a short-lived ROS client: it spins up a throwaway node, does one
request or one wait, and tears the node down. That keeps the tools independent
of the agent node's own executor, which is busy streaming LLM tokens, at the
cost of a little discovery latency per call.

Docstrings matter here -- they are what the model reads to decide which tool to
call and with what arguments, so they are written for the model, not just for
the reader.
"""

import json
import time

import rclpy
from langchain.agents import tool
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from kinematics import Kinematics

# Order matters: it matches the [beta, gama, alpha] order get_ik returns.
JOINT_NAMES = ['joint1', 'joint2', 'joint3']

# Hard clamp on anything the model asks for. The arm's own limits are tighter
# still (see kinematics.py); this is only here so a hallucinated angle cannot
# be forwarded to the controller verbatim.
JOINT_LIMIT = 3.14

# New publishers need a moment for discovery before their first message will
# actually reach anyone.
DISCOVERY_GRACE_SEC = 0.2


def _wait_for_message(msg_type, topic, timeout_sec):
    """Block until one message arrives on ``topic``, or return None on timeout.

    rclpy has no synchronous "read one message" call, so this subscribes on a
    temporary node and spins it until something lands.
    """
    if not rclpy.ok():
        rclpy.init()

    node = rclpy.create_node(f'rosa_tool_{int(time.time() * 1000)}')
    received = None

    def callback(msg):
        nonlocal received
        received = msg

    node.create_subscription(msg_type, topic, callback, 10)

    try:
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        deadline = node.get_clock().now().nanoseconds + timeout_sec * 1e9
        while received is None:
            executor.spin_once(timeout_sec=0.1)
            if node.get_clock().now().nanoseconds > deadline:
                return None
        return received
    finally:
        node.destroy_node()


def _joint_positions(joint_state):
    """Pull our three joints out of a JointState, or None if any is missing."""
    angles = dict(zip(joint_state.name, joint_state.position))
    try:
        return [angles[name] for name in JOINT_NAMES]
    except KeyError:
        return None


@tool
def get_detected_objects(timeout_sec: float = 3.0) -> str:
    """Lists the objects the camera can currently see.

    Returns each object's class and its position in the robot base frame, in
    metres. Use this to find out what is on the paper before planning a move.
    """
    msg = _wait_for_message(String, '/vision/object_positions', timeout_sec)
    if msg is None:
        return ('No data on /vision/object_positions. '
                'Is the vision pipeline running?')

    objects = json.loads(msg.data).get('objects', [])
    if not objects:
        return 'The camera sees no objects right now.'

    lines = ['Detected objects (robot base frame, metres):']
    lines += [
        f"- {obj.get('class')} at X={obj.get('x')}, Y={obj.get('y')}, bbox={obj.get('bbox')}"
        for obj in objects
    ]
    return '\n'.join(lines)


@tool
def get_joint_states(timeout_sec: float = 2.0) -> str:
    """Reads the current angle and speed of every joint, in radians."""
    msg = _wait_for_message(JointState, '/joint_states', timeout_sec)
    if msg is None:
        return ('Timed out waiting for /joint_states. '
                'Is the robot hardware or simulation running?')

    lines = ['Current joint states:']
    for i, name in enumerate(msg.name):
        position = msg.position[i] if i < len(msg.position) else 0.0
        velocity = msg.velocity[i] if i < len(msg.velocity) else 0.0
        lines.append(f'- {name}: position={position:.4f} rad, velocity={velocity:.4f} rad/s')
    return '\n'.join(lines)


@tool
def get_end_effector_pose(timeout_sec: float = 2.0) -> str:
    """Reports where the marker tip is right now, as [X, Y, Z] in metres.

    Derived from the measured joint angles via forward kinematics.
    """
    msg = _wait_for_message(JointState, '/joint_states', timeout_sec)
    if msg is None:
        return 'Timed out waiting for /joint_states.'

    positions = _joint_positions(msg)
    if positions is None:
        return f'Joints {JOINT_NAMES} are not present in /joint_states.'

    beta, gama, alpha = positions
    x, y, z = Kinematics().get_dk(beta, gama, alpha)
    return (f'Marker tip position (metres): X={x:.4f}, Y={y:.4f}, Z={z:.4f}\n'
            f'Joints: joint1={beta:.4f}, joint2={gama:.4f}, joint3={alpha:.4f}')


@tool
def move_robot_joints(positions: list[float], duration: float = 3.0) -> str:
    """Moves the three joints straight to the given angles, in radians.

    Takes exactly three values, in the order [joint1, joint2, joint3]:
    joint1 is the shoulder, joint2 the elbow, joint3 the base rotation.
    This bypasses path planning -- prefer plan_and_move_to_object for anything
    involving objects on the paper.
    """
    if len(positions) != 3:
        return f'Expected exactly 3 joint angles for {JOINT_NAMES}, got {len(positions)}.'

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node(f'rosa_mover_{int(time.time() * 1000)}')

    try:
        publisher = node.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
        )

        point = JointTrajectoryPoint()
        point.positions = [
            max(-JOINT_LIMIT, min(JOINT_LIMIT, float(angle))) for angle in positions
        ]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        msg.points.append(point)

        time.sleep(DISCOVERY_GRACE_SEC)
        publisher.publish(msg)
        # Let the message actually leave before the node is destroyed.
        time.sleep(0.1)

        return f'Commanded joint angles: {point.positions}'
    except Exception as exc:
        return f'Failed to move the joints: {exc}'
    finally:
        node.destroy_node()


@tool
def plan_and_move_to_object(
    start_class: str, goal_class: str, avoid_classes: list[str], timeout_sec: float = 5.0
) -> str:
    """Draws a line from one object to another, steering around obstacles.

    Plans a collision-free path from the object of class ``start_class`` to the
    object of class ``goal_class``, keeping clear of every class listed in
    ``avoid_classes``, and executes it. The arm lifts the marker while it
    travels, draws the stroke, then parks clear of the camera.

    Example: plan_and_move_to_object('car', 'traffic light', ['cat'])
    """
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node(f'rosa_planner_{int(time.time() * 1000)}')

    received_path = None

    def path_callback(msg):
        nonlocal received_path
        received_path = msg

    try:
        node.create_subscription(String, '/planning/path', path_callback, 10)
        request_pub = node.create_publisher(String, '/planning/request', 10)

        request = {
            'start_class': start_class,
            'goal_class': goal_class,
            'avoid_classes': avoid_classes,
        }
        msg = String()
        msg.data = json.dumps(request)

        time.sleep(DISCOVERY_GRACE_SEC)
        request_pub.publish(msg)

        # The planner answers on /planning/path; the motion nodes pick that up
        # on their own, so seeing the path is enough to confirm the move.
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        deadline = node.get_clock().now().nanoseconds + timeout_sec * 1e9
        while received_path is None:
            executor.spin_once(timeout_sec=0.1)
            if node.get_clock().now().nanoseconds > deadline:
                return (f'The path planner did not answer within {timeout_sec} s '
                        f'for {request}. Is path_planner_node running?')

        waypoints = json.loads(received_path.data).get('path', [])
        if not waypoints:
            return 'The planner could not find a collision-free path.'

        return (f'Planned a {len(waypoints)}-point path from {start_class} to '
                f'{goal_class} avoiding {avoid_classes}. The arm is drawing it now.')
    except Exception as exc:
        return f'Failed to plan and move: {exc}'
    finally:
        node.destroy_node()


TOOLS = [
    get_detected_objects,
    get_joint_states,
    get_end_effector_pose,
    move_robot_joints,
    plan_and_move_to_object,
]
