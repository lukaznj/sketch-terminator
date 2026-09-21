#!/usr/bin/env python3
"""Drives the arm to Cartesian points by solving inverse kinematics.

Subscribes to ``/current_point`` -- a stream of marker tip positions produced by
``generate_smooth_path`` at 25 Hz -- solves IK for each one and forwards the
joint angles to ``joint_trajectory_controller``.

Each point is commanded with a very short time-from-start: the trajectory
controller is being used as a position pipe here, and the smoothness of the
motion comes from the incoming point stream rather than from interpolation
inside the controller.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from kinematics import Kinematics

# Order matters: it has to match the [beta, gama, alpha] order get_ik returns.
JOINT_NAMES = ['joint1', 'joint2', 'joint3']

# Matches the 25 Hz publish rate of generate_smooth_path. Commanding a slightly
# shorter horizon than the point period keeps the arm from lagging behind the
# stream and rounding off corners.
POINT_DURATION = Duration(seconds=0.04)


class MoveRobotNode(Node):
    def __init__(self):
        super().__init__('move_robot_node')
        self.kinematics = Kinematics()
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
        )
        self.create_subscription(Point, 'current_point', self.point_callback, 10)
        self.get_logger().info('Move robot node started, listening on /current_point...')

    def point_callback(self, msg: Point):
        # get_ik clamps anything outside the safe envelope, so a stray point
        # cannot throw here and break the stream mid-stroke.
        joint_angles = self.kinematics.get_ik(msg.x, msg.y, msg.z)
        self.send_trajectory(joint_angles)

    def send_trajectory(self, joint_angles):
        point = JointTrajectoryPoint()
        point.positions = list(joint_angles)
        point.time_from_start = POINT_DURATION.to_msg()

        trajectory = JointTrajectory()
        trajectory.joint_names = JOINT_NAMES
        trajectory.points.append(point)
        self.trajectory_pub.publish(trajectory)


def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
