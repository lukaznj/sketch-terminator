#!/usr/bin/env python3
"""Publishes the marker tip position derived from the measured joint angles.

Subscribes to ``/joint_states`` (the real encoder feedback from the Dynamixels)
and republishes the corresponding Cartesian position on ``/marker_end_point``.

``generate_smooth_path`` latches this once at startup to learn where the arm
actually is before it plans its first move.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

from kinematics import Kinematics

# Wire names of the three joints, in the order get_dk expects its arguments.
BETA_JOINT = 'joint1'   # shoulder
GAMA_JOINT = 'joint2'   # elbow
ALPHA_JOINT = 'joint3'  # base yaw


class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')
        self.kinematics = Kinematics()
        self.pub_end_point = self.create_publisher(Point, 'marker_end_point', 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.get_logger().info('Kinematics node started, listening on /joint_states...')

    def joint_state_callback(self, msg: JointState):
        angles = dict(zip(msg.name, msg.position))
        try:
            beta = angles[BETA_JOINT]
            gama = angles[GAMA_JOINT]
            alpha = angles[ALPHA_JOINT]
        except KeyError:
            # Other publishers may put partial joint states on the topic; wait
            # for one that carries all three of ours.
            self.get_logger().warn(
                f'Joint state is missing one of {BETA_JOINT}/{GAMA_JOINT}/{ALPHA_JOINT}',
                throttle_duration_sec=5.0,
            )
            return

        x, y, z = self.kinematics.get_dk(beta, gama, alpha)
        self.pub_end_point.publish(Point(x=x, y=y, z=z))


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
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
