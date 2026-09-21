#!/usr/bin/env python3
"""Measures where the camera sits relative to the robot base.

Finds the checkerboard in the camera image, solves PnP against the board's
known position in the robot's frame, and prints the resulting camera-to-world
transform. Those numbers go into ``R_cam_to_robot`` and ``t_cam_to_robot`` in
``config/vision_config.yaml``, which is what lets the vision node turn pixels
into millimetres.

As a sanity check it also hunts for a red circular marker and prints where it
thinks that marker is in world coordinates -- put the marker somewhere you have
measured, and the printed value should match.

Run it through ``camera_world.launch.py``; the board geometry comes from
``config/calib_config.yaml``.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

# HSV-ish bounds in Lab space for the red validation marker.
RED_LAB_LOWER = np.array([70, 150, 100])
RED_LAB_UPPER = np.array([100, 255, 255])

AXIS_LENGTH_M = 0.05    # length of the drawn coordinate axes


def board_points_in_robot_frame(board_dims, square_size, x_offset, y_offset,
                                rotation_diagonal):
    """Corner positions of the checkerboard, expressed in the robot base frame.

    OpenCV wants the board's corners in world coordinates. The board is taped
    down at a known offset from the base with its axes flipped relative to the
    robot's, so the grid is built in board coordinates and then transformed.

    ``rotation_diagonal`` is the diagonal of that rotation -- the board is
    axis-aligned with the robot, just mirrored, so a full rotation matrix would
    be three signs and six zeros.
    """
    width, height = board_dims

    corners = np.zeros((width * height, 3), np.float32)
    corners[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
    corners *= square_size

    transform = np.eye(4)
    transform[0, 0], transform[1, 1], transform[2, 2] = rotation_diagonal
    # The offsets are measured to the board's far corner, hence the extra
    # squares: the grid is built from the opposite corner.
    transform[0, 3] = x_offset + square_size * (width + 3)
    transform[1, 3] = y_offset - square_size * (height + 1)

    homogeneous = np.hstack((corners, np.ones((corners.shape[0], 1))))
    return transform.dot(homogeneous.T)[:3, :].T.astype(np.float32)


def image_to_world(image_point, camera_intrinsics, T_camera_world, z=0.0):
    """Project a pixel onto the ``z`` plane of the world frame."""
    R = T_camera_world[:3, :3]
    t = T_camera_world[:3, 3]

    R_inv = np.linalg.inv(R)
    intrinsics_inv = np.linalg.inv(camera_intrinsics)
    inverse_P = np.linalg.inv(camera_intrinsics.dot(R))
    unrotated_t = R_inv.dot(t)

    point_h = np.array([image_point[0], image_point[1], 1.0]).reshape((3, 1))

    # Scale the ray so it lands on the requested plane:
    #   s * (inverse_P @ point_h)[2] = z + unrotated_t[2]
    scale = (z + unrotated_t[2]) / inverse_P.dot(point_h)[2]
    return R_inv.dot(intrinsics_inv.dot(point_h * scale) - t)


def draw_axes(image, image_points):
    """Draw the board's coordinate axes: X blue, Y green, Z red."""
    image_points = image_points.astype('int32')
    origin = tuple(image_points[0].ravel())
    for index, color in ((3, (255, 0, 0)), (2, (0, 255, 0)), (1, (0, 0, 255))):
        image = cv2.line(image, origin, tuple(image_points[index].ravel()), color, 5)
    return image


class CamToWorld(Node):
    def __init__(self):
        super().__init__('cam_to_world')

        # Checkerboard geometry: inner corners, square size in metres, and the
        # board's offset from the robot base. All measured on the real rig.
        self.declare_parameter('checkerboard_width', 9)
        self.declare_parameter('checkerboard_height', 7)
        self.declare_parameter('square_size', 0.0175)
        self.declare_parameter('x_offset', 0.0423)
        self.declare_parameter('y_offset', 0.1016)
        self.declare_parameter('rotation_diagonal', [-1.0, 1.0, -1.0])

        self.bridge = CvBridge()
        self.k_matrix = None
        self.distortion_params = None
        self.logged_board_setup = False

        self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera_info', self.camera_info_callback, 10
        )

    def camera_info_callback(self, data):
        self.k_matrix = np.reshape(np.array(data.k), (3, 3))
        self.distortion_params = np.array(data.d)
        self.get_logger().info('Received camera intrinsics.')
        # Intrinsics never change; one message is enough.
        self.destroy_subscription(self.camera_info_sub)

    def board_parameters(self):
        get = self.get_parameter
        return (
            (get('checkerboard_width').value, get('checkerboard_height').value),
            get('square_size').value,
            get('x_offset').value,
            get('y_offset').value,
            list(get('rotation_diagonal').value),
        )

    def image_callback(self, data):
        if self.k_matrix is None:
            self.get_logger().info('Waiting for camera info...', throttle_duration_sec=5.0)
            return

        # ROS publishes RGB, OpenCV works in BGR.
        raw_frame = self.bridge.imgmsg_to_cv2(data)
        image = cv2.cvtColor(raw_frame, cv2.COLOR_RGB2BGR)

        board_dims, square_size, x_offset, y_offset, rotation_diagonal = \
            self.board_parameters()

        if not self.logged_board_setup:
            self.get_logger().info(
                f'Board {board_dims[0]}x{board_dims[1]} inner corners, '
                f'{square_size} m squares, offset ({x_offset}, {y_offset}) m'
            )
            self.logged_board_setup = True

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, board_dims, None)
        if found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(image, board_dims, corners, found)
            self.solve_and_report(image, corners, board_dims, square_size,
                                  x_offset, y_offset, rotation_diagonal)

        cv2.imshow('Camera to world calibration', image)
        cv2.waitKey(1)

    def solve_and_report(self, image, corners, board_dims, square_size,
                         x_offset, y_offset, rotation_diagonal):
        world_points = board_points_in_robot_frame(
            board_dims, square_size, x_offset, y_offset, rotation_diagonal
        )
        solved, rvecs, tvecs = cv2.solvePnP(
            world_points, corners, self.k_matrix, self.distortion_params,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not solved:
            return

        axis = np.float32([
            [0, 0, 0], [AXIS_LENGTH_M, 0, 0], [0, AXIS_LENGTH_M, 0], [0, 0, AXIS_LENGTH_M]
        ]).reshape(-1, 3)
        image_points, _ = cv2.projectPoints(
            axis, rvecs, tvecs, self.k_matrix, self.distortion_params
        )
        draw_axes(image, image_points)

        T_cam_world = np.eye(4)
        T_cam_world[:3, :3] = cv2.Rodrigues(rvecs)[0]
        T_cam_world[:3, 3] = tvecs.T
        self.get_logger().info(
            'Camera to world transform (copy into vision_config.yaml):\n'
            + np.array2string(T_cam_world, precision=5, separator=',', suppress_small=True)
        )

        self.report_red_marker(image, T_cam_world)

    def report_red_marker(self, image, T_cam_world):
        """Locate a red circular marker and print its world position.

        This is the sanity check: place the marker at a position you have
        measured, and confirm the printed coordinates agree.
        """
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2Lab)
        mask = cv2.inRange(lab, RED_LAB_LOWER, RED_LAB_UPPER)
        mask = cv2.GaussianBlur(mask, (5, 5), 2, 2)

        circles = cv2.HoughCircles(
            mask, cv2.HOUGH_GRADIENT, 1, mask.shape[0] / 8,
            param1=100, param2=18, minRadius=5, maxRadius=60,
        )
        if circles is None:
            return

        for x, y, _ in circles[0]:
            world = image_to_world(np.array([x, y]), self.k_matrix, T_cam_world, z=0.0)
            self.get_logger().info(
                'Red marker at:\n'
                + np.array2string(world, precision=5, suppress_small=True)
            )

        for x, y, radius in np.round(circles[0, :]).astype('int'):
            cv2.circle(image, (x, y), radius, (0, 255, 0), 2)


def main(args=None):
    rclpy.init(args=args)
    node = CamToWorld()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
