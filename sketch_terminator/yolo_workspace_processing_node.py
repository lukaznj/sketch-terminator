#!/usr/bin/env python3
"""Projects YOLO detections from image pixels into robot base coordinates.

The camera looks down at the paper from a fixed mount. Because every object of
interest lies flat on the paper, a single ray-plane intersection is enough to
recover a full 3D position from a 2D pixel -- no depth sensor needed. Each
detection's bounding-box centre and its four corners are projected onto the
``z = 0`` plane and published as JSON for the planner.

The node also renders a debug overlay: the base frame's origin and axes, the
park position, every detection's real-world coordinates, and the latest planned
path drawn back into the camera image.

Topics
------
in   ``/yolo/detections``          DetectionArray from ``yolo_node``
in   ``/planning/path``            latest planned path, for the overlay
in   ``/dbg_image``                annotated camera frame from ``yolo_node``
out  ``/vision/object_positions``  detections in the robot base frame (JSON)
out  ``/planning/debug_image``     the overlay image

Calibration parameters come from ``config/vision_config.yaml``; see
``scripts/camera_to_world.py`` for how they are measured.
"""

import json

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from yolo_msgs.msg import DetectionArray

# Overlay colours, BGR.
COLOR_ORIGIN = (0, 0, 255)      # red: base origin and the X axis
COLOR_Y_AXIS = (0, 255, 0)      # green: Y axis, also the path start marker
COLOR_PARK = (255, 255, 0)      # cyan: park position
COLOR_PATH_NODE = (255, 0, 255) # magenta: path waypoints
COLOR_PATH_LINE = (0, 255, 255) # yellow: path segments

AXIS_LENGTH_M = 0.1     # how long to draw the base frame axes
PARK_POINT_M = (0.0, 0.15)   # must match PARK_POINT in generate_smooth_path.py


class YoloWorkspaceProcessingNode(Node):
    def __init__(self):
        super().__init__('yolo_workspace_processing_node')

        self.declare_parameter('yolo_detections_topic', '/yolo/detections')
        self.declare_parameter('positions_topic', '/vision/object_positions')

        # Intrinsics, from the rectified projection matrix produced by
        # camera_calibration (see config/camera_calibration_params.yaml).
        self.declare_parameter('fx', 721.29907)
        self.declare_parameter('fy', 716.27563)
        self.declare_parameter('cx', 344.87706)
        self.declare_parameter('cy', 292.58597)

        # Extrinsics from the checkerboard calibration: the 3x3 rotation is
        # flattened row-major into nine values, and the translation is in
        # millimetres. Together they are the OpenCV-convention world->camera
        # transform, P_cam = R * P_world + t.
        self.declare_parameter('R_cam_to_robot', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('t_cam_to_robot', [0.0, 0.0, 0.0])

        # Residual offset applied after projection, in metres. This is the knob
        # for dialling out the systematic error left over once the checkerboard
        # calibration is done -- adjust it against a measured test point rather
        # than re-deriving the extrinsics.
        self.declare_parameter('offset_x_m', -0.04)
        self.declare_parameter('offset_y_m', 0.0)

        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        self.R_cam_to_robot = np.array(
            self.get_parameter('R_cam_to_robot').value, dtype=np.float64
        ).reshape(3, 3)
        self.t_cam_to_robot = np.array(
            self.get_parameter('t_cam_to_robot').value, dtype=np.float64
        )
        self.offset_x_m = self.get_parameter('offset_x_m').value
        self.offset_y_m = self.get_parameter('offset_y_m').value

        self.get_logger().info('Loaded camera calibration from the configuration file.')
        self.get_logger().info(
            f'Camera height above the paper: {self.t_cam_to_robot[2]:.1f} mm'
        )

        self.bridge = CvBridge()
        self.latest_path = []
        self.latest_objects = []

        self.create_subscription(
            DetectionArray,
            self.get_parameter('yolo_detections_topic').value,
            self.yolo_callback,
            10,
        )
        self.positions_pub = self.create_publisher(
            String, self.get_parameter('positions_topic').value, 10
        )

        self.create_subscription(String, '/planning/path', self.path_callback, 10)
        self.create_subscription(Image, '/dbg_image', self.image_callback, 10)
        self.debug_image_pub = self.create_publisher(Image, '/planning/debug_image', 10)

    # --- Projection ------------------------------------------------------

    def pixel_to_robot_frame(self, u, v):
        """Project a pixel onto the paper plane; returns (x, y) in metres.

        Casts the ray through the pixel out of the camera and intersects it
        with the ``z = 0`` plane of the robot base frame. Solving for the two
        world coordinates and the ray scale at once turns this into a single
        3x3 linear system.
        """
        # Ray direction in camera coordinates (OpenCV convention: +Z forward).
        ray_cam = np.array([
            (u - self.cx) / self.fx,
            (v - self.cy) / self.fy,
            1.0,
        ], dtype=np.float64)

        R = self.R_cam_to_robot
        t = self.t_cam_to_robot

        # With P_cam = R * P_world + t and P_world = (X, Y, 0), a point on the
        # ray satisfies  s * ray_cam = X * R[:,0] + Y * R[:,1] + t,
        # i.e.  [R[:,0]  R[:,1]  -ray_cam] @ [X, Y, s] = -t.
        M = np.column_stack((R[:, 0], R[:, 1], -ray_cam))
        try:
            x_mm, y_mm, _ = np.linalg.solve(M, -t)
        except np.linalg.LinAlgError:
            raise RuntimeError('Ray is parallel to the paper; cannot project this pixel.')

        return (
            float(x_mm / 1000.0 + self.offset_x_m),
            float(y_mm / 1000.0 + self.offset_y_m),
        )

    def robot_to_pixel_frame(self, x, y):
        """Inverse of :meth:`pixel_to_robot_frame`, for drawing the overlay."""
        p_world = np.array([
            (x - self.offset_x_m) * 1000.0,
            (y - self.offset_y_m) * 1000.0,
            0.0,
        ], dtype=np.float64)

        p_cam = self.R_cam_to_robot.dot(p_world) + self.t_cam_to_robot
        if p_cam[2] <= 0:
            return 0, 0     # behind the camera, nothing sensible to draw

        return (
            int(p_cam[0] / p_cam[2] * self.fx + self.cx),
            int(p_cam[1] / p_cam[2] * self.fy + self.cy),
        )

    # --- Callbacks -------------------------------------------------------

    def yolo_callback(self, msg):
        objects_data = []

        for detection in msg.detections:
            cx = detection.bbox.center.position.x
            cy = detection.bbox.center.position.y
            w = detection.bbox.size.x
            h = detection.bbox.size.y

            corners_pixels = {
                'top_left': (cx - w / 2, cy - h / 2),
                'top_right': (cx + w / 2, cy - h / 2),
                'bottom_right': (cx + w / 2, cy + h / 2),
                'bottom_left': (cx - w / 2, cy + h / 2),
            }

            try:
                x_robot, y_robot = self.pixel_to_robot_frame(cx, cy)
                corners_robot = {
                    name: self.pixel_to_robot_frame(px, py)
                    for name, (px, py) in corners_pixels.items()
                }
            except RuntimeError as exc:
                # A degenerate calibration affects every detection equally, so
                # there is no point processing the rest of this frame.
                self.get_logger().error(str(exc))
                return

            # Projection is not axis-preserving -- the image rectangle becomes a
            # quadrilateral in the base frame -- so take its axis-aligned hull.
            xs = [p[0] for p in corners_robot.values()]
            ys = [p[1] for p in corners_robot.values()]

            objects_data.append({
                'class': detection.class_name,
                'x': round(x_robot, 4),
                'y': round(y_robot, 4),
                'bbox': {
                    'x_min': round(min(xs), 4),
                    'y_min': round(min(ys), 4),
                    'x_max': round(max(xs), 4),
                    'y_max': round(max(ys), 4),
                },
                'bbox_corners': {
                    name: {'x': round(px, 4), 'y': round(py, 4)}
                    for name, (px, py) in corners_robot.items()
                },
                'px': int(cx),
                'py': int(cy),
            })

        self.latest_objects = objects_data
        self.publish_object_positions(objects_data)

    def publish_object_positions(self, objects_data):
        msg_out = String()
        msg_out.data = json.dumps({
            'frame': 'robot_base',
            'units': 'm',
            'objects': objects_data,
        })
        self.positions_pub.publish(msg_out)

    def path_callback(self, msg):
        try:
            self.latest_path = json.loads(msg.data).get('path', [])
        except json.JSONDecodeError:
            self.get_logger().error('Could not decode the path JSON.')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.draw_base_frame(cv_image)
            self.draw_object_coordinates(cv_image)
            self.draw_path(cv_image)

            out_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            out_msg.header = msg.header
            self.debug_image_pub.publish(out_msg)
        except Exception as exc:
            self.get_logger().error(f'Failed to render the debug overlay: {exc}')

    # --- Debug overlay ---------------------------------------------------

    def draw_base_frame(self, image):
        """Draw the robot's origin, its X/Y axes and the park position."""
        origin = self.robot_to_pixel_frame(0.0, 0.0)
        cv2.drawMarker(image, origin, COLOR_ORIGIN, cv2.MARKER_CROSS, 30, 3)
        cv2.putText(image, 'ORIGIN (0, 0)', (origin[0] + 10, origin[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_ORIGIN, 2)

        for end_m, color, label in (
            ((AXIS_LENGTH_M, 0.0), COLOR_ORIGIN, 'X'),
            ((0.0, AXIS_LENGTH_M), COLOR_Y_AXIS, 'Y'),
        ):
            tip = self.robot_to_pixel_frame(*end_m)
            cv2.arrowedLine(image, origin, tip, color, 3, tipLength=0.2)
            cv2.putText(image, label, (tip[0] + 5, tip[1] + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        park = self.robot_to_pixel_frame(*PARK_POINT_M)
        cv2.drawMarker(image, park, COLOR_PARK, cv2.MARKER_CROSS, 20, 2)
        cv2.putText(image, f'HOME {PARK_POINT_M}', (park[0] + 10, park[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_PARK, 2)

    def draw_object_coordinates(self, image):
        """Label each detection with its position in metres."""
        for obj in self.latest_objects:
            px, py = obj.get('px'), obj.get('py')
            if px is None or py is None:
                continue
            text = f"{obj['class']}: ({obj['x']:.2f}, {obj['y']:.2f})"
            # Black outline then white fill, so the text stays readable over
            # whatever happens to be under it.
            for color, thickness in (((0, 0, 0), 3), ((255, 255, 255), 1)):
                cv2.putText(image, text, (px - 40, py + 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness)

    def draw_path(self, image):
        """Draw the latest planned path back into the camera image."""
        if not self.latest_path:
            return

        pixels = [self.robot_to_pixel_frame(p['x'], p['y']) for p in self.latest_path]
        for previous, current in zip(pixels, pixels[1:]):
            cv2.line(image, previous, current, COLOR_PATH_LINE, 3)
        for pixel in pixels:
            cv2.circle(image, pixel, 6, COLOR_PATH_NODE, -1)

        if len(pixels) >= 2:
            cv2.circle(image, pixels[0], 10, COLOR_Y_AXIS, -1)    # start, green
            cv2.circle(image, pixels[-1], 10, COLOR_ORIGIN, -1)   # goal, red


def main(args=None):
    rclpy.init(args=args)
    node = YoloWorkspaceProcessingNode()
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
