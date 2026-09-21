#!/usr/bin/env python3
"""Turns a sparse planned path into a smooth, speed-limited stream of points.

The planner emits only corner points. Feeding those straight to the IK node
would make the arm jump between them at whatever speed the controller felt
like. This node walks along the path in fixed-size steps at a fixed rate, so
the tip moves at a constant, safe ``TARGET_SPEED``.

It also wraps the drawing stroke in the approach and retreat moves that keep
the marker off the paper while travelling, and parks the arm clear of the
camera when it is done:

  1. lift straight up from wherever the arm currently is
  2. cross to a transit point in front of the base
  3. move above the first point of the stroke
  4. lower onto the paper and draw the whole stroke
  5. pause, so the ink settles and the last point is actually reached
  6. lift, cross back, and park out of the camera's view

The node tracks a *virtual* pose rather than the measured one: it reads the
real arm position exactly once at startup and integrates its own commands from
there. That keeps the stream open-loop and jitter-free, at the cost of not
noticing if the arm is physically blocked.
"""

import json
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String

TARGET_SPEED = 0.1       # m/s along the path
TIMER_PERIOD = 0.04      # s between published points (25 Hz)
STEP_DISTANCE = TARGET_SPEED * TIMER_PERIOD   # 4 mm per step

ARRIVAL_TOLERANCE = 0.003   # m; how close counts as "reached this waypoint"
PEN_UP_Z = 0.09             # m; marker fully clear of the paper
PEN_DOWN_Z = 0.0            # m; marker touching the paper
TRANSIT_POINT = (0.15, 0.0)         # staging point crossed before and after a stroke
PARK_POINT = (0.0, 0.15)            # rest position, clear of the camera's view
END_OF_STROKE_PAUSE = 1.0           # s to dwell on the final drawn point


class GenerateSmoothPath(Node):
    def __init__(self):
        super().__init__('generate_smooth_path')

        self.wait_steps = int(END_OF_STROKE_PAUSE / TIMER_PERIOD)

        self.virtual_pose = None          # integrated tip position we command from
        self.is_busy = False              # a stroke is in progress; reject new ones
        self.execution_queue = []         # waypoints still to visit
        self.current_target_point = None
        self.last_drawing_point = [0.0, 0.0, 0.0]

        self.is_waiting = False
        self.wait_counter = 0

        # Read the real arm position once, then never again -- see module docstring.
        self.sub_initial_pose = self.create_subscription(
            Point, 'marker_end_point', self.initial_pose_callback, 10
        )
        self.create_subscription(String, '/planning/path', self.planned_path_callback, 10)
        self.pub_current_point = self.create_publisher(Point, 'current_point', 10)

        self.create_timer(TIMER_PERIOD, self.navigation_loop)
        self.get_logger().info('Smooth path generator started. Waiting for initial pose...')

    def initial_pose_callback(self, msg: Point):
        if self.virtual_pose is not None:
            return
        self.virtual_pose = [msg.x, msg.y, msg.z]
        self.get_logger().info(
            f'Captured initial pose: X={msg.x:.4f} Y={msg.y:.4f} Z={msg.z:.4f}'
        )
        # We only ever needed one sample; stop listening.
        self.destroy_subscription(self.sub_initial_pose)

    def planned_path_callback(self, msg: String):
        if self.virtual_pose is None:
            self.get_logger().warn('Path received, but the initial arm pose is not known yet.')
            return
        if self.is_busy:
            self.get_logger().warn('Still drawing the previous path; ignoring this request.')
            return

        try:
            path_json = json.loads(msg.data).get('path', [])
        except json.JSONDecodeError:
            self.get_logger().error('Could not parse the JSON on /planning/path.')
            return

        if not path_json:
            self.get_logger().warn('Received an empty path.')
            return

        stroke = [[float(pt['x']), float(pt['y']), PEN_DOWN_Z] for pt in path_json]
        self.execution_queue = self._build_execution_queue(stroke)
        self.last_drawing_point = stroke[-1]

        self.current_target_point = self.execution_queue.pop(0)
        self.is_busy = True
        self.get_logger().info(
            f'New drawing request ({len(stroke)} points). Motion started.'
        )

    def _build_execution_queue(self, stroke):
        """Wrap a stroke in its pen-up approach, retreat and park moves."""
        start_x, start_y, _ = self.virtual_pose
        first_x, first_y, _ = stroke[0]
        last_x, last_y, _ = stroke[-1]

        return [
            [start_x, start_y, PEN_UP_Z],              # lift off wherever we are
            [*TRANSIT_POINT, PEN_UP_Z],                # cross to the staging point
            [first_x, first_y, PEN_UP_Z],              # hover over the stroke start
            [first_x, first_y, PEN_DOWN_Z],            # touch down
            *stroke[1:],                               # draw
            # The pause happens here, handled by navigation_loop.
            [last_x, last_y, PEN_UP_Z],                # lift off the paper
            [*TRANSIT_POINT, PEN_UP_Z],                # cross back
            [*PARK_POINT, PEN_UP_Z],                   # park clear of the camera
        ]

    def navigation_loop(self):
        if not self.is_busy or self.virtual_pose is None or self.current_target_point is None:
            return

        if self.is_waiting:
            self.wait_counter += 1
            if self.wait_counter >= self.wait_steps:
                self.is_waiting = False
                self.wait_counter = 0
                self.get_logger().info('Pause finished, retreating to the park position.')
                self.current_target_point = (
                    self.execution_queue.pop(0) if self.execution_queue else None
                )
            return

        distance = self._distance_to_target()

        if distance < ARRIVAL_TOLERANCE:
            if self._is_at_end_of_stroke():
                self.is_waiting = True
                self.get_logger().info(
                    f'Stroke finished. Holding for {END_OF_STROKE_PAUSE:.1f} s...'
                )
                return

            if not self.execution_queue:
                self.get_logger().info('Motion finished. Parked and ready for a new path.')
                self.current_target_point = None
                self.is_busy = False
                return

            self.current_target_point = self.execution_queue.pop(0)
            distance = self._distance_to_target()

        # Take one fixed-size step towards the target, or snap onto it if the
        # remaining distance is shorter than a step.
        if distance > STEP_DISTANCE:
            scale = STEP_DISTANCE / distance
            for axis in range(3):
                delta = self.current_target_point[axis] - self.virtual_pose[axis]
                self.virtual_pose[axis] += delta * scale
        else:
            self.virtual_pose = list(self.current_target_point)

        x, y, z = self.virtual_pose
        self.pub_current_point.publish(Point(x=x, y=y, z=z))

    def _distance_to_target(self):
        return math.dist(self.virtual_pose, self.current_target_point)

    def _is_at_end_of_stroke(self):
        """True when the tip is sitting on the last drawn point, still pen-down."""
        return (
            math.dist(self.virtual_pose[:2], self.last_drawing_point[:2]) < 0.005
            and abs(self.virtual_pose[2] - PEN_DOWN_Z) < 0.005
        )


def main(args=None):
    rclpy.init(args=args)
    node = GenerateSmoothPath()
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
