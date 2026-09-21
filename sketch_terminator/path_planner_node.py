#!/usr/bin/env python3
"""ROS wrapper around the visibility-graph planner.

Keeps the latest set of detected objects from the vision pipeline, and on
request plans a path from one object class to another while avoiding a named
set of obstacle classes.

All the geometry lives in ``visibility_graph.py``; this file only does ROS
plumbing and JSON.

Topics
------
in   ``/vision/object_positions``  detections in the robot base frame (JSON)
in   ``/planning/request``         {"start_class", "goal_class", "avoid_classes"}
out  ``/planning/path``            the planned polyline (JSON)
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from visibility_graph import expand_bbox, plan_path


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        self.declare_parameter('vision_topic', '/vision/object_positions')
        self.declare_parameter('request_topic', '/planning/request')
        self.declare_parameter('path_topic', '/planning/path')
        # How far to inflate every obstacle, in metres. 3 cm covers the width of
        # the pen holder plus the detector's bounding-box slop.
        self.declare_parameter('planning_margin_m', 0.03)

        self.latest_vision_data = None

        self.create_subscription(
            String, self.get_parameter('vision_topic').value, self.vision_callback, 10
        )
        self.create_subscription(
            String, self.get_parameter('request_topic').value, self.request_callback, 10
        )
        self.path_pub = self.create_publisher(
            String, self.get_parameter('path_topic').value, 10
        )

        self.get_logger().info('A* visibility-graph planner started.')

    def vision_callback(self, ros_msg):
        try:
            self.latest_vision_data = json.loads(ros_msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Could not parse the vision JSON.')

    def request_callback(self, ros_msg):
        if self.latest_vision_data is None:
            self.get_logger().warn('Plan requested, but no camera data has arrived yet.')
            return

        try:
            request = json.loads(ros_msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Could not parse the request JSON.')
            return

        start_class = request.get('start_class')
        goal_class = request.get('goal_class')
        avoid_classes = request.get('avoid_classes', [])

        if not start_class or not goal_class:
            self.get_logger().warn('Request is missing start_class or goal_class.')
            return

        # Tolerate a bare string where a list was expected -- the LLM agent
        # produces both.
        if isinstance(avoid_classes, str):
            avoid_classes = [avoid_classes]

        self.plan_from_latest_vision(start_class, goal_class, avoid_classes)

    def plan_from_latest_vision(self, start_class, goal_class, avoid_classes):
        data = self.latest_vision_data
        objects = data.get('objects', [])
        margin = float(self.get_parameter('planning_margin_m').value)

        start_obj = find_first_object(objects, start_class)
        goal_obj = find_first_object(objects, goal_class)

        if not start_obj or not goal_obj:
            self.get_logger().warn(
                f'Could not find start ({start_class}) or goal ({goal_class}) in view.'
            )
            return

        start = (float(start_obj['x']), float(start_obj['y']))
        goal = (float(goal_obj['x']), float(goal_obj['y']))
        obstacles = [
            expand_bbox(obj['bbox'], margin)
            for obj in objects
            if obj.get('class') in avoid_classes and 'bbox' in obj
        ]

        path = plan_path(start, goal, obstacles)

        out_msg = String()
        out_msg.data = json.dumps({
            'frame': data.get('frame', 'robot_base'),
            'units': data.get('units', 'm'),
            'start_class': start_class,
            'goal_class': goal_class,
            'avoid_classes': avoid_classes,
            'path': [{'x': round(p[0], 4), 'y': round(p[1], 4)} for p in path],
        })
        self.path_pub.publish(out_msg)
        self.get_logger().info(
            f'Published a path with {len(path)} points, avoiding {len(obstacles)} obstacles.'
        )


def find_first_object(objects, class_name):
    return next((obj for obj in objects if obj.get('class') == class_name), None)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
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
