#!/usr/bin/env python3

import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PathPlannerNode(Node):
    """
    Request-based 2D path-planning node za seminar.

    Ulazi:
      /vision/object_positions  (std_msgs/String, JSON)
        - stalno prima objekte iz camera/vision nodea
        - sprema zadnju dobru listu objekata

      /planning/request  (std_msgs/String, JSON)
        - GUI/ROSA posalje sto treba spojiti i sto treba izbjeci
        - primjer:
          {
            "start_class": "car",
            "goal_class": "traffic light",
            "avoid_classes": ["cat"]
          }

    Izlaz:
      /planning/path  (std_msgs/String, JSON)
        - lista tocaka/waypointa u robot_base koordinatama, u mm
    """

    def __init__(self):
        super().__init__("path_planner_node")

        self.declare_parameter("vision_topic", "/vision/object_positions")
        self.declare_parameter("request_topic", "/planning/request")
        self.declare_parameter("path_topic", "/planning/path")

        # Dodatna margina oko obstacle bboxa u mm.
        self.declare_parameter("planning_margin_mm", 5.0)

        vision_topic = self.get_parameter("vision_topic").value
        request_topic = self.get_parameter("request_topic").value
        path_topic = self.get_parameter("path_topic").value

        self.latest_vision_data = None

        self.vision_sub = self.create_subscription(
            String,
            vision_topic,
            self.vision_callback,
            10
        )

        self.request_sub = self.create_subscription(
            String,
            request_topic,
            self.request_callback,
            10
        )

        self.path_pub = self.create_publisher(
            String,
            path_topic,
            10
        )

        self.get_logger().info(f"Subscribed to vision: {vision_topic}")
        self.get_logger().info(
            f"Subscribed to planning requests: {request_topic}")
        self.get_logger().info(f"Publishing path to: {path_topic}")

    def vision_callback(self, ros_msg):
        """
        Samo sprema zadnje vision podatke.
        Ovdje se NE planira path, da se path ne bi stalno mijenjao
        dok kamera radi ili dok robot zaklanja objekte.
        """
        try:
            data = json.loads(ros_msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Could not parse /vision/object_positions JSON.")
            return

        self.latest_vision_data = data

    def request_callback(self, ros_msg):
        """
        Planira putanju samo kad GUI/ROSA posalje zahtjev.
        """
        if self.latest_vision_data is None:
            self.get_logger().warn("No vision data received yet. Cannot plan path.")
            return

        try:
            request = json.loads(ros_msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Could not parse /planning/request JSON.")
            return

        start_class = request.get("start_class")
        goal_class = request.get("goal_class")
        avoid_classes = request.get("avoid_classes", [])

        if not start_class or not goal_class:
            self.get_logger().error(
                "Planning request must contain 'start_class' and 'goal_class'."
            )
            return

        if isinstance(avoid_classes, str):
            avoid_classes = [avoid_classes]

        self.plan_from_latest_vision(start_class, goal_class, avoid_classes)

    def plan_from_latest_vision(self, start_class, goal_class, avoid_classes):
        data = self.latest_vision_data
        objects = data.get("objects", [])
        margin = float(self.get_parameter("planning_margin_mm").value)

        start_obj = self.find_first_object(objects, start_class)
        goal_obj = self.find_first_object(objects, goal_class)

        if start_obj is None:
            self.get_logger().warn(f"Start object not found: {start_class}")
            return

        if goal_obj is None:
            self.get_logger().warn(f"Goal object not found: {goal_class}")
            return

        start = (float(start_obj["x"]), float(start_obj["y"]))
        goal = (float(goal_obj["x"]), float(goal_obj["y"]))

        obstacles = []
        for obj in objects:
            if obj.get("class") in avoid_classes and "bbox" in obj:
                obstacles.append(self.expand_bbox(obj["bbox"], margin))

        path = self.plan_path(start, goal, obstacles)

        output = {
            "frame": data.get("frame", "robot_base"),
            "units": data.get("units", "mm"),
            "start_class": start_class,
            "goal_class": goal_class,
            "avoid_classes": avoid_classes,
            "path": [
                {"x": round(p[0], 1), "y": round(p[1], 1)}
                for p in path
            ]
        }

        out_msg = String()
        out_msg.data = json.dumps(output)
        self.path_pub.publish(out_msg)

        self.get_logger().info(f"Planned path: {out_msg.data}")

    def find_first_object(self, objects, class_name):
        """
        Vrati prvi detektirani objekt zadane klase.
        Ako YOLO detektira vise istih objekata, zasad uzima prvi.
        """
        for obj in objects:
            if obj.get("class") == class_name:
                return obj
        return None

    def expand_bbox(self, bbox, margin):
        """
        Prosiri obstacle bbox za marginu.
        """
        return {
            "x_min": float(bbox["x_min"]) - margin,
            "y_min": float(bbox["y_min"]) - margin,
            "x_max": float(bbox["x_max"]) + margin,
            "y_max": float(bbox["y_max"]) + margin,
        }

    def plan_path(self, start, goal, obstacles):
        """
        Planira putanju od start do goal.
        Ako ravna linija ne sijece obstacle, vrati [start, goal].
        Ako sijece, ubaci waypoint oko prepreke.
        """
        path = [start]
        current = start

        for obstacle in obstacles:
            if self.segment_intersects_bbox(current, goal, obstacle):
                waypoint = self.choose_best_waypoint(
                    current,
                    goal,
                    obstacle,
                    obstacles
                )

                if waypoint is not None:
                    path.append(waypoint)
                    current = waypoint
                else:
                    self.get_logger().warn(
                        "Could not find valid waypoint around obstacle."
                    )

        path.append(goal)
        return path

    def choose_best_waypoint(self, start, goal, obstacle, all_obstacles):
        """
        Generira 4 kandidata oko bboxa i bira najkraci validni put.
        """
        candidates = [
            (obstacle["x_min"], obstacle["y_min"]),
            (obstacle["x_max"], obstacle["y_min"]),
            (obstacle["x_max"], obstacle["y_max"]),
            (obstacle["x_min"], obstacle["y_max"]),
        ]

        valid_candidates = []

        for candidate in candidates:
            valid = True

            for obs in all_obstacles:
                if self.point_inside_bbox(candidate, obs):
                    valid = False
                    break

                if self.segment_intersects_bbox(start, candidate, obs):
                    valid = False
                    break

                if self.segment_intersects_bbox(candidate, goal, obs):
                    valid = False
                    break

            if valid:
                length = self.distance(start, candidate) + \
                    self.distance(candidate, goal)
                valid_candidates.append((length, candidate))

        if not valid_candidates:
            return None

        valid_candidates.sort(key=lambda item: item[0])
        return valid_candidates[0][1]

    def segment_intersects_bbox(self, p1, p2, bbox):
        """
        Provjerava sijece li segment p1-p2 pravokutnik bbox.
        """
        if self.point_inside_bbox(p1, bbox) or self.point_inside_bbox(p2, bbox):
            return True

        x_min = bbox["x_min"]
        y_min = bbox["y_min"]
        x_max = bbox["x_max"]
        y_max = bbox["y_max"]

        corners = [
            (x_min, y_min),
            (x_max, y_min),
            (x_max, y_max),
            (x_min, y_max),
        ]

        edges = [
            (corners[0], corners[1]),
            (corners[1], corners[2]),
            (corners[2], corners[3]),
            (corners[3], corners[0]),
        ]

        for e1, e2 in edges:
            if self.segments_intersect(p1, p2, e1, e2):
                return True

        return False

    def point_inside_bbox(self, p, bbox):
        x, y = p
        return (
            bbox["x_min"] <= x <= bbox["x_max"]
            and bbox["y_min"] <= y <= bbox["y_max"]
        )

    def segments_intersect(self, p1, p2, q1, q2):
        """
        Standardna 2D provjera presjeka dva segmenta.
        """
        def orientation(a, b, c):
            value = (
                (b[1] - a[1]) * (c[0] - b[0])
                - (b[0] - a[0]) * (c[1] - b[1])
            )
            if abs(value) < 1e-9:
                return 0
            return 1 if value > 0 else 2

        def on_segment(a, b, c):
            return (
                min(a[0], c[0]) <= b[0] <= max(a[0], c[0])
                and min(a[1], c[1]) <= b[1] <= max(a[1], c[1])
            )

        o1 = orientation(p1, p2, q1)
        o2 = orientation(p1, p2, q2)
        o3 = orientation(q1, q2, p1)
        o4 = orientation(q1, q2, p2)

        if o1 != o2 and o3 != o4:
            return True

        if o1 == 0 and on_segment(p1, q1, p2):
            return True
        if o2 == 0 and on_segment(p1, q2, p2):
            return True
        if o3 == 0 and on_segment(q1, p1, q2):
            return True
        if o4 == 0 and on_segment(q1, p2, q2):
            return True

        return False

    def distance(self, p1, p2):
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
