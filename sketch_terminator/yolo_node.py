#!/usr/bin/env python3
"""Runs YOLO object detection on the camera stream.

Subscribes to raw camera images, runs inference, and publishes typed
detections plus an annotated debug image. The model is whatever Ultralytics
weights file the ``model`` parameter names -- the default ``yolo12m.pt`` is
downloaded on first use and is not checked into the repository.

Topics
------
in   ``/image_raw``     camera frames
out  ``detections``     DetectionArray, remapped to ``/yolo/detections``
out  ``dbg_image``      the same frame with boxes and labels drawn on
"""

import cv2
import rclpy
import torch
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO
from yolo_msgs.msg import Detection, DetectionArray


def color_for_class(class_id):
    """Deterministic, reasonably distinct BGR colour per class id."""
    return (
        int((class_id * 75) % 255),
        int((class_id * 150) % 255),
        int((class_id * 225) % 255),
    )


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.declare_parameter('model', 'yolo12m.pt')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('threshold', 0.5)    # minimum confidence
        self.declare_parameter('iou', 0.5)          # non-max-suppression overlap
        self.declare_parameter('input_image_topic', '/image_raw')
        self.declare_parameter('detections_topic', 'detections')
        self.declare_parameter('debug_image_topic', 'dbg_image')
        self.declare_parameter('use_debug', True)

        model_name = self.get_parameter('model').value
        device_param = self.get_parameter('device').value
        self.threshold = self.get_parameter('threshold').value
        self.iou = self.get_parameter('iou').value
        self.use_debug = self.get_parameter('use_debug').value

        # Fall back to CPU rather than dying, so the vision pipeline still runs
        # (slowly) on a machine without a working CUDA install.
        if 'cuda' in device_param and not torch.cuda.is_available():
            self.get_logger().warn('CUDA requested but unavailable; falling back to CPU.')
            self.device = 'cpu'
        else:
            self.device = device_param

        self.get_logger().info(f"Loading YOLO model '{model_name}' on '{self.device}'...")
        self.model = YOLO(model_name)
        self.model.to(self.device)
        self.get_logger().info('YOLO model loaded.')

        self.bridge = CvBridge()
        self.detections_pub = self.create_publisher(
            DetectionArray, self.get_parameter('detections_topic').value, 10
        )
        self.debug_pub = self.create_publisher(
            Image, self.get_parameter('debug_image_topic').value, 10
        )

        input_image_topic = self.get_parameter('input_image_topic').value
        self.create_subscription(Image, input_image_topic, self.image_callback, 10)
        self.get_logger().info(f'Subscribed to {input_image_topic}.')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Failed to convert the incoming image: {exc}')
            return

        results = self.model.predict(
            source=cv_image,
            verbose=False,
            stream=False,
            conf=self.threshold,
            iou=self.iou,
            device=self.device,
        )
        if not results:
            return

        # Pull the result back to host memory before touching any of its fields.
        result = results[0].cpu()

        detections_msg = DetectionArray()
        detections_msg.header = msg.header
        dbg_image = cv_image.copy() if self.use_debug else None

        for box in result.boxes or []:
            class_id = int(box.cls[0])
            class_name = self.model.names[class_id]
            score = float(box.conf[0])
            # Ultralytics gives (centre x, centre y, width, height) in pixels.
            cx, cy, w, h = box.xywh[0].tolist()

            detection = Detection()
            detection.class_id = class_id
            detection.class_name = class_name
            detection.score = score
            detection.bbox.center.position.x = cx
            detection.bbox.center.position.y = cy
            detection.bbox.size.x = w
            detection.bbox.size.y = h
            detections_msg.detections.append(detection)

            if dbg_image is not None:
                self.draw_detection(dbg_image, class_id, class_name, score, cx, cy, w, h)

        self.detections_pub.publish(detections_msg)

        if dbg_image is not None:
            try:
                dbg_msg = self.bridge.cv2_to_imgmsg(dbg_image, encoding='bgr8')
                dbg_msg.header = msg.header
                self.debug_pub.publish(dbg_msg)
            except Exception as exc:
                self.get_logger().error(f'Failed to publish the debug image: {exc}')

    @staticmethod
    def draw_detection(image, class_id, class_name, score, cx, cy, w, h):
        x1, y1 = int(cx - w / 2), int(cy - h / 2)
        x2, y2 = int(cx + w / 2), int(cy + h / 2)
        color = color_for_class(class_id)

        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        # Keep the label on screen when the box is flush against the top edge.
        cv2.putText(image, f'{class_name} ({score:.2f})', (x1, max(y1 - 10, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
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
