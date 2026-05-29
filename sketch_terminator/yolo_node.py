#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import json
import torch
from ultralytics import YOLO

class YoloNode(Node):
    """
    Unified, self-contained ROS 2 node for YOLOv12 object detection.
    Subscribes to image topic, performs inference, publishes detections in JSON string format,
    and publishes annotated debug images for visualization.
    """
    def __init__(self):
        super().__init__('yolo_node')
        self.get_logger().info("Starting self-contained YOLOv12 Detection Node...")

        # Declare parameters
        self.declare_parameter("model", "yolo12m.pt")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("threshold", 0.5)
        self.declare_parameter("iou", 0.5)
        self.declare_parameter("input_image_topic", "/image_raw")
        self.declare_parameter("detections_topic", "detections")
        self.declare_parameter("debug_image_topic", "dbg_image")
        self.declare_parameter("use_debug", True)

        # Load parameters
        model_name = self.get_parameter("model").value
        device_param = self.get_parameter("device").value
        self.threshold = self.get_parameter("threshold").value
        self.iou = self.get_parameter("iou").value
        input_image_topic = self.get_parameter("input_image_topic").value
        detections_topic = self.get_parameter("detections_topic").value
        debug_image_topic = self.get_parameter("debug_image_topic").value
        self.use_debug = self.get_parameter("use_debug").value

        # Select CUDA if available and requested, otherwise fallback to CPU
        if "cuda" in device_param and not torch.cuda.is_available():
            self.get_logger().warn("CUDA was requested but is not available! Falling back to CPU.")
            self.device = "cpu"
        else:
            self.device = device_param

        self.get_logger().info(f"Loading YOLO model '{model_name}' on device '{self.device}'...")
        try:
            self.model = YOLO(model_name)
            self.model.to(self.device)
            self.get_logger().info("YOLOv12 model successfully loaded!")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise e

        self.bridge = CvBridge()

        # Publishers
        self.detections_pub = self.create_publisher(String, detections_topic, 10)
        self.debug_pub = self.create_publisher(Image, debug_image_topic, 10)

        # Subscription
        self.image_sub = self.create_subscription(
            Image,
            input_image_topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f"Subscribed to image topic: {input_image_topic}")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV BGR image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS Image: {e}")
            return

        # Perform YOLO inference
        results = self.model.predict(
            source=cv_image,
            verbose=False,
            stream=False,
            conf=self.threshold,
            iou=self.iou,
            device=self.device
        )
        
        if not results:
            return

        result = results[0].cpu()

        # Parse detections and prepare JSON payload
        detections_list = []
        
        # If we have debug visualization enabled, we'll draw on a copy of the image
        if self.use_debug:
            dbg_image = cv_image.copy()
        else:
            dbg_image = None

        if result.boxes:
            for idx, box in enumerate(result.boxes):
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                score = float(box.conf[0])
                
                # Get bounding box coordinates in xywh (center x, center y, width, height)
                xywh = box.xywh[0].tolist()
                cx, cy, w, h = xywh[0], xywh[1], xywh[2], xywh[3]

                detection_dict = {
                    "class_name": class_name,
                    "score": score,
                    "bbox": {
                        "center": {
                            "position": {
                                "x": cx,
                                "y": cy
                            }
                        },
                        "size": {
                            "x": w,
                            "y": h
                        }
                    }
                }
                detections_list.append(detection_dict)

                # Draw bounding box and label for debugging
                if self.use_debug and dbg_image is not None:
                    # Calculate corners
                    x1 = int(cx - w / 2)
                    y1 = int(cy - h / 2)
                    x2 = int(cx + w / 2)
                    y2 = int(cy + h / 2)
                    
                    # Generate distinct colors per class ID
                    color = (
                        int((class_id * 75) % 255),
                        int((class_id * 150) % 255),
                        int((class_id * 225) % 255)
                    )
                    
                    cv2.rectangle(dbg_image, (x1, y1), (x2, y2), color, 2)
                    label = f"{class_name} ({score:.2f})"
                    cv2.putText(dbg_image, label, (x1, max(y1 - 10, 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        # Publish detections JSON string
        payload = {"detections": detections_list}
        json_msg = String()
        json_msg.data = json.dumps(payload)
        self.detections_pub.publish(json_msg)

        # Publish debug image
        if self.use_debug and dbg_image is not None:
            try:
                dbg_msg = self.bridge.cv2_to_imgmsg(dbg_image, encoding="bgr8")
                dbg_msg.header = msg.header
                self.debug_pub.publish(dbg_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish debug image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
