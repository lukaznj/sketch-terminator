#!/usr/bin/env python3

import os
import json
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from yolo_msgs.msg import DetectionArray
from ament_index_python.packages import get_package_share_directory

from .kinematics import Kinematics
from .trajectory_generator import TrajectoryGenerator

class IntegrationNode(Node):
    def __init__(self):
        super().__init__('integration_node')

        self.get_logger().info("Initializing prarob_integration coordinator node...")

        # Declare parameters
        self.declare_parameter("detections_topic", "detections")
        self.declare_parameter("vision_positions_topic", "/vision/object_positions")
        self.declare_parameter("planning_path_topic", "/planning/path")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("joint_trajectory_topic", "/joint_trajectory_controller/joint_trajectory")
        
        # Load parameters
        detections_topic = self.get_parameter("detections_topic").value
        vision_positions_topic = self.get_parameter("vision_positions_topic").value
        planning_path_topic = self.get_parameter("planning_path_topic").value
        joint_states_topic = self.get_parameter("joint_states_topic").value
        joint_trajectory_topic = self.get_parameter("joint_trajectory_topic").value

        # Initialize Kinematics and Trajectory Generator
        self.kinematics = Kinematics()
        self.traj_gen = TrajectoryGenerator(v_max=1.0, a_max=2.0, dt_min=0.1, sample_dt=0.05)

        # Set default base offset parameters (in meters)
        self.flip_y = True
        self.flip_z = True
        self.offset_x = 0.173
        self.offset_y = 0.105
        self.offset_z = 0.0
        self.load_robot_offsets()

        # Load intrinsic calibration matrix from camera_calibration_params.yaml
        self.K_matrix = np.eye(3)
        self.D_coeff = np.zeros(5)
        self.load_camera_calibration()

        # Set default extrinsic matrix T_cam_world
        self.T_cam_world = np.array([
            [-0.99808, -0.01007, -0.06109,  0.06603],
            [ 0.01282, -0.99892, -0.04466,  0.06117],
            [-0.06058, -0.04536,  0.99713,  0.71017],
            [ 0.00000,  0.00000,  0.00000,  1.00000]
        ])
        self.load_extrinsic_matrix()

        # Storage for current joint states
        self.current_joints = None

        # Subscriptions
        self.yolo_sub = self.create_subscription(
            DetectionArray,
            detections_topic,
            self.yolo_callback,
            10
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self.joint_state_callback,
            10
        )
        self.path_sub = self.create_subscription(
            String,
            planning_path_topic,
            self.path_callback,
            10
        )

        # Publishers
        self.vision_pub = self.create_publisher(
            String,
            vision_positions_topic,
            10
        )
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            joint_trajectory_topic,
            10
        )

        self.get_logger().info("Coordinator node has successfully initialized and is listening to topics.")

    def load_camera_calibration(self):
        try:
            package_share = get_package_share_directory('sketch_terminator')
            calib_path = os.path.join(package_share, 'config', 'camera_calibration_params.yaml')
        except Exception:
            calib_path = ""

        if os.path.exists(calib_path):
            try:
                with open(calib_path, 'r') as f:
                    calib_data = yaml.safe_load(f)
                    k_data = calib_data['camera_matrix']['data']
                    self.K_matrix = np.array(k_data).reshape((3, 3))
                    d_data = calib_data['distortion_coefficients']['data']
                    self.D_coeff = np.array(d_data)
                    self.get_logger().info("Loaded intrinsic camera parameters successfully.")
            except Exception as e:
                self.get_logger().error(f"Error loading camera calibration: {e}")
        else:
            self.get_logger().warn(f"Calibration file {calib_path} not found. Using default camera intrinsics.")

    def load_extrinsic_matrix(self):
        try:
            package_share = get_package_share_directory('sketch_terminator')
            extrinsic_path = os.path.join(package_share, 'config', 'extrinsic_camera_params.yaml')
        except Exception:
            extrinsic_path = ""

        if os.path.exists(extrinsic_path):
            try:
                with open(extrinsic_path, 'r') as f:
                    extrinsic_data = yaml.safe_load(f)
                    matrix_data = extrinsic_data['extrinsic_matrix']['data']
                    self.T_cam_world = np.array(matrix_data).reshape((4, 4))
                    self.get_logger().info("Loaded extrinsic camera parameters successfully.")
            except Exception as e:
                self.get_logger().error(f"Error parsing extrinsic file: {e}. Using default values.")

    def load_robot_offsets(self):
        try:
            package_share = get_package_share_directory('sketch_terminator')
            offset_path = os.path.join(package_share, 'config', 'robot_offset_params.yaml')
        except Exception:
            offset_path = ""

        if os.path.exists(offset_path):
            try:
                with open(offset_path, 'r') as f:
                    offset_data = yaml.safe_load(f)
                    params = offset_data['robot_offset']
                    self.flip_y = bool(params.get('flip_y', True))
                    self.flip_z = bool(params.get('flip_z', True))
                    self.offset_x = float(params.get('offset_x', 0.173))
                    self.offset_y = float(params.get('offset_y', 0.105))
                    self.offset_z = float(params.get('offset_z', 0.0))
                    self.get_logger().info("Loaded robot base offset parameters successfully.")
            except Exception as e:
                self.get_logger().error(f"Error parsing base offset file: {e}. Using default values.")

    def image2world(self, imgpoint, z=0.0):
        """
        Projects a 2D image pixel coordinate [u, v] to 3D world coordinate [x, y, z] (in meters)
        assuming the point lies on a plane parallel to the world XY plane at height z.
        """
        R = self.T_cam_world[:3, :3]
        t = self.T_cam_world[:3, 3]

        R_inv = np.linalg.inv(R)
        intrinsic_inv = np.linalg.inv(self.K_matrix)

        P = self.K_matrix.dot(R)
        inverse_P = np.linalg.inv(P)
        unrotate_t = R_inv.dot(t)

        imgpoint_h = np.array([imgpoint[0], imgpoint[1], 1.0]).reshape((3, 1))

        # s * [(inverse_P * imgpoint_h)[2]] = z + unrotate_t[2]
        denom = (inverse_P.dot(imgpoint_h))[2, 0]
        if abs(denom) < 1e-6:
            return np.zeros((3, 1))

        s = (z + unrotate_t[2]) / denom
        world_point = R_inv.dot(intrinsic_inv.dot(imgpoint_h.dot(s)) - t.reshape((3, 1)))

        return world_point

    def joint_state_callback(self, msg):
        """Cache the current joint states."""
        try:
            idx1 = msg.name.index('joint1')
            idx2 = msg.name.index('joint2')
            idx3 = msg.name.index('joint3')
            self.current_joints = [
                msg.position[idx1],
                msg.position[idx2],
                msg.position[idx3]
            ]
        except ValueError:
            pass

    def yolo_callback(self, msg):
        """
        Callback for YOLO detections. Coordinates mapping 2D pixel bounding boxes to
        3D world space (in mm) and publishes in standard format on /vision/object_positions.
        """
        objects = []
        for det in msg.detections:
            center_x = det.bbox.center.position.x
            center_y = det.bbox.center.position.y
            size_x = det.bbox.size.x
            size_y = det.bbox.size.y

            # Compute pixel corners
            u_min = center_x - size_x / 2.0
            v_min = center_y - size_y / 2.0
            u_max = center_x + size_x / 2.0
            v_max = center_y + size_y / 2.0

            # Transform center to world base frame {R} (in meters)
            wp_center = self.image2world([center_x, center_y], z=0.0)
            
            # Transform corners
            wp_min = self.image2world([u_min, v_min], z=0.0)
            wp_max = self.image2world([u_max, v_max], z=0.0)

            # Transform coordinates and apply base offsets and flips
            xc = wp_center[0, 0]
            yc = wp_center[1, 0]
            zc = wp_center[2, 0]
            
            xc_r = xc + self.offset_x
            yc_r = (-yc if self.flip_y else yc) + self.offset_y
            zc_r = (-zc if self.flip_z else zc) + self.offset_z

            x_min = wp_min[0, 0]
            y_min = wp_min[1, 0]
            
            x_min_r = x_min + self.offset_x
            y_min_r = (-y_min if self.flip_y else y_min) + self.offset_y

            x_max = wp_max[0, 0]
            y_max = wp_max[1, 0]
            
            x_max_r = x_max + self.offset_x
            y_max_r = (-y_max if self.flip_y else y_max) + self.offset_y

            # Convert to mm as expected by the planner
            x_mm = float(xc_r * 1000.0)
            y_mm = float(yc_r * 1000.0)

            x_min_mm = float(x_min_r * 1000.0)
            y_min_mm = float(y_min_r * 1000.0)
            x_max_mm = float(x_max_r * 1000.0)
            y_max_mm = float(y_max_r * 1000.0)

            # Build object dict
            obj_dict = {
                "class": det.class_name,
                "score": float(det.score),
                "x": round(x_mm, 2),
                "y": round(y_mm, 2),
                "bbox": {
                    "x_min": round(min(x_min_mm, x_max_mm), 2),
                    "y_min": round(min(y_min_mm, y_max_mm), 2),
                    "x_max": round(max(x_min_mm, x_max_mm), 2),
                    "y_max": round(max(y_min_mm, y_max_mm), 2)
                }
            }
            objects.append(obj_dict)

        # Construct payload
        payload = {
            "frame": "robot_base",
            "units": "mm",
            "objects": objects
        }

        # Publish JSON string
        ros_msg = String()
        ros_msg.data = json.dumps(payload)
        self.vision_pub.publish(ros_msg)

    def path_callback(self, msg):
        """
        Receives planned path (in mm), converts to smooth timed joint trajectory,
        and commands the ros2_control joint_trajectory_controller.
        """
        try:
            path_data = json.loads(msg.data)
            path_pts = path_data.get("path", [])
        except Exception as e:
            self.get_logger().error(f"Failed to parse path JSON: {e}")
            return

        if not path_pts:
            self.get_logger().warn("Empty path received. No trajectory to generate.")
            return

        self.get_logger().info(f"Received path with {len(path_pts)} waypoints. Generating smooth trajectory...")

        # Scale waypoints to meters (planner outputs in mm)
        cartesian_path_meters = []
        for pt in path_pts:
            x_m = pt["x"] / 1000.0
            y_m = pt["y"] / 1000.0
            cartesian_path_meters.append([x_m, y_m, 0.0])  # Draw on Z=0

        # Generate trajectory starting from current cached joints
        trajectory = self.traj_gen.generate_trajectory(
            cartesian_path=cartesian_path_meters,
            current_joints=self.current_joints,
            default_z=0.0
        )

        if trajectory:
            self.get_logger().info(f"Successfully generated trajectory with {len(trajectory.points)} points. Publishing...")
            self.trajectory_pub.publish(trajectory)
        else:
            self.get_logger().error("Failed to generate timed trajectory.")

def main(args=None):
    rclpy.init(args=args)
    node = IntegrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
