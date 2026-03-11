#!/usr/bin/env python3
"""
Lab 8 - ArUco Detector
Subscribes to RGB image, detects ArUco markers, estimates pose in camera frame,
transforms to map frame, and publishes PoseStamped when detected.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose
import cv2
import cv_bridge
import tf2_ros
import tf2_geometry_msgs  # noqa: F401

SENSOR_QOS = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
)


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.bridge = cv_bridge.CvBridge()
        self.camera_info: CameraInfo | None = None

        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('camera_info_topic', '/oakd/rgb/preview/camera_info')
        self.declare_parameter('output_topic', '/aruco_pose')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('marker_size', 0.1)  # meters, ArUco cube face
        self.declare_parameter('aruco_dict', 'DICT_4X4_100')

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        dict_name = self.get_parameter('aruco_dict').get_parameter_value().string_value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ArUco dictionary
        aruco_dict_map = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
            'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
        }
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            aruco_dict_map.get(dict_name, cv2.aruco.DICT_4X4_100)
        )
        self.detector = None
        self.detector_params = None
        try:
            self.detector_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)
        except AttributeError:
            try:
                self.detector_params = cv2.aruco.DetectorParameters_create()
            except AttributeError:
                self.detector_params = None

        self.pose_pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.create_subscription(Image, self.image_topic, self._image_cb, SENSOR_QOS)
        self.create_subscription(CameraInfo, self.camera_info_topic, self._info_cb, SENSOR_QOS)

        self._last_pose: PoseStamped | None = None

        self.get_logger().info(
            f'ArucoDetector started: image={self.image_topic}, output={self.output_topic}, '
            f'target_frame={self.target_frame}, marker_size={self.marker_size}m'
        )

    def _info_cb(self, msg: CameraInfo):
        self.camera_info = msg

    def _image_cb(self, msg: Image):
        if self.camera_info is None:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'CvBridge error: {e}', throttle_duration_sec=5.0)
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cam_matrix = np.array(self.camera_info.k).reshape(3, 3)
        dist_coeffs = np.array(self.camera_info.d)

        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            kwargs = {} if self.detector_params is None else {'parameters': self.detector_params}
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, **kwargs)

        if ids is None or len(ids) == 0:
            return

        # Object points for marker (center at origin, marker in XY plane)
        half = self.marker_size / 2.0
        obj_pts = np.array([
            [-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]
        ], dtype=np.float32)
        # Estimate pose using solvePnP
        success, rvec, tvec = cv2.solvePnP(
            obj_pts, corners[0], cam_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not success:
            return
        t = tvec.flatten()

        pose_cam = PoseStamped()
        pose_cam.header = msg.header
        pose_cam.pose.position.x = float(t[0])
        pose_cam.pose.position.y = float(t[1])
        pose_cam.pose.position.z = float(t[2])
        # Convert rvec to quaternion (simplified: identity for cube center)
        pose_cam.pose.orientation.w = 1.0

        try:
            pose_map = self.tf_buffer.transform(pose_cam, self.target_frame)
        except Exception as e:
            self.get_logger().warn(
                f'TF transform to {self.target_frame} failed: {e}',
                throttle_duration_sec=2.0,
            )
            return

        pose_map.header.stamp = self.get_clock().now().to_msg()
        self._last_pose = pose_map
        self.pose_pub.publish(pose_map)
        self.get_logger().info(
            f'ArUco detected at map: ({pose_map.pose.position.x:.3f}, '
            f'{pose_map.pose.position.y:.3f}, {pose_map.pose.position.z:.3f})'
        )

    def get_last_pose(self) -> PoseStamped | None:
        return self._last_pose


def main(args=None):
    import sys
    import traceback
    rclpy.init(args=args)
    try:
        node = ArucoDetectorNode()
    except Exception as e:
        print(f'[aruco_detector] 启动失败: {e}', file=sys.stderr)
        traceback.print_exc(file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
