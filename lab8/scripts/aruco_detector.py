#!/usr/bin/env python3
"""ArUco detector: subscribes to camera, publishes /aruco_pose in map frame."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped
import cv2
import numpy as np


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.declare_parameter('marker_size_m', 0.10)
        self.declare_parameter('aruco_dict', 'DICT_4X4_1000')
        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('camera_info_topic', '/oakd/rgb/preview/camera_info')
        self.declare_parameter('min_detections', 3)

        self.marker_size = self.get_parameter('marker_size_m').value
        dict_name = self.get_parameter('aruco_dict').value
        self.min_detections = self.get_parameter('min_detections').value
        img_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.dict_map = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
            'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
        }
        aruco_dict = cv2.aruco.getPredefinedDictionary(self.dict_map.get(dict_name, cv2.aruco.DICT_4X4_1000))
        try:
            self.detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
            self._use_new_api = True
        except AttributeError:
            self._aruco_dict_obj = aruco_dict
            self._det_params = cv2.aruco.DetectorParameters_create()
            self.detector = None
            self._use_new_api = False

        self.camera_info = None
        self.detection_count = 0
        self.last_pose = None

        self.create_subscription(Image, img_topic, self._img_cb, 10)
        self.create_subscription(CameraInfo, info_topic, self._info_cb, 10)
        self._pose_pub = self.create_publisher(PoseStamped, '/aruco_pose', 10)

    def _info_cb(self, msg):
        self.camera_info = msg

    def _img_cb(self, msg):
        if self.camera_info is None:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'CvBridge error: {e}')
            return

        K = np.array(self.camera_info.k).reshape(3, 3)
        D = np.array(self.camera_info.d) if self.camera_info.d else None

        if self._use_new_api:
            corners, ids, _ = self.detector.detectMarkers(cv_image)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self._aruco_dict_obj, parameters=self._det_params)
        if ids is None or len(ids) == 0:
            self.detection_count = 0
            self.last_pose = None
            return

        obj_pts = np.array([
            [-self.marker_size/2, self.marker_size/2, 0],
            [self.marker_size/2, self.marker_size/2, 0],
            [self.marker_size/2, -self.marker_size/2, 0],
            [-self.marker_size/2, -self.marker_size/2, 0],
        ], dtype=np.float32)

        for i, corner in enumerate(corners):
            ret, rvec, tvec = cv2.solvePnP(
                obj_pts, corner.astype(np.float32), K, D,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not ret:
                continue
            R, _ = cv2.Rodrigues(rvec)
            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = self.camera_info.header.frame_id
            pose.pose.position.x = float(tvec[0])
            pose.pose.position.y = float(tvec[1])
            pose.pose.position.z = float(tvec[2])
            # rotation matrix to quaternion
            trace = R[0,0] + R[1,1] + R[2,2]
            if trace > 0:
                s = 0.5 / np.sqrt(trace + 1.0)
                pose.pose.orientation.w = 0.25 / s
                pose.pose.orientation.x = (R[2,1] - R[1,2]) * s
                pose.pose.orientation.y = (R[0,2] - R[2,0]) * s
                pose.pose.orientation.z = (R[1,0] - R[0,1]) * s
            else:
                if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
                    s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
                    pose.pose.orientation.w = (R[2,1] - R[1,2]) / s
                    pose.pose.orientation.x = 0.25 * s
                    pose.pose.orientation.y = (R[0,1] + R[1,0]) / s
                    pose.pose.orientation.z = (R[0,2] + R[2,0]) / s
                elif R[1,1] > R[2,2]:
                    s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
                    pose.pose.orientation.w = (R[0,2] - R[2,0]) / s
                    pose.pose.orientation.x = (R[0,1] + R[1,0]) / s
                    pose.pose.orientation.y = 0.25 * s
                    pose.pose.orientation.z = (R[1,2] + R[2,1]) / s
                else:
                    s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
                    pose.pose.orientation.w = (R[1,0] - R[0,1]) / s
                    pose.pose.orientation.x = (R[0,2] + R[2,0]) / s
                    pose.pose.orientation.y = (R[1,2] + R[2,1]) / s
                    pose.pose.orientation.z = 0.25 * s

            try:
                trans = self.tf_buffer.lookup_transform(
                    'map', pose.header.frame_id, msg.header.stamp, rclpy.duration.Duration(seconds=0.5))
                pose_map = do_transform_pose_stamped(pose, trans)
                pose_map.header.frame_id = 'map'
                self.detection_count += 1
                self.last_pose = pose_map
                if self.detection_count >= self.min_detections:
                    self._pose_pub.publish(pose_map)
            except Exception as e:
                self.detection_count = 0


def main():
    rclpy.init()
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
