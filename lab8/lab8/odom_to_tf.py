#!/usr/bin/env python3
"""
odom_to_tf - 将 /odom 话题的位姿发布到 TF 树
Create 3 发布 /odom 但不发布 odom->base_link TF，Nav2 需要此 TF。
运行: ros2 run lab8 odom_to_tf
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('fallback_after_sec', 5.0)
        odom_topic = self.get_parameter('odom_topic').value
        rate_hz = self.get_parameter('publish_rate').value
        self._fallback_sec = self.get_parameter('fallback_after_sec').value
        self._last_tf = None
        self._msg_count = 0
        self._fallback_logged = False
        self._start_time = self.get_clock().now()
        # Create 3 的 /odom 使用 BEST_EFFORT，必须用 sensor_data QoS
        self.sub = self.create_subscription(
            Odometry, odom_topic, self._odom_cb, qos_profile_sensor_data
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._timer = self.create_timer(1.0 / rate_hz, self._timer_cb)
        self.get_logger().info(f'Publishing odom->base_link from {odom_topic}')

    def _odom_cb(self, msg: Odometry):
        self._msg_count += 1
        if self._msg_count == 1:
            self.get_logger().info('Received first /odom, publishing TF')
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._last_tf = t

    def _timer_cb(self):
        now = self.get_clock().now()
        if self._last_tf is not None:
            self._last_tf.header.stamp = now.to_msg()
            self.tf_broadcaster.sendTransform(self._last_tf)
        elif (now - self._start_time).nanoseconds / 1e9 >= self._fallback_sec:
            # 未收到 /odom 时发布静态 identity，让 Nav2 能启动
            if not self._fallback_logged:
                self._fallback_logged = True
                self.get_logger().warn(
                    f'No /odom after {self._fallback_sec}s, publishing static identity (odom=base_link)'
                )
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
