#!/usr/bin/env python3
"""
Convert Create3 /wheel_ticks to /odom and odom->base_link TF.
Use when the robot does not publish /odom (e.g. publish_odom_tfs disabled).
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from irobot_create_msgs.msg import WheelTicks
import tf2_ros

# Create3 specs
TICKS_PER_REV = 508.8
WHEEL_RADIUS = 0.022  # m
WHEEL_BASE = 0.235  # m
DIST_PER_TICK = 2 * math.pi * WHEEL_RADIUS / TICKS_PER_REV


class WheelTicksToOdomNode(Node):
    def __init__(self):
        super().__init__('wheel_ticks_to_odom')
        self.declare_parameter('wheel_ticks_topic', '/wheel_ticks')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        wt_topic = self.get_parameter('wheel_ticks_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(WheelTicks, wt_topic, self._wheel_ticks_cb, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_ticks_left = None
        self.last_ticks_right = None
        self.last_stamp = None

        # Publish odom immediately so odom frame exists (avoids controller timeout)
        now = self.get_clock().now().to_msg()
        self._publish_odom(now, 0.0, 0.0)
        self.timer = self.create_timer(0.05, self._timer_cb)  # 20 Hz

        self.get_logger().info(f'wheel_ticks_to_odom: {wt_topic} -> {odom_topic}')

    def _wheel_ticks_cb(self, msg: WheelTicks):
        stamp = msg.header.stamp
        ticks_left = msg.ticks_left
        ticks_right = msg.ticks_right

        if self.last_ticks_left is None:
            self.last_ticks_left = ticks_left
            self.last_ticks_right = ticks_right
            self.last_stamp = stamp
            self._publish_odom(stamp, 0.0, 0.0)
            return

        d_left = (ticks_left - self.last_ticks_left) * DIST_PER_TICK
        d_right = (ticks_right - self.last_ticks_right) * DIST_PER_TICK
        dt = (stamp.sec - self.last_stamp.sec) + (stamp.nanosec - self.last_stamp.nanosec) * 1e-9

        self.last_ticks_left = ticks_left
        self.last_ticks_right = ticks_right
        self.last_stamp = stamp

        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / WHEEL_BASE
        if dt <= 0:
            dt = 0.016
        v = d_center / dt
        omega = d_theta / dt

        self.x += d_center * math.cos(self.theta + d_theta / 2)
        self.y += d_center * math.sin(self.theta + d_theta / 2)
        self.theta += d_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self._publish_odom(stamp, v, omega)

    def _timer_cb(self):
        """Periodically republish odom so TF stays fresh."""
        now = self.get_clock().now().to_msg()
        v = 0.0
        omega = 0.0
        if self.last_stamp is not None:
            pass  # v, omega from last update (we don't store them, use 0 for timer)
        self._publish_odom(now, v, omega)

    def _publish_odom(self, stamp, v: float, omega: float):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.theta / 2)
        q.w = math.cos(self.theta / 2)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = WheelTicksToOdomNode()
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
