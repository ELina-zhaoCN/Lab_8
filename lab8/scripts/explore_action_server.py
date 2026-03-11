#!/usr/bin/env python3
"""Explore action: wall-follow + ArUco detection. Publishes to /cmd_vel."""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from lab8_msgs.action import Explore

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class ExploreActionServer(Node):
    def __init__(self):
        super().__init__('explore_action_server')
        self._action_server = ActionServer(
            self, Explore, 'explore',
            execute_callback=self._execute_callback)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self._scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self._scan = None
        self._aruco_pose = None
        self._aruco_sub = self.create_subscription(PoseStamped, '/aruco_pose', self._aruco_cb, 10)

    def _scan_cb(self, msg):
        self._scan = msg

    def _aruco_cb(self, msg):
        self._aruco_pose = msg

    def _execute_callback(self, goal_handle):
        max_dur = goal_handle.request.max_duration_sec
        start = self.get_clock().now()
        result = Explore.Result()
        result.aruco_found = False
        result.aruco_pose = PoseStamped()

        while rclpy.ok():
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed >= max_dur:
                break
            if self._aruco_pose is not None:
                result.aruco_found = True
                result.aruco_pose = self._aruco_pose
                goal_handle.succeed()
                return result

            # Simple wall-follow: keep distance from right wall
            twist = Twist()
            if self._scan is not None and len(self._scan.ranges) > 0:
                n = len(self._scan.ranges)
                front = min(self._scan.ranges[n//4:3*n//4]) if n > 0 else 1.0
                right = self._scan.ranges[n//4] if n > n//4 else 1.0
                left = self._scan.ranges[3*n//4] if n > 3*n//4 else 1.0
                wall_dist = 0.4
                if front < 0.5:
                    twist.angular.z = 0.5
                    twist.linear.x = 0.05
                elif right < wall_dist * 0.7:
                    twist.linear.x = 0.15
                    twist.angular.z = -0.3
                elif right > wall_dist * 1.3:
                    twist.linear.x = 0.15
                    twist.angular.z = 0.3
                else:
                    twist.linear.x = 0.2
                    twist.angular.z = 0.0
            else:
                twist.linear.x = 0.15
            self._cmd_pub.publish(twist)
            goal_handle.publish_feedback(Explore.Feedback(elapsed_sec=float(elapsed)))
            rclpy.spin_once(self, timeout_sec=0.05)

        self._cmd_pub.publish(Twist())
        goal_handle.succeed()
        return result


def main():
    rclpy.init()
    node = ExploreActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
