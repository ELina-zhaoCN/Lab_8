#!/usr/bin/env python3
"""
Test node: publish Twist to /cmd_vel for a few seconds at startup.
Use to verify the cmd_vel -> Ignition bridge works.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist


def main():
    rclpy.init()
    node = Node('test_cmd_vel')
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    pub = node.create_publisher(Twist, '/cmd_vel', qos)
    node.get_logger().info('Test: publishing Twist to /cmd_vel for 5 seconds (forward 0.15 m/s)')
    import time
    time.sleep(1.0)  # brief wait for bridge
    start = time.monotonic()
    while rclpy.ok() and (time.monotonic() - start) < 5.0:
        t = Twist()
        t.linear.x = 0.15
        t.angular.z = 0.0
        pub.publish(t)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.1)  # 10 Hz
    # Stop
    t = Twist()
    t.linear.x = 0.0
    t.angular.z = 0.0
    pub.publish(t)
    node.get_logger().info('Test done - if robot moved, bridge works')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
