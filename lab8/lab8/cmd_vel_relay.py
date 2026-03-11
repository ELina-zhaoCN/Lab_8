#!/usr/bin/env python3
"""
Relay /cmd_vel -> /model/turtlebot4/cmd_vel for Ignition sim (NAV_TO_CUBE phase).
Explore publishes directly to both; relay forwards Nav2's Twist to model topic.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist


class CmdVelRelayNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Subscribe to Nav2's /cmd_vel output and forward directly to /diffdrive_controller/cmd_vel_unstamped
        # which cmd_vel_bridge (Create3) bridges to Ignition /cmd_vel -> DiffDrive plugin -> robot moves.
        self._sub = self.create_subscription(Twist, '/cmd_vel', self._cb, sub_qos)
        self._pub = self.create_publisher(Twist, '/diffdrive_controller/cmd_vel_unstamped', pub_qos)
        self.get_logger().info('Relay: /cmd_vel -> /diffdrive_controller/cmd_vel_unstamped (RELIABLE)')

    def _cb(self, msg: Twist):
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = CmdVelRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
