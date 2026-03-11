#!/usr/bin/env python3
"""
Publish an initial empty map so costmap doesn't timeout waiting for /map.
slam_toolbox will overwrite when it publishes.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose


class EmptyMapPublisher(Node):
    def __init__(self):
        super().__init__('empty_map_publisher')
        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.timer = self.create_timer(1.0, self._publish)
        self.get_logger().info('Publishing initial empty map to /map')
        self._publish()

    def _publish(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = 0.05
        msg.info.width = 100
        msg.info.height = 100
        msg.info.origin = Pose()
        msg.info.origin.position.x = -2.5
        msg.info.origin.position.y = -2.5
        msg.info.origin.position.z = 0.0
        msg.data = [-1] * (100 * 100)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EmptyMapPublisher()
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
