#!/usr/bin/env python3
"""Orchestrator: UNDOCK -> EXPLORE -> RETURN -> NAV_TO_CUBE -> DOCK -> DONE"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from lab8_msgs.action import Explore
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from nav2_simple_commander.robot_navigator import TaskResult
import math


class OrchestratorNode(Node):
    def __init__(self):
        super().__init__('orchestrator')
        self.declare_parameter('skip_undock', False)
        val = self.get_parameter('skip_undock').value
        self.skip_undock = val if isinstance(val, bool) else str(val).lower() == 'true'
        self.nav = TurtleBot4Navigator()
        self.explore_client = ActionClient(self, Explore, 'explore')
        self.start_pose = None
        self.aruco_pose = None
        self.approach_offset_m = 0.12

    def run(self):
        self.get_logger().info('Lab 8 Orchestrator starting')

        # UNDOCK
        if not self.skip_undock:
            self.get_logger().info('Undocking...')
            self.nav.undock()
            rclpy.spin_once(self, timeout_sec=0.5)
        else:
            self.get_logger().info('Skipping undock (skip_undock=true)')

        # Record start pose (for RETURN)
        try:
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            import time
            time.sleep(2.0)
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), rclpy.duration.Duration(seconds=5.0))
            self.start_pose = PoseStamped()
            self.start_pose.header.frame_id = 'map'
            self.start_pose.header.stamp = self.get_clock().now().to_msg()
            self.start_pose.pose.position.x = trans.transform.translation.x
            self.start_pose.pose.position.y = trans.transform.translation.y
            self.start_pose.pose.position.z = trans.transform.translation.z
            self.start_pose.pose.orientation = trans.transform.rotation
            self.get_logger().info(f'Start pose: ({self.start_pose.pose.position.x:.2f}, {self.start_pose.pose.position.y:.2f})')
        except Exception as e:
            self.get_logger().warn(f'Could not get start pose: {e}, using origin')
            self.start_pose = self.nav.getPoseStamped([0.0, 0.0], 0.0)

        # EXPLORE
        self.get_logger().info('Exploring...')
        self.explore_client.wait_for_server()
        goal = Explore.Goal()
        goal.max_duration_sec = 120.0
        future = self.explore_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('Explore goal rejected')
            return
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.aruco_pose = result.aruco_pose if result.aruco_found else None
        self.get_logger().info(f'Explore done. ArUco found: {result.aruco_found}')

        # RETURN to start
        self.get_logger().info('Returning to start...')
        self.nav.goToPose(self.start_pose)
        while not self.nav.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Return complete')

        # NAV_TO_CUBE (if ArUco found)
        if self.aruco_pose is not None and result.aruco_found:
            self.get_logger().info('Navigating to ArUco cube...')
            goal_pose = self._compute_approach_pose()
            self.nav.goToPose(goal_pose)
            while not self.nav.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info('Nav to cube complete')

        # DOCK
        self.get_logger().info('Docking...')
        self.nav.dock()
        self.get_logger().info('Lab 8 DONE')

    def _compute_approach_pose(self):
        """Goal ~10-12cm from cube, along direction cube->start."""
        cx = self.aruco_pose.pose.position.x
        cy = self.aruco_pose.pose.position.y
        sx = self.start_pose.pose.position.x
        sy = self.start_pose.pose.position.y
        dx = sx - cx
        dy = sy - cy
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 0.01:
            dx, dy = 1.0, 0.0
            dist = 1.0
        ux = dx / dist
        uy = dy / dist
        gx = cx + self.approach_offset_m * ux
        gy = cy + self.approach_offset_m * uy
        yaw = math.degrees(math.atan2(-uy, -ux))
        return self.nav.getPoseStamped([gx, gy], yaw)


def main():
    rclpy.init()
    node = OrchestratorNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
