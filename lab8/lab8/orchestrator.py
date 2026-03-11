#!/usr/bin/env python3
"""
Lab 8 - Orchestrator
State machine: UNDOCK -> EXPLORE -> RETURN -> NAV_TO_CUBE -> DOCK
Uses Explore action (Lab4 wall_follow + ArUco stop) instead of iRobot WallFollow.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import tf2_ros

from lab8_msgs.action import Explore
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from nav2_simple_commander.robot_navigator import TaskResult


class OrchestratorNode(Node):
    STATES = ['UNDOCK', 'EXPLORE', 'RETURN', 'NAV_TO_CUBE', 'DOCK', 'DONE']

    def __init__(self):
        super().__init__('orchestrator')
        self.navigator = TurtleBot4Navigator()
        self.explore_client = ActionClient(self, Explore, 'explore')

        self.declare_parameter('explore_max_duration_sec', 120.0)
        self.declare_parameter('approach_offset_m', 0.12)  # stop ~12cm from cube center
        self.declare_parameter('skip_undock', False)  # set true in sim if undock blocks
        self.declare_parameter('skip_explore', False)  # set true to skip explore, go straight to cube (for testing)
        self.declare_parameter('fallback_cube_pose', [0.8, 0.8, 0.0])  # [x,y,z] cube near start

        self.explore_max = self.get_parameter('explore_max_duration_sec').get_parameter_value().double_value
        self.approach_offset = self.get_parameter('approach_offset_m').get_parameter_value().double_value
        self.skip_undock = self.get_parameter('skip_undock').get_parameter_value().bool_value
        self.skip_explore = self.get_parameter('skip_explore').get_parameter_value().bool_value
        self.fallback_cube = self.get_parameter('fallback_cube_pose').get_parameter_value().double_array_value

        self.start_pose: PoseStamped | None = None
        self.aruco_pose: PoseStamped | None = None
        self.state = 'UNDOCK'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def run(self):
        while rclpy.ok() and self.state != 'DONE':
            if self.state == 'UNDOCK':
                self._do_undock()
            elif self.state == 'EXPLORE':
                self._do_explore()
            elif self.state == 'RETURN':
                self._do_return()
            elif self.state == 'NAV_TO_CUBE':
                self._do_nav_to_cube()
            elif self.state == 'DOCK':
                self._do_dock()
            rclpy.spin_once(self, timeout_sec=0.1)

    def _do_undock(self):
        self.get_logger().info('State: UNDOCK')
        if self.skip_undock:
            self.get_logger().info('Skipping undock (skip_undock=true)')
        else:
            self.navigator.undock()
        # Record start pose for return (retry: map frame may need a moment)
        t = None
        for attempt in range(5):
            try:
                t = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=3.0)
                )
                break
            except Exception as e:
                if attempt < 4:
                    self.get_logger().warn(f'Start pose lookup attempt {attempt + 1}/5: {e}, retrying...')
                    rclpy.spin_once(self, timeout_sec=1.0)
                else:
                    self.get_logger().error(f'Failed to get start pose: {e}')
        if t is not None:
            self.start_pose = PoseStamped()
            self.start_pose.header = t.header
            self.start_pose.pose.position.x = t.transform.translation.x
            self.start_pose.pose.position.y = t.transform.translation.y
            self.start_pose.pose.position.z = t.transform.translation.z
            self.start_pose.pose.orientation = t.transform.rotation
            self.get_logger().info(
                f'Start pose recorded: ({self.start_pose.pose.position.x:.2f}, '
                f'{self.start_pose.pose.position.y:.2f})'
            )
        else:
            self.start_pose = self.navigator.getPoseStamped([0.0, 0.0], 0.0)
        self.state = 'EXPLORE'

    def _do_explore(self):
        self.get_logger().info('State: EXPLORE (Lab4 wall_follow + ArUco search via Explore action)')
        self.aruco_pose = None

        if self.skip_explore:
            self.get_logger().info('Skipping explore (skip_explore=true) - using fallback cube pose')
            if len(self.fallback_cube) >= 2:
                self.aruco_pose = self.navigator.getPoseStamped(
                    [float(self.fallback_cube[0]), float(self.fallback_cube[1])], 0.0
                )
                self.aruco_pose.header.frame_id = 'map'
            self.state = 'NAV_TO_CUBE'  # skip RETURN (robot already at start)
            return

        self.explore_client.wait_for_server(timeout_sec=10.0)
        if not self.explore_client.server_is_ready():
            self.get_logger().error('Explore action server not ready - cannot proceed')
            self.state = 'RETURN'
            return

        goal_msg = Explore.Goal()
        goal_msg.max_duration_sec = float(self.explore_max)

        send_future = self.explore_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Explore goal rejected')
            self.state = 'RETURN'
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.success and result.aruco_pose.header.frame_id:
            self.aruco_pose = result.aruco_pose
            self.get_logger().info('ArUco cube found via Explore action!')
        else:
            self.get_logger().warn('Explore finished without ArUco - using fallback cube pose')
            if len(self.fallback_cube) >= 2:
                self.aruco_pose = self.navigator.getPoseStamped(
                    [float(self.fallback_cube[0]), float(self.fallback_cube[1])], 0.0
                )
                self.aruco_pose.header.frame_id = 'map'

        self.state = 'RETURN'

    def _do_return(self):
        self.get_logger().info('State: RETURN to start')
        if self.start_pose is None:
            self.get_logger().error('No start pose - skipping return')
            self.state = 'NAV_TO_CUBE'
            return
        self.start_pose.header.stamp = self.get_clock().now().to_msg()
        self.navigator.goToPose(self.start_pose)
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Returned to start')
        else:
            self.get_logger().warn(f'Return result: {result}')
        self.state = 'NAV_TO_CUBE'

    def _do_nav_to_cube(self):
        self.get_logger().info('State: NAV_TO_CUBE')
        if self.aruco_pose is None:
            self.get_logger().warn('No ArUco pose - skipping nav to cube')
            self.state = 'DOCK'
            return

        # Goal: approach to within ~10cm. Offset goal from cube center toward start.
        goal = PoseStamped()
        goal.header = self.aruco_pose.header
        goal.header.stamp = self.get_clock().now().to_msg()
        cx = self.aruco_pose.pose.position.x
        cy = self.aruco_pose.pose.position.y
        cz = self.aruco_pose.pose.position.z
        if self.start_pose is not None:
            sx = self.start_pose.pose.position.x
            sy = self.start_pose.pose.position.y
            dx = sx - cx
            dy = sy - cy
            dist = math.hypot(dx, dy)
            if dist > 0.01:
                dx /= dist
                dy /= dist
                goal.pose.position.x = cx + self.approach_offset * dx
                goal.pose.position.y = cy + self.approach_offset * dy
            else:
                goal.pose.position.x = cx - self.approach_offset
                goal.pose.position.y = cy
        else:
            goal.pose.position.x = cx - self.approach_offset
            goal.pose.position.y = cy
        goal.pose.position.z = cz
        goal.pose.orientation = self.aruco_pose.pose.orientation

        self.navigator.goToPose(goal)
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Reached ArUco cube')
        else:
            self.get_logger().warn(f'Nav to cube result: {result}')
        self.state = 'DOCK'

    def _do_dock(self):
        self.get_logger().info('State: DOCK')
        self.navigator.dock()
        self.get_logger().info('Lab 8 sequence complete!')
        self.state = 'DONE'


def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # launch 可能已关闭 context


if __name__ == '__main__':
    main()
