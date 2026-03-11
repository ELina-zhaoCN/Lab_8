#!/usr/bin/env python3
"""
Lab 8 - Explore Action Server
Uses Lab 4 wall_follow logic (PID + cmd_vel) to explore the maze while SLAM builds the map.
Stops when ArUco cube is detected or max_duration is reached.
"""

import numpy as np
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from lab8_msgs.action import Explore


class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, setpoint: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self._prev_error = 0.0
        self._integral = 0.0

    def __call__(self, system_measurement: float, dt: float) -> float:
        if dt <= 0:
            return 0.0
        error = self.setpoint - system_measurement
        p_out = self.kp * error
        self._integral += error * dt
        i_out = self.ki * self._integral
        derivative = (error - self._prev_error) / dt
        d_out = self.kd * derivative
        self._prev_error = error
        return p_out + i_out + d_out


class ExploreActionServer(Node):
    """Explore action: wall-follow (Lab4 logic) until ArUco detected or timeout."""

    def __init__(self):
        super().__init__('explore_action_server')

        self.declare_parameter('aruco_pose_topic', '/aruco_pose')
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('setpoint', 0.5)
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('obstacle_speed', 0.1)
        # 新增：三层检测可配置参数
        self.declare_parameter('safe_distance', 0.50)       # 层2: 安全距离 (m)
        self.declare_parameter('min_avoid_distance', 0.28)  # 层2: 最小避障距离，触发后退 (m)
        self.declare_parameter('front_angle_deg', 15.0)     # 层2: 前方检测半角 (度)
        self.declare_parameter('wall_confirm_sec', 0.20)    # 层3: 墙体确认时长 (s)

        self.aruco_pose_topic = self.get_parameter('aruco_pose_topic').get_parameter_value().string_value
        kp = self.get_parameter('kp').get_parameter_value().double_value
        ki = self.get_parameter('ki').get_parameter_value().double_value
        kd = self.get_parameter('kd').get_parameter_value().double_value
        setpoint = self.get_parameter('setpoint').get_parameter_value().double_value
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.obstacle_speed = self.get_parameter('obstacle_speed').get_parameter_value().double_value
        self.safe_distance = self.get_parameter('safe_distance').get_parameter_value().double_value
        self.min_avoid_distance = self.get_parameter('min_avoid_distance').get_parameter_value().double_value
        self.front_angle_rad = self.get_parameter('front_angle_deg').get_parameter_value().double_value * 3.14159 / 180.0
        self.wall_confirm_sec = self.get_parameter('wall_confirm_sec').get_parameter_value().double_value

        self.pid = PIDController(kp=kp, ki=ki, kd=kd, setpoint=setpoint)
        self.last_scan: LaserScan | None = None
        self.aruco_pose: PoseStamped | None = None
        self._running = False
        self._goal_handle = None
        self._last_scan_warn_time = 0.0
        # 层3: 墙体连续存在计时
        self._wall_detected_since: float | None = None
        # 避障方向记忆（确定后不再切换，避免摆动）
        self._avoid_turn_right: bool | None = None
        self._avoid_active = False  # 是否处于避障模式

        cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self, Explore, 'explore',
            execute_callback=self._execute_cb,
            callback_group=cb_group,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Publish to both: /cmd_vel (direct) and /cmd_vel_smoothed (Nav2 pipeline)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', best_effort_qos)
        self.drive_pub = self.create_publisher(Twist, '/cmd_vel_smoothed', best_effort_qos)
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, scan_qos)
        self.aruco_sub = self.create_subscription(
            PoseStamped, self.aruco_pose_topic, self._aruco_cb, 10
        )

        self.get_logger().info('Explore action server ready (Lab4 wall_follow + ArUco stop)')

    def _scan_cb(self, msg: LaserScan):
        self.last_scan = msg

    def _aruco_cb(self, msg: PoseStamped):
        self.aruco_pose = msg

    def _stop_robot(self):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.cmd_pub.publish(t)
        self.drive_pub.publish(t)

    def _compute_wall_follow_twist(self) -> Twist | None:
        """
        三层检测避障 + 右墙跟随 PID 前进。
        层1: /scan 数据有效性
        层2: 正前方墙体检测（front_angle_deg±, safe_distance）
        层3: 墙体连续存在时长（wall_confirm_sec）
        """
        now = time.monotonic()

        # ── 层1: /scan 有效性检测 ──────────────────────────────────────────
        if self.last_scan is None:
            return None
        msg = self.last_scan
        scan_ranges = np.array(msg.ranges, dtype=np.float32)
        valid_mask = np.isfinite(scan_ranges) & (scan_ranges > 0.0)
        if valid_mask.sum() < 5:
            if now - self._last_scan_warn_time >= 3.0:
                self.get_logger().warn('层1: /scan 有效数据不足，缓慢前进')
                self._last_scan_warn_time = now
            t = Twist()
            t.linear.x = 0.08
            t.angular.z = 0.0
            return t

        # 无效值视为近距离障碍（贴墙时 LiDAR 返回 0/inf）
        scan_ranges[~valid_mask] = 0.15
        scan_ranges = np.clip(scan_ranges, 0.01, msg.range_max)
        n = len(scan_ranges)
        amin = msg.angle_min
        ainc = msg.angle_increment

        def idx(angle: float) -> int:
            i = int(round((angle - amin) / ainc))
            return max(0, min(i, n - 1))

        # ── 层2: 正前方墙体检测（可配置角度和安全距离） ──────────────────
        half = self.front_angle_rad           # 默认 ±15°
        front_samples = np.linspace(-half, half, max(5, int(half / ainc)))
        front_idxs = [idx(a) for a in front_samples]
        dist_front_narrow = float(np.min(scan_ranges[front_idxs]))

        # 同时采集较宽前方区域（±60°）用于侧方感知
        wide_idxs = [idx(a) for a in np.linspace(-1.05, 1.05, 50)]
        dist_front_wide = float(np.min(scan_ranges[wide_idxs]))
        dist_front = min(dist_front_narrow, dist_front_wide)

        dist_right = float(scan_ranges[idx(-1.5708)])
        dist_left  = float(scan_ranges[idx( 1.5708)])

        wall_ahead = dist_front < self.safe_distance  # 层2 通过条件

        # ── 层3: 墙体连续存在时长确认（过滤噪点） ───────────────────────
        if wall_ahead:
            if self._wall_detected_since is None:
                self._wall_detected_since = now
            wall_confirmed = (now - self._wall_detected_since) >= self.wall_confirm_sec
        else:
            self._wall_detected_since = None
            wall_confirmed = False

        # ── 三层全部通过 → 触发避障 ──────────────────────────────────────
        if wall_confirmed:
            # 进入避障模式：选向（仅首次选方向，之后固定不切换）
            if not self._avoid_active:
                self._avoid_active = True
                # 修改：避障优先级 - 右侧 ±30° 空间足够则优先右绕，否则左绕
                right30_idxs = [idx(a) for a in np.linspace(-0.52, 0.0, 10)]
                dist_right30 = float(np.min(scan_ranges[right30_idxs]))
                if dist_right30 > self.safe_distance * 1.2:
                    self._avoid_turn_right = True   # 右侧空旷 → 右绕
                else:
                    self._avoid_turn_right = False  # 右侧被挡 → 左绕

            tr = self._avoid_turn_right
            twist = Twist()
            if dist_front < self.min_avoid_distance:
                twist.linear.x = -0.10  # 太近先后退
            else:
                twist.linear.x = 0.0    # 原地转
            twist.angular.z = -1.4 if tr else 1.4
            return twist

        # 层3 未通过（或墙体消失）→ 退出避障模式
        if not wall_ahead:
            self._avoid_active = False
            self._avoid_turn_right = None

        # ── 正常前进：右墙跟随 PID ──────────────────────────────────────
        twist = Twist()
        if dist_front_wide < self.safe_distance * 1.4:
            # 接近但尚未确认：微转向预防
            tr = self._avoid_turn_right if self._avoid_turn_right is not None else (dist_left >= dist_right)
            twist.linear.x = self.obstacle_speed
            twist.angular.z = -0.5 if tr else 0.5
        else:
            # 避障完成后自动回正：PID 右墙跟随
            z = self.pid(dist_right, 0.05)
            twist.angular.z = float(np.clip(z, -0.8, 0.8))
            twist.linear.x = self.forward_speed
        return twist

    def _execute_cb(self, goal_handle):
        self.get_logger().info('Explore action started')
        self._running = True
        self.aruco_pose = None
        max_duration = goal_handle.request.max_duration_sec
        if max_duration <= 0:
            max_duration = 300.0
        start_time = time.monotonic()
        prev_time = start_time

        try:
            while rclpy.ok() and self._running:
                elapsed = time.monotonic() - start_time
                if elapsed > max_duration:
                    self.get_logger().info('Explore timeout')
                    break
                if self.aruco_pose is not None:
                    self.get_logger().info('ArUco detected!')
                    break

                # Publish feedback
                feedback = Explore.Feedback()
                feedback.elapsed_sec = float(elapsed)
                goal_handle.publish_feedback(feedback)

                # Wall follow (Lab4 logic)
                twist = self._compute_wall_follow_twist()
                if twist is not None:
                    self.cmd_pub.publish(twist)
                    self.drive_pub.publish(twist)
                else:
                    # Fallback: no /scan yet - drive slowly forward
                    t = Twist()
                    t.linear.x = 0.12
                    t.angular.z = 0.0
                    self.cmd_pub.publish(t)
                    self.drive_pub.publish(t)

                rclpy.spin_once(self, timeout_sec=0.05)
        finally:
            self._stop_robot()
            self._running = False

        result = Explore.Result()
        result.success = self.aruco_pose is not None
        if self.aruco_pose is not None:
            result.aruco_pose = self.aruco_pose
        else:
            result.aruco_pose = PoseStamped()

        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ExploreActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._stop_robot()
        except Exception:
            pass  # context may already be invalid during shutdown
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
