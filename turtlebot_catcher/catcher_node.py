#!/usr/bin/env python3
"""
TurtleBot Pursuit & Evasion Challenge — Catcher Node
=====================================================
Strategy:
  1. Subscribe to the Runner's pose (geometry_msgs/PoseStamped on /runner/pose).
     Falls back to the Gazebo bridge topic automatically if the competition
     publishes a different topic name — just remap at launch time.
  2. Subscribe to our own odometry (/odom) so we always know where we are.
  3. Subscribe to /scan (LaserScan) for simple obstacle avoidance.
  4. Compute bearing + distance from Catcher → Runner.
  5. Drive with proportional angular + linear control.
  6. Stop when within CAPTURE_DISTANCE metres.

Compatible with: ROS 2 Humble, TurtleBot 4 Lite, Gazebo Ignition Fortress.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


# ---------------------------------------------------------------------------
# Helper: extract yaw from a geometry_msgs/Quaternion
# ---------------------------------------------------------------------------
def yaw_from_quaternion(q) -> float:
    """Return yaw (radians) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# ---------------------------------------------------------------------------
# Helper: clamp a value to [-limit, limit]
# ---------------------------------------------------------------------------
def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


# ---------------------------------------------------------------------------
# CatcherNode
# ---------------------------------------------------------------------------
class CatcherNode(Node):

    def __init__(self):
        super().__init__('catcher_node')

        # ── Parameters (tunable without recompiling) ─────────────────────
        self.declare_parameter('capture_distance',  0.50)  # metres — stop here
        self.declare_parameter('linear_gain',       0.6)   # P-gain for linear speed
        self.declare_parameter('angular_gain',      1.5)   # P-gain for angular speed
        self.declare_parameter('max_linear_speed',  0.3)   # m/s  (TB4 Lite: 0.31 m/s max)
        self.declare_parameter('max_angular_speed', 1.5)   # rad/s
        self.declare_parameter('obstacle_distance', 0.40)  # metres — start avoiding
        self.declare_parameter('runner_topic',      '/runner/pose')  # remappable
        self.declare_parameter('cmd_vel_topic',     '/cmd_vel')
        self.declare_parameter('odom_topic',        '/odom')
        self.declare_parameter('scan_topic',        '/scan')
        self.declare_parameter('control_rate',      10.0)  # Hz

        # Read parameters
        self.capture_dist   = self.get_parameter('capture_distance').value
        self.k_lin          = self.get_parameter('linear_gain').value
        self.k_ang          = self.get_parameter('angular_gain').value
        self.max_lin        = self.get_parameter('max_linear_speed').value
        self.max_ang        = self.get_parameter('max_angular_speed').value
        self.obs_dist       = self.get_parameter('obstacle_distance').value
        runner_topic        = self.get_parameter('runner_topic').value
        cmd_vel_topic       = self.get_parameter('cmd_vel_topic').value
        odom_topic          = self.get_parameter('odom_topic').value
        scan_topic          = self.get_parameter('scan_topic').value
        rate_hz             = self.get_parameter('control_rate').value

        # ── State ─────────────────────────────────────────────────────────
        self.catcher_x:   float = 0.0
        self.catcher_y:   float = 0.0
        self.catcher_yaw: float = 0.0

        self.runner_x:    float = None   # None = not yet received
        self.runner_y:    float = None

        self.obstacle_ahead: bool = False  # set by scan callback
        self.captured:       bool = False  # terminal state

        # ── QoS profiles ─────────────────────────────────────────────────
        # Best-effort for sensor data (LaserScan, Odometry), reliable for pose
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        # ── Subscribers ───────────────────────────────────────────────────
        self.runner_sub = self.create_subscription(
            PoseStamped, runner_topic,
            self._runner_pose_cb, reliable_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic,
            self._odom_cb, sensor_qos
        )
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic,
            self._scan_cb, sensor_qos
        )

        # ── Publisher ─────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # ── Control loop timer ────────────────────────────────────────────
        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f'CatcherNode started. capture_distance={self.capture_dist}m '
            f'listening on runner_topic="{runner_topic}"'
        )

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _runner_pose_cb(self, msg: PoseStamped):
        """Update Runner position from PoseStamped message."""
        self.runner_x = msg.pose.position.x
        self.runner_y = msg.pose.position.y

    def _odom_cb(self, msg: Odometry):
        """Update our own position from odometry."""
        self.catcher_x   = msg.pose.pose.position.x
        self.catcher_y   = msg.pose.pose.position.y
        self.catcher_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def _scan_cb(self, msg: LaserScan):
        """
        Simple obstacle detection: check the frontal arc (±30°) for
        any reading closer than obs_dist.
        """
        if not msg.ranges:
            return

        n = len(msg.ranges)
        # Degrees-to-index helper (handles wrapped indices)
        def arc_indices(center_deg, half_deg):
            center_idx = int((math.radians(center_deg) - msg.angle_min)
                             / msg.angle_increment) % n
            half_idx   = int(math.radians(half_deg) / msg.angle_increment)
            return [(center_idx + i) % n
                    for i in range(-half_idx, half_idx + 1)]

        front_indices = arc_indices(0, 30)  # front ±30°

        self.obstacle_ahead = any(
            msg.range_min < msg.ranges[i] < self.obs_dist
            for i in front_indices
            if not math.isnan(msg.ranges[i]) and not math.isinf(msg.ranges[i])
        )

    # ── Control loop ───────────────────────────────────────────────────────

    def _control_loop(self):
        """
        Main proportional pursuit controller.

        Steps:
          1. Guard: wait until both odom and runner pose are available.
          2. Compute distance + bearing error.
          3. If within capture distance → stop and declare victory.
          4. If obstacle ahead → rotate in place to clear it.
          5. Otherwise → proportional linear + angular command.
        """
        if self.captured:
            self._stop()
            return

        # Wait for first runner pose
        if self.runner_x is None:
            self.get_logger().info('Waiting for runner pose...', throttle_duration_sec=5.0)
            return

        # ── 1. Geometry ──────────────────────────────────────────────────
        dx = self.runner_x - self.catcher_x
        dy = self.runner_y - self.catcher_y
        distance = math.sqrt(dx * dx + dy * dy)

        # Bearing to runner in world frame, then error in robot frame
        target_bearing = math.atan2(dy, dx)
        bearing_error  = self._normalize_angle(target_bearing - self.catcher_yaw)

        # ── 2. Capture check ─────────────────────────────────────────────
        if distance <= self.capture_dist:
            self._stop()
            self.captured = True
            self.get_logger().info(
                f'*** RUNNER CAPTURED! Final distance: {distance:.3f}m ***'
            )
            return

        # ── 3. Obstacle avoidance ─────────────────────────────────────────
        if self.obstacle_ahead:
            # Rotate right to try to go around
            cmd = Twist()
            cmd.linear.x  = 0.0
            cmd.angular.z = -0.5  # fixed turn rate to clear obstacle
            self.cmd_pub.publish(cmd)
            self.get_logger().debug('Obstacle ahead — rotating to avoid.')
            return

        # ── 4. Proportional pursuit ───────────────────────────────────────
        # Scale linear speed down when not facing the target (large heading error)
        heading_factor = math.cos(bearing_error)   # 1 when aligned, 0 at 90°
        heading_factor = max(0.0, heading_factor)  # no backward motion

        linear_cmd  = clamp(self.k_lin * distance * heading_factor, self.max_lin)
        angular_cmd = clamp(self.k_ang * bearing_error,             self.max_ang)

        cmd = Twist()
        cmd.linear.x  = linear_cmd
        cmd.angular.z = angular_cmd
        self.cmd_pub.publish(cmd)

        self.get_logger().debug(
            f'dist={distance:.2f}m  bearing_err={math.degrees(bearing_error):.1f}°  '
            f'lin={linear_cmd:.2f}  ang={angular_cmd:.2f}'
        )

    # ── Utilities ──────────────────────────────────────────────────────────

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Wrap angle to [-π, π]."""
        while angle >  math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def _stop(self):
        """Publish zero velocity."""
        self.cmd_pub.publish(Twist())


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = CatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
