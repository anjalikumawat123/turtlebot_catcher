#!/usr/bin/env python3
"""
TurtleBot Pursuit & Evasion Challenge — Optimized Catcher Node
===============================================================
Strategy (Round 1 — fastest capture wins):

  PHASE 1 — ORIENT: Rotate in place to face the runner before moving.
            Avoids wide arcing paths that waste time.

  PHASE 2 — INTERCEPT: Predict where the runner will be in T seconds
            using its estimated velocity, then aim at that intercept point.
            Much faster than chasing the current position.

  PHASE 3 — SPRINT: Drive at full speed (0.31 m/s) once aligned.
            Use angular correction while moving — no stopping to turn.

  PHASE 4 — CAPTURE: Stop immediately when within capture_distance.

  OBSTACLE AVOIDANCE: Weighted LiDAR repulsion — steer away from
  obstacles while continuing to pursue. No full stops.

Compatible with: ROS 2 Humble, TurtleBot 4 Lite, Gazebo Ignition Fortress.
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def yaw_from_quaternion(q) -> float:
    """Extract yaw angle (radians) from geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value: float, limit: float) -> float:
    """Clamp value to [-limit, +limit]."""
    return max(-limit, min(limit, value))


def normalize_angle(angle: float) -> float:
    """Wrap angle to [-π, π]."""
    while angle >  math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle


# ---------------------------------------------------------------------------
# CatcherNode
# ---------------------------------------------------------------------------
class CatcherNode(Node):

    def __init__(self):
        super().__init__('catcher_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('capture_distance',   0.40)   # m  — stop here (smaller = more aggressive)
        self.declare_parameter('linear_gain',        1.0)    # P-gain linear
        self.declare_parameter('angular_gain',       2.5)    # P-gain angular (high = snappy turns)
        self.declare_parameter('max_linear_speed',   0.31)   # m/s — TurtleBot 4 Lite hardware max
        self.declare_parameter('max_angular_speed',  1.90)   # rad/s — TurtleBot 4 Lite hardware max
        self.declare_parameter('obstacle_distance',  0.45)   # m  — LiDAR avoidance trigger
        self.declare_parameter('predict_time',       0.8)    # s  — how far ahead to predict runner
        self.declare_parameter('orient_threshold',   0.25)   # rad — start moving when within this heading error
        self.declare_parameter('runner_topic',       '/runner/pose')
        self.declare_parameter('cmd_vel_topic',      '/cmd_vel')
        self.declare_parameter('odom_topic',         '/odom')
        self.declare_parameter('scan_topic',         '/scan')
        self.declare_parameter('control_rate',       20.0)   # Hz — 20Hz for fast response

        # Read parameters
        self.capture_dist    = self.get_parameter('capture_distance').value
        self.k_lin           = self.get_parameter('linear_gain').value
        self.k_ang           = self.get_parameter('angular_gain').value
        self.max_lin         = self.get_parameter('max_linear_speed').value
        self.max_ang         = self.get_parameter('max_angular_speed').value
        self.obs_dist        = self.get_parameter('obstacle_distance').value
        self.predict_time    = self.get_parameter('predict_time').value
        self.orient_thresh   = self.get_parameter('orient_threshold').value
        runner_topic         = self.get_parameter('runner_topic').value
        cmd_vel_topic        = self.get_parameter('cmd_vel_topic').value
        odom_topic           = self.get_parameter('odom_topic').value
        scan_topic           = self.get_parameter('scan_topic').value
        rate_hz              = self.get_parameter('control_rate').value

        # ── Catcher state ─────────────────────────────────────────────────
        self.catcher_x:   float = 0.0
        self.catcher_y:   float = 0.0
        self.catcher_yaw: float = 0.0

        # ── Runner state + velocity estimator ────────────────────────────
        self.runner_x:    float = None
        self.runner_y:    float = None
        self.runner_vx:   float = 0.0    # estimated runner velocity X
        self.runner_vy:   float = 0.0    # estimated runner velocity Y
        self._prev_runner_x:   float = None
        self._prev_runner_y:   float = None
        self._prev_runner_t:   float = None  # wall time of last runner update

        # ── Obstacle state ────────────────────────────────────────────────
        self.obstacle_ahead:  bool  = False
        self.avoidance_steer: float = 0.0   # weighted steer correction (-=left, +=right)

        # ── Terminal state ────────────────────────────────────────────────
        self.captured: bool = False

        # ── QoS ───────────────────────────────────────────────────────────
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
        self.create_subscription(PoseStamped, runner_topic,  self._runner_cb,  reliable_qos)
        self.create_subscription(Odometry,    odom_topic,    self._odom_cb,    sensor_qos)
        self.create_subscription(LaserScan,   scan_topic,    self._scan_cb,    sensor_qos)

        # ── Publisher ─────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # ── Control loop at 20 Hz ─────────────────────────────────────────
        self.create_timer(1.0 / rate_hz, self._control_loop)

        self.get_logger().info(
            f'Optimized CatcherNode ready. '
            f'capture_distance={self.capture_dist}m  '
            f'max_speed={self.max_lin}m/s  '
            f'runner_topic="{runner_topic}"'
        )

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _runner_cb(self, msg: PoseStamped):
        """
        Update runner position and estimate runner velocity.
        Velocity is used for predictive intercept.
        """
        now = time.monotonic()
        x = msg.pose.position.x
        y = msg.pose.position.y

        # Estimate velocity if we have a previous reading
        if (self._prev_runner_x is not None and
                self._prev_runner_t is not None):
            dt = now - self._prev_runner_t
            if dt > 0.01:  # avoid division by near-zero
                # Low-pass filter: blend new estimate with old (α=0.4)
                alpha = 0.4
                raw_vx = (x - self._prev_runner_x) / dt
                raw_vy = (y - self._prev_runner_y) / dt
                self.runner_vx = alpha * raw_vx + (1.0 - alpha) * self.runner_vx
                self.runner_vy = alpha * raw_vy + (1.0 - alpha) * self.runner_vy

        self.runner_x = x
        self.runner_y = y
        self._prev_runner_x = x
        self._prev_runner_y = y
        self._prev_runner_t = now

    def _odom_cb(self, msg: Odometry):
        """Update catcher position from odometry."""
        self.catcher_x   = msg.pose.pose.position.x
        self.catcher_y   = msg.pose.pose.position.y
        self.catcher_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def _scan_cb(self, msg: LaserScan):
        """
        Compute obstacle avoidance steering from LiDAR.

        Uses a weighted repulsion field:
        - Each beam closer than obs_dist contributes a repulsion force
        - Beams on the left push right (negative z), beams on right push left
        - The total gives a smooth steer correction instead of binary stop/turn
        """
        if not msg.ranges:
            return

        n = len(msg.ranges)
        repulsion = 0.0
        obstacle_found = False

        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            if msg.range_min < r < self.obs_dist:
                # Beam angle in robot frame
                angle = msg.angle_min + i * msg.angle_increment
                # Only consider frontal 180° arc
                if abs(angle) > math.pi / 2.0:
                    continue
                # Weight: stronger repulsion when closer
                weight = (self.obs_dist - r) / self.obs_dist
                # Beam on left (angle>0) → push right (negative)
                # Beam on right (angle<0) → push left (positive)
                repulsion -= weight * math.sin(angle)
                obstacle_found = True

        self.obstacle_ahead  = obstacle_found
        # Scale repulsion to angular correction
        self.avoidance_steer = clamp(repulsion * 2.0, self.max_ang)

    # ── Control loop ───────────────────────────────────────────────────────

    def _control_loop(self):
        """
        Optimized pursuit controller — runs at 20 Hz.

        Algorithm:
          1. Predict runner's future position using velocity estimate.
          2. Compute bearing error to intercept point.
          3. ORIENT phase: if heading error > threshold, rotate fast in place.
          4. SPRINT phase: drive at full speed with angular correction.
          5. Blend in obstacle avoidance steering continuously.
          6. Stop on capture.
        """
        # Terminal state — keep publishing zero so robot stays stopped
        if self.captured:
            self.cmd_pub.publish(Twist())
            return

        # Wait for first runner pose
        if self.runner_x is None:
            self.get_logger().info(
                'Waiting for runner pose...', throttle_duration_sec=3.0
            )
            return

        # ── 1. Predict runner intercept position ─────────────────────────
        # Where will the runner be in predict_time seconds?
        intercept_x = self.runner_x + self.runner_vx * self.predict_time
        intercept_y = self.runner_y + self.runner_vy * self.predict_time

        # ── 2. Geometry to intercept point ───────────────────────────────
        dx = intercept_x - self.catcher_x
        dy = intercept_y - self.catcher_y
        distance_to_runner = math.sqrt(
            (self.runner_x - self.catcher_x) ** 2 +
            (self.runner_y - self.catcher_y) ** 2
        )
        distance_to_intercept = math.sqrt(dx * dx + dy * dy)

        target_bearing = math.atan2(dy, dx)
        bearing_error  = normalize_angle(target_bearing - self.catcher_yaw)

        # ── 3. Capture check (use actual runner position, not intercept) ──
        if distance_to_runner <= self.capture_dist:
            self.cmd_pub.publish(Twist())
            self.captured = True
            self.get_logger().info(
                f'*** RUNNER CAPTURED! distance={distance_to_runner:.3f}m ***'
            )
            return

        # ── 4. Compute base velocity commands ────────────────────────────
        cmd = Twist()

        if abs(bearing_error) > self.orient_thresh and distance_to_runner > 1.0:
            # ORIENT phase: large heading error and far away → rotate fast in place
            # This avoids a wide arc that wastes time
            cmd.linear.x  = 0.10  # small forward creep while turning
            cmd.angular.z = clamp(self.k_ang * bearing_error, self.max_ang)
        else:
            # SPRINT phase: aligned enough → full speed ahead
            # Linear speed: maximum when aligned, scales down only near capture
            if distance_to_runner < 1.0:
                # Slow down slightly in the final metre for precise capture
                linear_cmd = clamp(self.k_lin * distance_to_runner, self.max_lin)
            else:
                # Full speed when far away
                linear_cmd = self.max_lin

            # Angular: proportional correction while moving (no stopping)
            angular_cmd = clamp(self.k_ang * bearing_error, self.max_ang)

            cmd.linear.x  = linear_cmd
            cmd.angular.z = angular_cmd

        # ── 5. Blend obstacle avoidance steering ─────────────────────────
        # Add repulsion steer on top of pursuit steer (don't stop, just deflect)
        if self.obstacle_ahead:
            cmd.angular.z = clamp(
                cmd.angular.z + self.avoidance_steer,
                self.max_ang
            )
            # Reduce speed near obstacles but don't stop
            cmd.linear.x *= 0.5
            self.get_logger().debug(
                f'Obstacle avoidance: steer={self.avoidance_steer:.2f}'
            )

        self.cmd_pub.publish(cmd)

        self.get_logger().debug(
            f'dist={distance_to_runner:.2f}m  '
            f'err={math.degrees(bearing_error):.1f}°  '
            f'vrun=({self.runner_vx:.2f},{self.runner_vy:.2f})  '
            f'lin={cmd.linear.x:.2f}  ang={cmd.angular.z:.2f}'
        )


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
        node.cmd_pub.publish(Twist())  # safety stop
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
