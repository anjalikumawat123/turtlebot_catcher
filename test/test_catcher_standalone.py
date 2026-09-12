#!/usr/bin/env python3
"""
test_catcher_standalone.py — Standalone integration test for the Catcher node
===============================================================================
This test does NOT require a full Gazebo simulation.
It runs the CatcherNode in the same process and drives it with synthetic data:

  • Publishes a fake /runner/pose that moves in a straight line
  • Publishes fake /odom starting at (0, 0)
  • Subscribes to /cmd_vel and checks the robot moves toward the runner
  • Verifies the robot stops when within capture_distance

Run (after sourcing your workspace):
  python3 test/test_catcher_standalone.py

Or as a pytest test:
  pytest test/test_catcher_standalone.py -v
"""

import math
import threading
import time
import pytest

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# Import the node under test (works when package is installed or run from src)
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from turtlebot_catcher.catcher_node import CatcherNode


# ---------------------------------------------------------------------------
# Fake publisher helper node
# ---------------------------------------------------------------------------
class FakeWorld(Node):
    """Publishes synthetic /runner/pose, /odom and /scan topics."""

    def __init__(self, runner_start=(3.0, 0.0), runner_speed=0.05):
        super().__init__('fake_world')
        self.runner_x, self.runner_y = runner_start
        self.runner_speed = runner_speed  # metres per tick (runner moves on Y axis)
        self.tick = 0

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        self.runner_pub = self.create_publisher(PoseStamped, '/runner/pose', reliable_qos)
        self.odom_pub   = self.create_publisher(Odometry,    '/odom',        sensor_qos)
        self.scan_pub   = self.create_publisher(LaserScan,   '/scan',        sensor_qos)

        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self._tick)

    def _tick(self):
        self.tick += 1
        now = self.get_clock().now().to_msg()

        # Runner drifts along +Y every 5 ticks
        if self.tick % 5 == 0:
            self.runner_y += self.runner_speed

        # ── Publish runner pose ──────────────────────────────────────────
        pose_msg = PoseStamped()
        pose_msg.header.stamp    = now
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self.runner_x
        pose_msg.pose.position.y = self.runner_y
        pose_msg.pose.orientation.w = 1.0
        self.runner_pub.publish(pose_msg)

        # ── Publish catcher odometry (stationary at origin for this test) ─
        odom_msg = Odometry()
        odom_msg.header.stamp    = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.pose.pose.orientation.w = 1.0  # facing +X
        self.odom_pub.publish(odom_msg)

        # ── Publish clear scan (all ranges at max, no obstacles) ─────────
        scan_msg = LaserScan()
        scan_msg.header.stamp    = now
        scan_msg.header.frame_id = 'laser'
        scan_msg.angle_min       = -math.pi
        scan_msg.angle_max       =  math.pi
        scan_msg.angle_increment =  math.pi / 180.0  # 1° resolution
        scan_msg.range_min       = 0.1
        scan_msg.range_max       = 10.0
        n_beams = int((scan_msg.angle_max - scan_msg.angle_min)
                      / scan_msg.angle_increment) + 1
        scan_msg.ranges = [5.0] * n_beams  # all clear
        self.scan_pub.publish(scan_msg)


# ---------------------------------------------------------------------------
# cmd_vel listener
# ---------------------------------------------------------------------------
class CmdVelMonitor(Node):
    """Records all /cmd_vel messages published by the Catcher."""

    def __init__(self):
        super().__init__('cmd_vel_monitor')
        self.messages = []
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self._cb, 10
        )

    def _cb(self, msg: Twist):
        self.messages.append(msg)


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------
def test_catcher_moves_toward_runner():
    """
    End-to-end: runner is at (3,0), catcher starts at origin.
    Within 3 seconds the catcher should publish positive linear.x (moving forward).
    """
    rclpy.init()
    try:
        fake_world  = FakeWorld(runner_start=(3.0, 0.0))
        catcher     = CatcherNode()
        cmd_monitor = CmdVelMonitor()

        # Use MultiThreadedExecutor so all 3 nodes spin concurrently
        executor = MultiThreadedExecutor()
        executor.add_node(fake_world)
        executor.add_node(catcher)
        executor.add_node(cmd_monitor)

        # Spin for 3 seconds in a background thread
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        time.sleep(3.0)
        executor.shutdown(timeout_sec=1.0)

        # ── Assertions ───────────────────────────────────────────────────
        assert len(cmd_monitor.messages) > 0, \
            "No /cmd_vel messages received — catcher did not publish commands!"

        # At least one message should have positive forward velocity
        forward_msgs = [m for m in cmd_monitor.messages if m.linear.x > 0.0]
        assert len(forward_msgs) > 0, \
            f"Catcher never moved forward. Messages: {cmd_monitor.messages[:5]}"

        print(f"\n✓ Catcher published {len(cmd_monitor.messages)} cmd_vel messages.")
        print(f"✓ {len(forward_msgs)} of them had positive linear.x (moving toward runner).")

    finally:
        rclpy.shutdown()


def test_catcher_stops_when_close():
    """
    Runner is placed at (0.3, 0.0) — inside capture_distance (0.5 m).
    Catcher should immediately publish zero velocity.
    """
    rclpy.init()
    try:
        fake_world  = FakeWorld(runner_start=(0.3, 0.0), runner_speed=0.0)
        catcher     = CatcherNode()
        cmd_monitor = CmdVelMonitor()

        executor = MultiThreadedExecutor()
        executor.add_node(fake_world)
        executor.add_node(catcher)
        executor.add_node(cmd_monitor)

        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        time.sleep(2.0)
        executor.shutdown(timeout_sec=1.0)

        # After a brief warm-up (first runner pose received), should stop
        # Check the last few messages — they should all be zero
        recent = cmd_monitor.messages[-3:] if len(cmd_monitor.messages) >= 3 else cmd_monitor.messages
        for msg in recent:
            assert msg.linear.x  == pytest.approx(0.0, abs=1e-6), \
                f"Expected zero linear.x after capture, got {msg.linear.x}"
            assert msg.angular.z == pytest.approx(0.0, abs=1e-6), \
                f"Expected zero angular.z after capture, got {msg.angular.z}"

        print(f"\n✓ Catcher stopped correctly when runner is within capture distance.")
        print(f"✓ Total messages: {len(cmd_monitor.messages)}")

    finally:
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# Main — run tests directly without pytest
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    print("=" * 60)
    print("TEST 1: Catcher moves toward runner")
    print("=" * 60)
    test_catcher_moves_toward_runner()

    print()
    print("=" * 60)
    print("TEST 2: Catcher stops when close")
    print("=" * 60)
    test_catcher_stops_when_close()

    print()
    print("All tests passed ✓")
