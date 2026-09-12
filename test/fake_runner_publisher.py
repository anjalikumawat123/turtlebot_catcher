#!/usr/bin/env python3
"""
fake_runner_publisher.py
========================
Utility script: publishes a /runner/pose topic with a moving target.
Useful for testing the Catcher WITHOUT a full Gazebo simulation.

Usage:
  # Terminal 1 — run the catcher
  ros2 launch turtlebot_catcher catcher.launch.py use_sim_time:=false

  # Terminal 2 — run this fake runner
  python3 test/fake_runner_publisher.py

  # Terminal 3 — watch cmd_vel output
  ros2 topic echo /cmd_vel

The runner starts at (2.0, 0.0) and moves in a circle of radius 2m.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped


class FakeRunnerPublisher(Node):

    def __init__(self):
        super().__init__('fake_runner_publisher')

        # Declare parameters
        self.declare_parameter('radius',    2.0)   # metres
        self.declare_parameter('speed',     0.3)   # rad/s (angular speed of circle)
        self.declare_parameter('rate_hz',   10.0)  # publish rate

        self.radius  = self.get_parameter('radius').value
        self.speed   = self.get_parameter('speed').value
        rate_hz      = self.get_parameter('rate_hz').value

        self.angle   = 0.0  # current angle on circle (radians)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )
        self.pub = self.create_publisher(PoseStamped, '/runner/pose', reliable_qos)
        self.timer = self.create_timer(1.0 / rate_hz, self._publish)
        self.get_logger().info(
            f'Fake runner started: radius={self.radius}m speed={self.speed}rad/s'
        )

    def _publish(self):
        dt = 1.0 / self.get_parameter('rate_hz').value
        self.angle += self.speed * dt

        # Runner moves in a circle
        x = self.radius * math.cos(self.angle)
        y = self.radius * math.sin(self.angle)

        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)

        self.get_logger().info(f'Runner at ({x:.2f}, {y:.2f})', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRunnerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
