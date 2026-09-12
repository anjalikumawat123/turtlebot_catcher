#!/usr/bin/env python3
"""
Launch file for the TurtleBot 4 Catcher node.

Usage (after building):
  ros2 launch turtlebot_catcher catcher.launch.py

Override parameters on the command line:
  ros2 launch turtlebot_catcher catcher.launch.py runner_topic:=/runner/pose capture_distance:=0.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Declare overridable arguments ──────────────────────────────────────
    args = [
        DeclareLaunchArgument('runner_topic',       default_value='/runner/pose',
                              description='Topic publishing the runner PoseStamped'),
        DeclareLaunchArgument('cmd_vel_topic',      default_value='/cmd_vel',
                              description='Velocity command topic'),
        DeclareLaunchArgument('odom_topic',         default_value='/odom',
                              description='Odometry topic'),
        DeclareLaunchArgument('scan_topic',         default_value='/scan',
                              description='LaserScan topic'),
        DeclareLaunchArgument('capture_distance',   default_value='0.5',
                              description='Stop distance in metres'),
        DeclareLaunchArgument('linear_gain',        default_value='0.6',
                              description='Proportional gain for linear speed'),
        DeclareLaunchArgument('angular_gain',       default_value='1.5',
                              description='Proportional gain for angular speed'),
        DeclareLaunchArgument('max_linear_speed',   default_value='0.3',
                              description='Maximum linear speed m/s'),
        DeclareLaunchArgument('max_angular_speed',  default_value='1.5',
                              description='Maximum angular speed rad/s'),
        DeclareLaunchArgument('obstacle_distance',  default_value='0.4',
                              description='Obstacle avoidance trigger distance (m)'),
        DeclareLaunchArgument('control_rate',       default_value='10.0',
                              description='Control loop frequency in Hz'),
        DeclareLaunchArgument('use_sim_time',       default_value='true',
                              description='Use simulation (Gazebo) clock'),
    ]

    # ── Catcher node ───────────────────────────────────────────────────────
    catcher_node = Node(
        package='turtlebot_catcher',
        executable='catcher_node',
        name='catcher_node',
        output='screen',
        parameters=[{
            'use_sim_time':      LaunchConfiguration('use_sim_time'),
            'runner_topic':      LaunchConfiguration('runner_topic'),
            'cmd_vel_topic':     LaunchConfiguration('cmd_vel_topic'),
            'odom_topic':        LaunchConfiguration('odom_topic'),
            'scan_topic':        LaunchConfiguration('scan_topic'),
            'capture_distance':  LaunchConfiguration('capture_distance'),
            'linear_gain':       LaunchConfiguration('linear_gain'),
            'angular_gain':      LaunchConfiguration('angular_gain'),
            'max_linear_speed':  LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            'obstacle_distance': LaunchConfiguration('obstacle_distance'),
            'control_rate':      LaunchConfiguration('control_rate'),
        }],
        # Remapping example: competition may use /target/pose instead of /runner/pose
        # remappings=[('/runner/pose', '/target/pose')],
    )

    return LaunchDescription(args + [catcher_node])
