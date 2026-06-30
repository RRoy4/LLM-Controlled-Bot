#!/usr/bin/env python3
"""
Custom obstacle-avoidance navigator (no Nav2 required).

All tuning values are ROS 2 parameters — override at launch:
  ros2 run diff_drive_robot navigation.py --ros-args \
      -p goal_x:=3.0 -p goal_y:=2.0 -p base_speed:=0.8
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np


class ReliableObstacleNavigator(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_navigator')

        self.declare_parameter('goal_x',             5.0)
        self.declare_parameter('goal_y',             4.0)
        self.declare_parameter('obstacle_threshold', 1.0)
        self.declare_parameter('clearance_required', 2.0)
        self.declare_parameter('move_distance',      2.5)
        self.declare_parameter('scan_angle_deg',     60.0)
        self.declare_parameter('front_angle_deg',    30.0)
        self.declare_parameter('base_speed',         1.5)
        self.declare_parameter('turn_speed',         3.5)
        self.declare_parameter('goal_tolerance',     0.3)
        self.declare_parameter('timer_period',       0.05)
        self.declare_parameter('cmd_vel_topic',  '/cmd_vel')
        self.declare_parameter('scan_topic',     '/scan')
        self.declare_parameter('odom_topic',     '/odom')

        self.goal = [
            self.get_parameter('goal_x').value,
            self.get_parameter('goal_y').value,
        ]
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.clearance_required = self.get_parameter('clearance_required').value
        self.move_distance      = self.get_parameter('move_distance').value
        self.scan_angle         = math.radians(self.get_parameter('scan_angle_deg').value)
        self.front_angle_range  = math.radians(self.get_parameter('front_angle_deg').value)
        self.base_speed         = self.get_parameter('base_speed').value
        self.turn_speed         = self.get_parameter('turn_speed').value
        self.goal_tolerance     = self.get_parameter('goal_tolerance').value

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        scan_topic    = self.get_parameter('scan_topic').value
        odom_topic    = self.get_parameter('odom_topic').value

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)

        self.state      = 'GOAL_SEEK'
        self.robot_pos  = [0.0, 0.0, 0.0]   # x, y, yaw
        self.start_pos  = [0.0, 0.0]
        self.target_yaw = 0.0
        self.laser_ranges: list = []
        self.laser_angles: list = []

        timer_period = self.get_parameter('timer_period').value
        self.create_timer(timer_period, self.navigate)

        self.get_logger().info(
            f'Navigator ready. Goal: ({self.goal[0]}, {self.goal[1]})')

    # ------------------------------------------------------------------
    # Callbacks — do not modify
    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        self.robot_pos[0] = msg.pose.pose.position.x
        self.robot_pos[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_pos[2] = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y ** 2 + q.z ** 2))

    def scan_callback(self, msg):
        self.laser_ranges = msg.ranges
        if len(self.laser_angles) != len(msg.ranges):
            self.laser_angles = [
                msg.angle_min + i * msg.angle_increment
                for i in range(len(msg.ranges))]

    def distance_moved(self):
        return math.hypot(
            self.robot_pos[0] - self.start_pos[0],
            self.robot_pos[1] - self.start_pos[1])

    # ------------------------------------------------------------------
    # TODO 1 — Front obstacle distance
    # ------------------------------------------------------------------
    def get_front_obstacle_distance(self):
        """
        Return the minimum lidar range within self.front_angle_range of
        straight ahead. Return float('inf') if no scan data is available.

        Use self.laser_ranges and self.laser_angles.
        """
        raise NotImplementedError

    # ------------------------------------------------------------------
    # TODO 2 — Clear direction search
    # ------------------------------------------------------------------
    def find_clear_direction(self):
        """
        Scan candidate headings between -90° and +90° (relative to the robot).
        For each sector, compute the minimum range. Pick the heading whose
        minimum range exceeds self.clearance_required and is the largest.

        Return (True, absolute_yaw)  if a clear direction is found.
        Return (False, goal_yaw)     if nothing is clear (fall back to goal).

        Use self.laser_ranges, self.laser_angles, self.scan_angle,
        self.clearance_required, and self.robot_pos.
        """
        raise NotImplementedError

    # ------------------------------------------------------------------
    # TODO 3 — Navigation FSM
    # ------------------------------------------------------------------
    def navigate(self):
        """
        Timer callback — runs every self.timer_period seconds.

        States: GOAL_SEEK → FIND_CLEAR → MOVE_CLEAR → REALIGN → GOAL_SEEK

        GOAL_SEEK:
          - If within self.goal_tolerance of goal, stop and return.
          - If obstacle closer than self.obstacle_threshold, switch to FIND_CLEAR.
          - Otherwise drive toward the goal: scale linear speed by heading
            alignment, correct heading with angular velocity.

        FIND_CLEAR:
          - Call find_clear_direction() to pick self.target_yaw.
          - Rotate toward it. Once within 5° switch to MOVE_CLEAR and
            record self.start_pos.

        MOVE_CLEAR:
          - Drive forward along self.target_yaw.
          - If self.distance_moved() >= self.move_distance → REALIGN.
          - If a new obstacle appears → back to FIND_CLEAR.

        REALIGN:
          - Rotate back toward the goal.
          - Once within 5° → back to GOAL_SEEK.

        Clamp all twist values to ±self.base_speed / ±self.turn_speed.
        Guard against NaN before publishing.
        """
        raise NotImplementedError


def main(args=None):
    rclpy.init(args=args)
    node = ReliableObstacleNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()