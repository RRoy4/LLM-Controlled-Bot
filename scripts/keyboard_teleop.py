#!/usr/bin/env python3
"""
keyboard_teleop.py  —  Keyboard teleoperation for the diff-drive robot.

Your job: read keypresses from the terminal and publish geometry_msgs/Twist
messages on /cmd_vel so the robot moves accordingly.

Controls to implement
---------------------
  W / ↑   : forward
  S / ↓   : backward
  A / ←   : turn left
  D / →   : turn right
  Q        : forward-left arc
  E        : forward-right arc
  SPACE    : full stop (zero all velocity)
  + / =    : increase speed by 10 %
  - / _    : decrease speed by 10 %
  Ctrl-C   : quit cleanly, publish one final zero-velocity message

Stop-on-release semantics: the robot should stop when no recognised key
is pressed (treat any unknown key as a stop command).

ROS 2 parameters (already declared for you):
  ~linear_speed   (float, default 0.3)  m/s base forward/backward speed
  ~angular_speed  (float, default 0.8)  rad/s base turn speed
  ~publish_hz     (float, default 20.0) publish rate in Hz
  ~cmd_vel_topic  (str,   default /cmd_vel)

Hints
-----
- Use the `tty` and `termios` stdlib modules to read single keypresses
  without waiting for Enter.
- Arrow keys arrive as 3-byte ANSI escape sequences: ESC [ X
  (e.g. up arrow = '\\x1b[A').  Read the first byte; if it is '\\x1b',
  read two more bytes and concatenate.
- Run the ROS executor (rclpy.spin) in a background thread so the
  publish timer keeps firing while your key-reading loop blocks on stdin.
- Always restore terminal settings and publish a zero Twist on exit,
  even if an exception occurs — otherwise the terminal stays in raw mode.

Usage:
  ros2 run diff_drive_robot keyboard_teleop.py
  ros2 run diff_drive_robot keyboard_teleop.py --ros-args \
      -p linear_speed:=0.5 -p angular_speed:=1.2
"""

import sys
import tty
import termios
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

SPEED_STEP = 0.1   # fractional speed change per +/- keypress
SPEED_MIN  = 0.05
SPEED_MAX  = 2.0


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # ── parameters (do not change these) ────────────────────────────────
        self.declare_parameter('linear_speed',  0.3)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('publish_hz',    20.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self._lin_speed = self.get_parameter('linear_speed').value
        self._ang_speed = self.get_parameter('angular_speed').value
        hz              = self.get_parameter('publish_hz').value
        topic           = self.get_parameter('cmd_vel_topic').value

        # ── state ────────────────────────────────────────────────────────────
        self._lin_x: float = 0.0
        self._ang_z: float = 0.0
        self._lock  = threading.Lock()

        # ── ROS publisher + timer (do not change these) ───────────────────
        self._pub = self.create_publisher(Twist, topic, 10)
        self.create_timer(1.0 / hz, self._publish_cb)

        self.get_logger().info(
            f'keyboard_teleop ready  |  lin={self._lin_speed:.2f} m/s  '
            f'ang={self._ang_speed:.2f} rad/s  topic={topic}')

    # ── TODO 1: publish timer callback ───────────────────────────────────────
    def _publish_cb(self):
        """
        Called automatically at ~publish_hz Hz by the ROS timer.

        Read self._lin_x and self._ang_z (use self._lock when accessing them),
        build a Twist message, and publish it on self._pub.

        A Twist has two relevant fields:
          msg.linear.x  = forward/backward speed  (m/s)
          msg.angular.z = left/right rotation rate (rad/s)
        """
        raise NotImplementedError("TODO 1: build and publish a Twist from self._lin_x / self._ang_z")

    # ── TODO 2: velocity helpers ──────────────────────────────────────────────
    def _set_velocity(self, lin_factor: float, ang_factor: float):
        """
        Set the current commanded velocity.

        Multiply self._lin_speed by lin_factor  → self._lin_x
        Multiply self._ang_speed by ang_factor  → self._ang_z

        Acquire self._lock before writing.
        """
        raise NotImplementedError("TODO 2a: set self._lin_x and self._ang_z from the given factors")

    def _stop(self):
        """Set both self._lin_x and self._ang_z to 0.0 (acquire self._lock)."""
        raise NotImplementedError("TODO 2b: zero out self._lin_x and self._ang_z")

    def _change_speed(self, delta: float):
        """
        Increase or decrease both self._lin_speed and self._ang_speed by delta.
        Clamp to [SPEED_MIN, SPEED_MAX].
        Log the new values with self.get_logger().info(...).
        """
        raise NotImplementedError("TODO 2c: clamp and update speed, then log the result")

    # ── TODO 3: key reading loop ──────────────────────────────────────────────
    def read_keys(self):
        """
        Blocking loop that reads keypresses and updates velocity state.

        Steps:
        1. Save terminal settings with termios.tcgetattr(sys.stdin).
        2. Print a short banner so the user knows what keys to press.
        3. Loop while rclpy.ok():
             a. Read one keypress (handle arrow-key escape sequences).
             b. Ctrl-C ('\\x03') → break out of the loop.
             c. SPACE           → call self._stop().
             d. '+' or '='      → call self._change_speed(+step).
             e. '-' or '_'      → call self._change_speed(-step).
             f. Recognised motion key → call self._set_velocity(lin_f, ang_f).
             g. Unknown key     → call self._stop() (stop-on-release semantics).
        4. In a finally block:
             - Restore terminal settings.
             - Call self._stop() then self._publish_cb() to send a final
               zero-velocity message before exiting.

        Use tty.setraw(sys.stdin.fileno()) before each read and restore
        settings immediately after — or wrap everything in a try/finally.
        """
        raise NotImplementedError("TODO 3: implement the keypress reading loop")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    # Spin ROS in a background thread so the timer keeps publishing
    # while the main thread blocks waiting for keypresses.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.read_keys()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()