#!/usr/bin/env python3
"""
LLM Navigation node — plain-English commands → Nav2 goal.

Pipeline:
  text topic/input → ollama LLM → NavigateToPose action

Usage
─────
  ros2 run diff_drive_robot llm_nav.py

Deps:
  ollama must be running: `ollama serve`
"""

import json
import math
import os
import re
import threading
import time
import urllib.request
import urllib.error

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose


# ── helpers — do not modify ───────────────────────────────────────────────────

def _load_locations(share_dir: str) -> dict:
    candidates = [
        os.path.join(share_dir, 'config', 'locations.yaml'),
        os.path.join(os.path.expanduser('~'), 'rosnav', 'locations.yaml'),
    ]
    try:
        import yaml
    except ImportError:
        return {}
    for p in candidates:
        if os.path.isfile(p):
            with open(p) as f:
                data = yaml.safe_load(f) or {}
            return data.get('locations', {})
    return {}


def _yaw_to_quat(yaw_deg: float):
    yaw = math.radians(yaw_deg)
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


# ── TODO 1 — System prompt ────────────────────────────────────────────────────

# Define _SYSTEM as a module-level string that will be used by _parse_command().
#
# The prompt must instruct the LLM to return ONLY a JSON object — no preamble,
# no explanation, no markdown. It should handle four cases:
#   - Go to a named location  → {"action":"go","location":"<name>"}
#   - Go to coordinates       → {"action":"go","x":<float>,"y":<float>,"yaw":0.0}
#   - Stop                    → {"action":"stop"}
#   - Anything else           → {"action":"unknown","reason":"<why>"}
#
# The prompt receives two format placeholders at call time:
#   {locations} — comma-separated list of known location names
#   {command}   — the raw text the user typed
#
# Include at least two examples so the model has concrete output patterns to
# follow. The quality of your prompt directly determines whether the LLM
# returns clean JSON or unparseable text — experiment with it.

_SYSTEM = None  # replace with your prompt string


# ── TODO 2 — Ollama API call ──────────────────────────────────────────────────

def call_ollama(model: str, prompt: str, base_url: str = 'http://localhost:11434') -> str:
    """
    Send a completion request to a locally running Ollama instance and return
    the model's response string.

    Endpoint: POST {base_url}/api/generate
    Request body (JSON):
      model   — the model name (e.g. "tinyllama")
      prompt  — the full prompt string
      stream  — False (we want a single complete response, not a stream)
      format  — "json" (instructs Ollama to constrain output to valid JSON)

    The response body is JSON. The model's text is in response["response"].
    Return that string.
    """
    raise NotImplementedError


# ── TODO 3 — JSON extraction ──────────────────────────────────────────────────

def _extract_json(text: str) -> dict | None:
    """
    LLMs often wrap their JSON in prose or markdown. This function defensively
    extracts the first {...} block from the raw output string and parses it.

    Return the parsed dict if a valid JSON object is found, None otherwise.
    """
    raise NotImplementedError


# ── ROS node ──────────────────────────────────────────────────────────────────

class LLMNavigator(Node):
    def __init__(self):
        super().__init__('llm_navigator')

        self.declare_parameter('ollama_model', 'tinyllama')
        self.declare_parameter('ollama_url',   'http://localhost:11434')
        self.declare_parameter('nav_action',   'navigate_to_pose')
        self.declare_parameter('frame_id',     'map')

        g = self.get_parameter
        self._ollama_model = g('ollama_model').value
        self._ollama_url   = g('ollama_url').value
        self._frame_id     = g('frame_id').value

        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('diff_drive_robot')
        except Exception:
            share = os.path.join(
                os.path.expanduser('~'), 'rosnav', 'src', 'diff_drive_robot-main')
        self._locations = _load_locations(share)
        self.get_logger().info(f'Loaded locations: {list(self._locations.keys())}')

        self._nav_client = ActionClient(self, NavigateToPose, g('nav_action').value)

        self.create_subscription(String, '/llm_nav/command', self._text_cmd_cb, 10)

        self._current_pose: tuple[float, float] | None = None
        self._goal_xy: tuple[float, float] | None = None
        self._nav_start_time: float | None = None
        self._recovery_count = 0
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)

        self._busy = False
        self._busy_lock = threading.Lock()

        self.get_logger().info(f'LLM nav ready.  ollama={self._ollama_model}')
        self.get_logger().info('Type a command in this terminal or publish to /llm_nav/command')

    # ── LLM parse — do not modify ─────────────────────────────────────────────

    def _parse_command(self, text: str) -> dict | None:
        location_list = ', '.join(self._locations.keys()) if self._locations else 'none'
        prompt = _SYSTEM.format(locations=location_list, command=text)
        try:
            raw = call_ollama(self._ollama_model, prompt, self._ollama_url)
        except (urllib.error.URLError, TimeoutError) as e:
            self.get_logger().error(f'ollama error: {e}')
            return None
        parsed = _extract_json(raw)
        if parsed is None:
            self.get_logger().error(f'LLM returned unparseable: {raw[:200]}')
        return parsed

    # ── TODO 4 — Goal resolution ──────────────────────────────────────────────

    def _resolve_goal(self, parsed: dict) -> tuple[float, float, float] | None:
        """
        Convert the LLM's parsed JSON into an (x, y, yaw_deg) tuple.

        The parsed dict will have one of these shapes:
          {"action": "stop"}
          {"action": "unknown", "reason": "..."}
          {"action": "go", "location": "<name>"}
          {"action": "go", "x": <float>, "y": <float>, "yaw": <float>}

        For "stop": cancel all goals via self._nav_client and return None.
        For "unknown" or unrecognised actions: log a warning and return None.
        For "go" with a location name: look it up in self._locations. Each
          entry is [x, y] or [x, y, yaw]. Return None if the name is missing.
        For "go" with raw coordinates: read x, y, and optional yaw directly.
        Return None for any malformed input.
        """
        raise NotImplementedError

    # ── TODO 5 — Goal dispatch ────────────────────────────────────────────────

    def _send_goal(self, x: float, y: float, yaw_deg: float):
        """
        Dispatch (x, y, yaw_deg) to Nav2 in a background thread so the ROS
        executor is never blocked.
        """
        raise NotImplementedError

    def _send_goal_thread(self, x: float, y: float, yaw_deg: float):
        """
        Wait for the NavigateToPose action server (up to 60s), build a
        PoseStamped goal using self._frame_id and _yaw_to_quat(), and
        send it via self._nav_client.

        Store self._goal_xy, self._nav_start_time, and reset
        self._recovery_count before sending. Register
        self._goal_accepted_cb as the done callback and
        self._feedback_cb as the feedback callback.

        If the server never comes up, log an error, clear self._busy,
        and return.
        """
        raise NotImplementedError

    # ── TODO 6 — Result handling ──────────────────────────────────────────────

    def _result_cb(self, future):
        """
        Called when Nav2 finishes (success or failure).

        Read future.result().status and compare against GoalStatus constants.
        Log whether the goal succeeded or failed, how long it took
        (self._nav_start_time), and how many recoveries Nav2 triggered
        (self._recovery_count).

        If successful and both self._goal_xy and self._current_pose are
        available, compute the Euclidean distance between them and log it
        as the accuracy of the navigation.

        Always clear self._busy at the end (acquire self._busy_lock).
        """
        raise NotImplementedError

    # ── callbacks — do not modify ─────────────────────────────────────────────

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        self._current_pose = (p.x, p.y)

    def _feedback_cb(self, fb):
        dist = fb.feedback.distance_remaining
        self._recovery_count = fb.feedback.number_of_recoveries
        if dist > 0.0:
            self.get_logger().info(
                f'  distance remaining: {dist:.2f}m', throttle_duration_sec=3.0)

    def _goal_accepted_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by Nav2.')
            with self._busy_lock:
                self._busy = False
            return
        handle.get_result_async().add_done_callback(self._result_cb)

    def _text_cmd_cb(self, msg: String):
        self._process(msg.data.strip())

    def handle_typed(self, text: str):
        with self._busy_lock:
            busy = self._busy
        if busy:
            print('Still navigating — wait or type "stop".', flush=True)
            return
        self._process(text)

    def _process(self, text: str):
        self.get_logger().info(f'Command: "{text}"')
        print(f'   Asking {self._ollama_model}…', flush=True)
        parsed = self._parse_command(text)
        if parsed is None:
            return
        self.get_logger().info(f'LLM parsed: {parsed}')
        goal = self._resolve_goal(parsed)
        if goal:
            with self._busy_lock:
                self._busy = True
            self._send_goal(*goal)


# ── main — do not modify ──────────────────────────────────────────────────────

def _ui_loop(node: LLMNavigator):
    print('\n─────────────────────────────────────────', flush=True)
    print(' LLM Navigator  |  ctrl-C to quit', flush=True)
    print(' Type a command → send as text', flush=True)
    print('─────────────────────────────────────────\n', flush=True)
    while rclpy.ok():
        try:
            line = input('> ').strip()
        except (EOFError, KeyboardInterrupt):
            break
        if line:
            node.handle_typed(line)


def main(args=None):
    rclpy.init(args=args)
    node = LLMNavigator()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        _ui_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()