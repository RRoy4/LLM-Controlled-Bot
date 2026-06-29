# LLM-Controlled Bot

---

## Problem Statement

Autonomous mobile robots are built in layers. Before a robot can be trusted to operate on its own, it needs to **move safely**, **explore the unknown**, and **understand what a person actually wants** — without anyone touching a coordinate or a frame name.

This project builds that robot one layer at a time: drive it by hand, teach it to navigate on its own, let it explore a space it's never seen, and finally hand control over to a language model so a plain-text command is enough to send it anywhere on the map.

---

## The Story: From Joystick to Plain Text

A robot is dropped into a maze it has never seen.

At first, it can't do anything by itself — you have to drive it, manually, with the keyboard, just to prove it can move at all.

Then you teach it to drive itself: given a goal point, it should reach it without you touching a key, swerving around whatever gets in its way.

Once it can do that reliably, you let it go further — you stop giving it goals entirely, and it has to decide for itself where the unexplored parts of the map are, drive there, and keep going until there's nothing left to discover.

Finally, the robot stops needing coordinates altogether. You type *"go to room_a"* — and somewhere between your sentence and the robot's wheels, a language model reads your words, looks up where "room_a" actually is, and sends the robot there.

That is the arc of this project: **teleoperation → navigation → exploration → LLM control.**

---

## Objective

Build a complete autonomous **ROS 2 Jazzy** pipeline on top of **Nav2**, **SLAM Toolbox**, and **Gazebo Harmonic**, capable of:

- Driving the robot manually over a ROS 2 topic
- Navigating to a goal point while avoiding obstacles, with no Nav2 required
- Exploring an unknown map on its own using frontier-based exploration
- Accepting plain-English commands and turning them into navigation goals via a local LLM

---

## System Overview

The project consists of four major components, each building on the one before it.

### 1. Teleoperation
A minimal keyboard-to-`Twist` node. Proves the robot can move and that your sim, bridge, and topics are wired correctly before anything autonomous is attempted.

### 2. Custom Navigation
A standalone obstacle-avoidance navigator that does **not** use Nav2. It reads `/scan` and `/odom` directly and drives the robot to a goal point using a finite-state machine: seek the goal, detect an obstacle, find a clear heading, move past it, realign, repeat.

### 3. Frontier Exploration
Once SLAM Toolbox is mapping the environment, the robot has to decide *where* to go next without being told. The frontier explorer scans the live occupancy grid for the boundary between known free space and unknown space, picks the nearest unvisited frontier, and sends it to Nav2 as a goal — repeating until no frontiers remain.

### 4. LLM Navigation
A node that takes plain-English text (typed or published to a topic), sends it to a local Ollama model with a small instruction prompt, parses the model's JSON response, resolves it against a list of named locations, and sends the result to Nav2 as a `NavigateToPose` goal.

---

## What You Need To Implement

This repository contains several TODOs that must be completed. The ROS 2 node boilerplate — parameters, publishers, subscribers, `main()` — is already written. What's missing is the core logic.

### 1. Teleoperation — `scripts/keyboard_teleop.py`

**TODO 1 — Keypress to Twist**

Read keypresses from the terminal and convert them into linear/angular velocity commands published on `/cmd_vel`. Standard WASD (or arrow-key) mapping is expected, with a clean stop on key release and on exit.

**✅ How to check:**
```bash
ros2 launch diff_drive_robot slam_nav.launch.py world_name:=maze
ros2 run diff_drive_robot keyboard_teleop.py
```
Drive in all directions and confirm the robot stops cleanly the moment you release a key. You can also run `ros2 topic echo /cmd_vel` in a separate terminal to confirm the published values match what you'd expect for each key.

---

### 2. Custom Navigation — `scripts/navigation.py`

**TODO 1 — Front Obstacle Detection**

Using the live `/scan` data, compute the minimum distance to anything directly in front of the robot within a configurable angle.

**TODO 2 — Clear Direction Search**

When an obstacle is detected, scan a range of headings around the robot and find the direction with the most clearance that still moves the robot roughly toward the goal.

**TODO 3 — Navigation State Machine**

Implement the four-state FSM (`GOAL_SEEK → FIND_CLEAR → MOVE_CLEAR → REALIGN`) that ties detection and direction-finding together into continuous goal-reaching behavior, publishing `Twist` commands every cycle.

> A second controller, `scripts/pid_controller.py`, asks you to solve the same goal-seeking problem using two independent PID loops (heading and distance) instead of an FSM — useful for comparing control strategies.

**✅ How to check:**
```bash
ros2 launch diff_drive_robot slam_nav.launch.py world_name:=obstacles
ros2 run diff_drive_robot navigation.py --ros-args -p goal_x:=3.0 -p goal_y:=2.0
```
Confirm the robot drives toward the goal, steers around any obstacle in its path without colliding, and comes to a stop once it reaches the goal. Try a few different `goal_x`/`goal_y` values and a couple of obstacle layouts before moving on.

---

### 3. Frontier Exploration — `scripts/frontier_explorer.py`

**TODO 1 — Frontier Detection**

Given the live `/map` occupancy grid, find all **frontier cells** — free cells directly adjacent to unknown cells — and cluster them into connected regions large enough to be worth visiting.

**TODO 2 — Frontier Selection**

From the detected frontier clusters, pick the nearest one the robot hasn't already visited, using the robot's live pose from TF, and send it to Nav2 as a `NavigateToPose` goal. Repeat until no valid frontiers remain.

**✅ How to check:**
```bash
ros2 launch diff_drive_robot slam_nav.launch.py world_name:=maze explore:=true
```
Open RViz and watch the occupancy grid — the robot should start moving toward unexplored regions roughly 12 seconds after launch, with no goals given by you. Confirm it keeps picking new frontiers as old ones get explored, and that it stops cleanly once the whole map is filled in (rather than looping or getting stuck on a tiny leftover frontier).

---

### 4. LLM Navigation — `scripts/llm_nav.py`

**TODO 1 — Goal Resolution**

Given the parsed JSON response from the LLM (`{"action": "go", "location": "room_a"}` or `{"action": "go", "x": ..., "y": ...}`), resolve it into an `(x, y, yaw)` goal — looking up named locations in `config/locations.yaml` where needed — and reject anything malformed or unknown.

**TODO 2 — Goal Dispatch**

Send the resolved goal to the Nav2 `NavigateToPose` action server in a background thread (so the ROS executor never blocks), wait for the result, and report whether the goal succeeded, including how close the robot actually ended up to the target.

> `config/locations.yaml` ships with the key structure in place but no coordinates filled in. You'll populate it yourself once you've explored and mapped the world — this is intentional.

> ⚠️ **Keep commands simple.** This setup runs a small local model (`tinyllama`) through Ollama, not a hosted LLM — it has limited capacity for parsing complex or multi-part instructions. Stick to simple, single-intent commands like `go to room_a` or `go to 2.5 1.0`. Long, compound, or oddly phrased commands ("first go to room_a, then circle back to the kitchen if it's clear") are likely to cause the model to time out or return something `_resolve_goal()` can't parse.

**✅ How to check:**
```bash
# Start nav stack on your saved map
ros2 launch diff_drive_robot robot.launch.py map:=src/diff_drive_robot-main/maps/map_maze.yaml

# Separate terminal — start the LLM navigator
ros2 run diff_drive_robot llm_nav.py
```
Try a named location (`go to room_a`), raw coordinates (`go to 2.5 1.0`), and something invalid (a typo'd location name, or gibberish text) — confirm the valid commands send the robot to the right place and the invalid one is rejected cleanly instead of crashing the node or sending a bogus goal.

---

## Running the Project

> **Before opening any terminal, build the workspace:**
> ```bash
> cd ~/rosnav
> colcon build
> ```
>
> **For every new terminal, source the workspace:**
> ```bash
> source ~/rosnav/install/setup.bash
> ```

---

### Prerequisites

```bash
# ROS 2 Jazzy (Ubuntu 24.04)
sudo apt install -y \
  ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge \
  ros-jazzy-xacro ros-jazzy-joint-state-publisher \
  ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 ros-jazzy-teleop-twist-keyboard
```

**Ollama (for LLM navigation):**
```bash
curl -fsSL https://ollama.com/install.sh | sh
ollama serve &
ollama pull tinyllama
```

> Run order and exact commands for each stage are in the **✅ How to check** block right after that stage's TODOs above. One thing not covered there: once exploration finishes, save the map before moving on to LLM navigation:
> ```bash
> ros2 run nav2_map_server map_saver_cli -f src/diff_drive_robot-main/maps/map_maze
> ```

---

## Deliverables

### 1. Source Code
- Completed implementations for all TODOs
- Working ROS 2 package, building cleanly with `colcon build`
- All provided launch files functioning unmodified

### 2. Demonstration Video
Show, in order:
- Keyboard teleoperation
- Custom navigation reaching a goal and avoiding an obstacle
- Frontier exploration completing a full map
- LLM navigation responding correctly to at least three different plain-English commands

### 3. Report
Briefly describe:
- Your navigation FSM (or PID) design and how you tuned it
- Your frontier detection and selection strategy
- How you structured your LLM prompt and handled malformed responses
- Challenges faced and what you'd improve given more time

---

## Final Message

Congratulations — if you've made it through all four stages, you've built a robot that can be driven by hand, navigate on its own, explore a space it's never seen, and respond to plain-text commands by reading a map it built itself. That's a complete autonomy stack, end to end, and it's no small thing to have working.

If you want to keep going, this project has plenty of room to grow:

- **Smarter LLM parsing** — handle multi-step commands ("go to room_a, then room_b"), relative instructions, or confirmation prompts before executing a goal.
- **Better exploration** — add frontier-size weighting, information-gain scoring, or a smarter revisit policy so the robot explores more efficiently.
- **Tighter control** — swap the FSM in `navigation.py` for a smoother controller, or tune the PID gains in `pid_controller.py` for faster, less jerky goal-seeking.
- **New environments** — try `warehouse`, `house`, or `corridor` instead of `maze`, and see what breaks.
- **Voice input** — feed `llm_nav.py` from a speech-to-text pipeline instead of typed text, so commands can be spoken instead of typed.
- **Multiple robots** — extend the stack to coordinate more than one robot exploring or navigating the same map.

Pick whichever direction is most interesting to you and keep building.
