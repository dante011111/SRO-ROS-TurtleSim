# turtle_mission

Autonomous control of `turtle1` in **turtlesim** using **ROS 1 (Noetic)**.
The turtle is driven only through `/turtle1/cmd_vel`, using closed-loop feedback from `/turtle1/pose`.
There is no keyboard teleop and no teleport service anywhere in this package.

| | |
|---|---|
| **Author** | _Your Name_ (_Your Student ID_) |
| **Course** | _Course name_ |
| **ROS version** | Noetic (Ubuntu 20.04), Python 3 |
| **Package** | `turtle_mission` |

## Table of contents

1. [Levels at a glance](#1-levels-at-a-glance)
2. [Package structure](#2-package-structure)
3. [Concepts used](#3-concepts-used)
4. [Installation and build](#4-installation-and-build)
5. [Custom messages and service](#5-custom-messages-and-service)
6. [Level 1: Waypoint Patrol](#6-level-1-waypoint-patrol)
7. [Level 2: Safe Explorer](#7-level-2-safe-explorer)
8. [Level 3: Hunter](#8-level-3-hunter)
9. [Bonus ideas](#9-bonus-ideas)
10. [Running the project](#10-running-the-project)
11. [Verifying the assignment rules](#11-verifying-the-assignment-rules)
12. [Troubleshooting](#12-troubleshooting)
13. [Demo and Q&A notes](#13-demo-and-qa-notes)

## 1. Levels at a glance

| Level | Behavior | Launch command | Custom nodes |
|---|---|---|---|
| **1. Waypoint Patrol** | Visits 4+ waypoints in order, stops within 0.1 units of each | `roslaunch turtle_mission level1.launch auto_start:=true` | `mission_manager`, `waypoint_controller` |
| **2. Safe Explorer** | Wanders around the arena without touching the walls, using a virtual sensor message | `roslaunch turtle_mission level2.launch` | `virtual_sensor`, `explorer` |
| **3. Hunter** | Spawns prey turtles, chases them, kills them under 0.5 units, publishes a score | `roslaunch turtle_mission level3.launch` | `hunter` |

Each level has its own launch file. Run one at a time. Two levels running together would have two nodes publishing to `/turtle1/cmd_vel`, and the turtle would receive conflicting commands.

## 2. Package structure

```
turtle_mission/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── waypoints.yaml          # waypoint list and controller gains (Level 1)
├── launch/
│   ├── level1.launch
│   ├── level2.launch
│   └── level3.launch
├── msg/
│   ├── Waypoint.msg            # manager -> controller: current target
│   ├── MissionStatus.msg       # controller -> manager: progress
│   ├── WallSensor.msg          # virtual sensor readings (Level 2)
│   └── Score.msg               # hunter score (Level 3)
├── srv/
│   └── StartMission.srv        # /start_mission service (Level 1)
└── scripts/
    ├── mission_manager.py      # Level 1
    ├── waypoint_controller.py  # Level 1
    ├── virtual_sensor.py       # Level 2
    ├── explorer.py             # Level 2
    └── hunter.py               # Level 3
```

## 3. Concepts used

| ROS concept | Where it is used |
|---|---|
| **Node** | Every script in `scripts/` is a node |
| **Topic, Publisher, Subscriber** | `/turtle1/cmd_vel`, `/turtle1/pose`, `/mission/target`, `/mission/status`, `/wall_sensor`, `/score` |
| **Message (custom)** | `Waypoint`, `MissionStatus`, `WallSensor`, `Score` |
| **Service (custom)** | `/start_mission` (`StartMission.srv`) |
| **Service (built-in client)** | `/spawn` and `/kill` from turtlesim (Level 3) |
| **Parameter server** | Waypoints and gains in `waypoints.yaml`; `~safe_distance`, `~speed`, `~rounds` and others |
| **Launch file** | One `roslaunch` command per level |

## 4. Installation and build

```bash
# 1. Install ROS Noetic and turtlesim if needed
sudo apt install ros-noetic-desktop-full ros-noetic-turtlesim

# 2. Create a workspace (skip if you already have one)
mkdir -p ~/catkin_ws/src

# 3. Put this package in ~/catkin_ws/src/turtle_mission

# 4. Make scripts executable
chmod +x ~/catkin_ws/src/turtle_mission/scripts/*.py

# 5. Build
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Add this to `~/.bashrc` so every terminal is ready:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

Rebuild with `catkin_make` after any change to a `.msg`, `.srv`, or `CMakeLists.txt`. Editing a `.py` script needs no rebuild.

## 5. Custom messages and service

### `msg/Waypoint.msg`: the current target

```
uint8 index      # position of this waypoint in the route (0-based)
uint8 total      # total waypoints in the route
float32 x
float32 y
```

Published by `mission_manager` on `/mission/target`.

### `msg/MissionStatus.msg`: controller progress

```
uint8 waypoint_index
float32 distance_to_target
bool reached
string state     # IDLE, DRIVING or REACHED
```

Published by `waypoint_controller` on `/mission/status`.

### `srv/StartMission.srv`: start the patrol

```
bool return_home    # request: add the start point as a final waypoint
---
bool accepted       # response
string message
```

Served by `mission_manager` at `/start_mission`.

### `msg/WallSensor.msg`: virtual sensor (Level 2)

```
float32 front_distance   # distance to the wall straight ahead
float32 left_distance    # ray 40 degrees to the left
float32 right_distance   # ray 40 degrees to the right
float32 min_distance     # distance to the nearest wall
string nearest_wall      # left, right, top or bottom
bool near_wall           # min_distance < safe_distance
```

Published by `virtual_sensor` on `/wall_sensor`.

### `msg/Score.msg`: hunter score (Level 3)

```
int32 score
string last_prey
```

Published by `hunter` on `/score`.

## 6. Level 1: Waypoint Patrol

**Goal:** visit at least 4 waypoints in order and stop within 0.1 units of each. After the last one, stop (or return to the start).

### Architecture

```
                 /start_mission (srv)
                        │
                        ▼
 ┌──────────────────┐  /mission/target (Waypoint)   ┌─────────────────────┐
 │ mission_manager  │ ────────────────────────────▶ │ waypoint_controller │
 │ (route + order)  │ ◀──────────────────────────── │   (P-controller)    │
 └──────────────────┘  /mission/status              └─────────────────────┘
                       (MissionStatus)                 ▲              │
                                          /turtle1/pose│              │/turtle1/cmd_vel
                                                       │              ▼
                                                    ┌───────────────────┐
                                                    │     turtlesim     │
                                                    └───────────────────┘
```

### Nodes

**`mission_manager.py`** is the brain. It:
- reads the waypoint list from the parameter server (the single source of the route);
- offers the `/start_mission` service;
- publishes one target at a time on `/mission/target` (latched, so the controller gets the latest even if it starts late);
- listens to `/mission/status` and advances to the next waypoint when `reached` is true for the current index;
- optionally appends the start position as a final waypoint (`return_home: true`);
- with `auto_start:=true`, starts the mission by itself 1.5 s after launch.

**`waypoint_controller.py`** is the closed-loop controller. At 50 Hz it:
1. reads the pose from `/turtle1/pose`;
2. computes distance and heading error to the target;
3. publishes a `Twist` to `/turtle1/cmd_vel`;
4. publishes progress on `/mission/status`.

### The control law

```
dx, dy        = target - pose
distance      = sqrt(dx² + dy²)
heading_error = wrap( atan2(dy, dx) - pose.theta )

angular.z = Kp_angular × heading_error
linear.x  = min( Kp_linear × distance, v_max ) × max(0, cos(heading_error))
```

- **Angular P-control** turns the turtle toward the target.
- **Linear P-control** makes the turtle slow down as it approaches, so it does not overshoot.
- **The `cos(heading_error)` factor** makes the turtle rotate in place when it faces away, and drive when it faces the target.
- **Stopping:** when `distance < tolerance` the controller publishes a zero `Twist` and reports `reached`.
- `wrap()` normalizes the angle to [-π, π], so the turtle always takes the shortest turn.

### Configuration: `config/waypoints.yaml`

```yaml
mission:
  tolerance: 0.05      # assignment requires < 0.1
  kp_linear: 1.5
  kp_angular: 4.0
  max_linear: 2.0
  waypoints:
    - [2.0, 2.0]
    - [9.0, 2.0]
    - [9.0, 9.0]
    - [2.0, 9.0]
```

The waypoint list lives only in this file. To change the route, edit the list. No code changes are needed. The turtlesim arena spans roughly 0 to 11.08 on both axes, and the turtle starts near (5.54, 5.54).

### Run

```bash
roslaunch turtle_mission level1.launch auto_start:=true
```

Or start the mission manually:

```bash
roslaunch turtle_mission level1.launch
# in a second terminal:
rosservice call /start_mission "return_home: true"
```

### Level 1 checklist

| Criterion | How it is met |
|---|---|
| Reaches each waypoint, tolerance < 0.1 | `tolerance: 0.05` in the YAML |
| Stops (or returns) after the last waypoint | Manager sets `running = False`; `return_home` adds the start point |
| Waypoint list easy to change | One list in `config/waypoints.yaml` |

## 7. Level 2: Safe Explorer

**Goal:** explore the arena without hitting the walls. The explorer must act on a custom-message "virtual sensor" and not on raw pose.

### Architecture

```
 turtlesim ──/turtle1/pose──▶ virtual_sensor ──/wall_sensor──▶ explorer ──/turtle1/cmd_vel──▶ turtlesim
                              (3 range rays)    (WallSensor)    (wander + avoid)
```

### Nodes

**`virtual_sensor.py`** simulates a small range sensor. From `/turtle1/pose` it casts three rays against the arena walls (front, 40° left, 40° right) using simple ray-versus-wall geometry. It also finds the nearest wall and sets `near_wall` if that distance is below `safe_distance`. On shutdown it logs the closest approach to any wall, which is useful evidence in a demo.

**`explorer.py`** never reads the pose. It subscribes only to `/wall_sensor`. Its behavior has two states:

| State | Condition | Action |
|---|---|---|
| **Wander** | Path is clear | Drive forward at up to `speed`, with a random gentle turn that changes every 1 to 2.5 s |
| **Avoid** | Front ray < `safe`, or a side ray < 0.8 × `safe` | Stop and turn in place toward the side with more room |

Design details:
- **Hysteresis:** avoidance starts when `front < safe` but ends only when `front > 2 × safe` and both sides are above `safe`. This prevents flickering between turning and driving at the threshold.
- **Side rays:** the front ray alone can miss a wall approached at a shallow angle. The ±40° rays cover that case.
- **Slowing down:** `linear.x = min(speed, 1.2 × front_distance)` makes the turtle slow near walls.
- **Freshness check:** if no sensor message arrives for 0.5 s, the explorer stops instead of driving blind.

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `safe_distance` | 1.0 | Wall distance that triggers avoidance |
| `speed` | 1.5 | Maximum forward speed |
| `turn_speed` | 2.0 | Turning rate while avoiding (set in `explorer.py`) |

### Run

```bash
roslaunch turtle_mission level2.launch
roslaunch turtle_mission level2.launch safe_distance:=1.5 speed:=2.0   # safer and faster
roslaunch turtle_mission level2.launch plot:=true                      # live plot of the 3 rays
```

### Proving "no wall hits"

- Turtlesim prints `Oh no! I hit the wall!` if the turtle touches a boundary. A good run shows none.
- Run for 2 to 3 minutes, stop with `Ctrl+C`, and read the "Closest approach" line from `virtual_sensor`.
- Restart several times. The random start and wander differ each run, and corners are the hardest case.

## 8. Level 3: Hunter

**Goal:** spawn a target turtle at a random position, chase it, kill it when the distance is below 0.5, repeat, and publish the score.

### Architecture

```
                /spawn (srv) ──▶ turtlesim ──▶ prey1, prey2, ... turtles
                /kill  (srv) ──▶ turtlesim
                                     │
 /turtle1/pose ─────────┐            │ /preyN/pose
                        ▼            ▼
                    ┌────────────────────┐ ──/turtle1/cmd_vel──▶ turtlesim
                    │       hunter       │
                    └────────────────────┘ ──/score (Score)────▶ any subscriber
```

### Node: `hunter.py`

Each round does the following:

1. **Spawn.** Calls `/spawn` with a random position in [1, 10] × [1, 10], a random heading, and a unique name (`prey1`, `prey2`, ...).
2. **Track.** Subscribes to `/preyN/pose` and to `/turtle1/pose`.
3. **Chase.** Uses the same P-controller as Level 1, with the prey's live position as the target. The target moves, so the controller keeps correcting.
4. **Catch.** When `distance < kill_distance` (0.5), it stops and unsubscribes from the prey's pose topic. It then calls `/kill`, increments the score, and publishes on `/score`.
5. **Repeat** until `rounds` catches are reached, then stop the turtle.

The hunter still moves only through `/turtle1/cmd_vel`. `/spawn` and `/kill` are used only to create and remove the prey.

Spawning inside [1, 10] keeps prey away from the edges. The path between two points inside the arena stays inside the arena, so the hunter does not need wall avoidance. If you want extra safety, you can subscribe to `/wall_sensor` and add a check.

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `~kill_distance` | 0.5 | Catch threshold |
| `~rounds` | 5 | Number of prey to catch |

### Run

```bash
roslaunch turtle_mission level3.launch
rostopic echo /score        # in a second terminal
```

Set the round count with `<param name="rounds" value="5"/>` in `launch/level3.launch`, or run the node directly with `rosrun turtle_mission hunter.py _rounds:=8`.

### Level 3 checklist

| Requirement | How it is met |
|---|---|
| Spawn target at random position via `/spawn` | `spawn_prey()` |
| Chase the target | P-controller toward the live prey pose |
| `/kill` at distance < 0.5 | `catch()` |
| Repeat | Loop over `rounds` |
| Publish score on a custom topic | `/score` (`Score.msg`) |
| Fully autonomous | No input needed after launch |

## 9. Bonus ideas

These are optional and do not replace Level 3.

- **Pen color per phase.** Use the allowed `/turtle1/set_pen` service (it is not teleport):
  ```python
  from turtlesim.srv import SetPen
  set_pen = rospy.ServiceProxy("/turtle1/set_pen", SetPen)
  set_pen(255, 0, 0, 3, 0)    # r, g, b, width, off
  ```
- **Draw your initials.** Encode letter strokes as waypoints in the YAML and use `off=1` when moving between letters.
- **Leader–follower.** `/spawn` a second turtle and run the same P-controller on `/turtle2/cmd_vel`, using `/turtle1/pose` as its target.

## 10. Running the project

Recommended workflow: one terminal to launch, one to monitor.

```bash
# Terminal 1: launch (roslaunch starts roscore automatically)
roslaunch turtle_mission level1.launch auto_start:=true

# Terminal 2: monitor
rostopic list
rostopic echo /mission/status
rqt_graph
```

Useful commands:

```bash
rosservice call /clear          # wipe the drawn trail between runs
rosservice call /reset          # put turtle1 back at the start
rosparam get /mission/waypoints # check the loaded route
```

If a level misbehaves, run its nodes separately, one terminal each, to find the faulty one:

```bash
roscore
rosrun turtlesim turtlesim_node
rosrun turtle_mission virtual_sensor.py
rosrun turtle_mission explorer.py
```

## 11. Verifying the assignment rules

| Rule | Evidence | Command |
|---|---|---|
| 1. Motion only via `/turtle1/cmd_vel`; no teleop or teleport | The only publishers are the project nodes; no code calls `teleport_*` | `rostopic info /turtle1/cmd_vel` |
| 2. Closed loop on `/turtle1/pose` | Controllers compute commands from pose (Levels 1 and 3) or from the pose-derived sensor (Level 2); no timed sequences | read the code |
| 3. At least 2 custom nodes talking by topic | Level 1: `mission_manager` ↔ `waypoint_controller`. Level 2: `virtual_sensor` → `explorer` | `rqt_graph` |
| 4. At least 1 custom `.msg` and 1 `.srv` | 4 messages and `StartMission.srv` | `rosmsg show turtle_mission/Waypoint`, `rossrv show turtle_mission/StartMission` |
| 5. Own catkin package, single `roslaunch` | `turtle_mission`, one launch file per level | `roslaunch turtle_mission level1.launch auto_start:=true` |

Rule 1 check: in Level 1, `rostopic info /turtle1/cmd_vel` must list only `/waypoint_controller` as a publisher. In Level 2 it must list only `/explorer`. In Level 3 it must list only `/hunter`.

## 12. Troubleshooting

| Symptom | Fix |
|---|---|
| `ImportError: cannot import name ...` from `turtle_mission.msg` | Run `catkin_make`, then `source ~/catkin_ws/devel/setup.bash` |
| `Couldn't find executable named ...py` | `chmod +x ~/catkin_ws/src/turtle_mission/scripts/*.py` |
| `package 'turtle_mission' not found` | `source ~/catkin_ws/devel/setup.bash` in this terminal |
| Turtle does not move (Level 1) | Call `/start_mission`, or launch with `auto_start:=true` |
| Turtle does not move (Level 2) | `rostopic hz /wall_sensor` should show about 60 Hz. If empty, check `virtual_sensor` |
| Turtle jerks or fights itself | Two nodes publish to `/turtle1/cmd_vel`. Stop the other launch |
| Turtle spirals around a waypoint | Lower `kp_angular` |
| Turtle never "arrives" | Keep `tolerance` at 0.05 or higher and the loop at 50 Hz |
| Turtle grazes walls (Level 2) | Increase `safe_distance` or lower `speed` |
| `/spawn` name already exists (Level 3) | Restart turtlesim, or call `/reset` before relaunching |
| Turtlesim window does not appear (WSL or VM) | It needs a display: WSLg on Windows 11, or run from the VM desktop |
