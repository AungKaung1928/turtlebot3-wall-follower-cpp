# TurtleBot3 Wall Follower (C++)

A C++17 ROS 2 Humble node that finds the nearest wall with a 2D LiDAR and follows it at a
fixed standoff in Gazebo Harmonic: two-beam wall-angle estimate, filtered PD on the
look-ahead distance, three-state FSM with collision pre-emption.

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/Installation.html)
[![C++](https://img.shields.io/badge/C++-17-blue?style=for-the-badge&logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/17)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange?style=for-the-badge&logo=gazebo&logoColor=white)](https://gazebosim.org/docs/harmonic/ros_installation/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange?style=for-the-badge&logo=ubuntu&logoColor=white)](https://ubuntu.com/)

Python sibling with the same behaviour and benchmark: [turtlebot3-wall-follower](https://github.com/AungKaung1928/turtlebot3-wall-follower).

## Node graph
```
 gz sim (Harmonic)                 ros_gz_bridge                wall_follower_cpp
 ┌──────────────────┐   gz.msgs   ┌───────────────┐   ROS 2    ┌────────────────────────┐
 │ gpu_lidar 5 Hz   ├────────────►│ /scan         ├───────────►│ WallDetector           │
 │ DiffDrive        │◄────────────┤ /cmd_vel      │◄───────────┤ PidController          │
 │ /clock /odom /tf ├────────────►│ (gz_bridge.yaml)          │ FSM SEARCHING/FOLLOWING│
 └──────────────────┘             └───────────────┘            │     /AVOIDING  20 Hz   │
                                                                └──────────┬─────────────┘
                                                                           └─► /wall_follower/state
```

## Key decisions
- **Two-beam wall estimate, not one side beam.** Side beam `b` at ±90° and `a` swung 40°
  forward give the wall angle `alpha = atan2(b − a·cosθ, a·sinθ)` and the true perpendicular
  distance `b·cos(alpha)`. A single side beam inflates by 1/cos(yaw) as soon as the controller
  steers inward, which made the old node declare "wall lost" every second.
- **Look-ahead error.** The PD acts on `d_perp − lookahead·sin(alpha) − desired`, so a heading
  that converges on the wall is corrected before the wall arrives. Derivative is EMA-filtered
  (α = 0.3) because dividing by dt = 0.05 s amplifies scan noise 20x.
- **Outside corners arc, inside corners stop-and-turn.** When both beams look past the end of a
  wall while the rear-side arc still has it, the node arcs at the standoff radius. A blocked
  front pre-empts into AVOIDING (turn away from the followed wall, back off if wedged), then
  resumes FOLLOWING with the lock kept.
- **Every number is a declared, range-checked parameter** in `config/wall_following_params.yaml`
  (the previous version hard-coded all gains and never read its YAML).
- **Shutdown really stops the robot.** `rclcpp::init` with `SignalHandlerOptions::None` and an
  own SIGINT flag, so the zero Twist is published while the context is still valid.
- **Scan QoS = `SensorDataQoS`** (best effort), compatible with both the sim bridge and the
  real LDS driver.
- **Sim side is pure gz-sim**: robot plugins in `description/turtlebot3_burger.urdf.xacro`,
  worlds in plain SDF 1.8, no `turtlebot3_gazebo`, no Gazebo Classic.

## Layout
```
turtlebot3-wall-follower-cpp/            # clone into <ws>/src/
├── include/wall_following_cpp_project/
│   ├── wall_detector.hpp                # scan geometry: rangeAt / minInArc / two-beam estimate
│   ├── pid_controller.hpp               # filtered PD
│   └── wall_follower_controller.hpp     # rclcpp::Node, FSM
├── src/                                 # implementations + main.cpp (signal-safe shutdown)
├── config/
│   ├── wall_following_params.yaml       # all controller parameters
│   └── gz_bridge.yaml                   # ros_gz_bridge topics
├── description/turtlebot3_burger.urdf.xacro
├── worlds/
│   ├── wall_follow_world.sdf            # 6 x 6 m room, partition, block (default)
│   └── turtlebot3_world.sdf             # hexagon + 9 pillars benchmark arena
├── models/turtlebot3_world/             # meshes for the arena
├── launch/
│   ├── wall_follower_gazebo_cpp.launch.py   # Gazebo Harmonic + bridge + robot + controller
│   └── wall_following_cpp.launch.py         # controller only
└── rviz/wall_follower_config.rviz
```

## Dependencies
```bash
sudo apt install gz-harmonic ros-humble-ros-gzharmonic \
                 ros-humble-turtlebot3-description ros-humble-xacro ros-humble-robot-state-publisher
```
`gz-harmonic` and `ros-humble-ros-gzharmonic` come from packages.osrfoundation.org
(see gazebosim.org/docs/harmonic/ros_installation). Installing them removes Gazebo Classic.

## Build and launch
```bash
mkdir -p ~/wf_cpp_ws/src && cd ~/wf_cpp_ws/src
git clone https://github.com/AungKaung1928/turtlebot3-wall-follower-cpp.git
cd .. && colcon build --symlink-install && source install/setup.bash

pkill -9 -f "gz sim"      # two gz servers on one bus corrupt /clock and /odom
ros2 launch wall_following_cpp_project wall_follower_gazebo_cpp.launch.py
```
Launch arguments: `world`, `headless:=true` (server only), `x_pose y_pose yaw`, `params_file`.
Benchmark arena: `world:=$(ros2 pkg prefix wall_following_cpp_project)/share/wall_following_cpp_project/worlds/turtlebot3_world.sdf x_pose:=-2.0 y_pose:=-0.5`.

Watch the FSM: `ros2 topic echo /wall_follower/state`.

## Parameters (`config/wall_following_params.yaml`)
| Parameter | Value | Meaning |
|---|---|---|
| `desired_distance` | 0.5 m | standoff from the followed wall |
| `forward_speed` / `search_speed` | 0.18 / 0.12 m/s | cruise / search and corner-arc speed |
| `max_angular_speed` | 1.0 rad/s | clamp on the PD output |
| `kp` / `kd` | 2.0 / 0.6 | PD gains (kd on the filtered derivative) |
| `lookahead_distance` | 0.3 m | horizon for the predicted wall distance |
| `beam_spread_deg` | 40 | angle between the side beam and the forward wall beam |
| `emergency_stop_distance` / `slow_down_distance` | 0.35 / 0.80 m | front zone trip / speed taper |
| `wall_min_distance` / `wall_lost_distance` | 0.28 / 1.2 m | too-close turn-away / lock lost |
| `side_clearance` | 0.25 m | side-zone trip while not following that side |
| `control_frequency` | 20 Hz | timer rate; dt for the derivative |

Constraints: `side_clearance < wall_min < desired < wall_lost`, `emergency_stop < slow_down`.
Out-of-range values are logged and replaced by the range midpoint. Parameters are read once at
start-up; edit the YAML and relaunch.

## Measured results
Independent 10 Hz recorder (not the controller), headless Gazebo Harmonic, target 0.5 m.
"Followed-wall distance" is `min(right, left)` over all samples. Contact = any scan return
below 0.16 m.

| Metric | Room lap (170 s) | turtlebot3_world (120 s) |
|---|---|---|
| Path length | **23.74 m** (full lap) | **17.87 m** |
| Time FOLLOWING / AVOIDING / SEARCHING | 93.0 / 7.0 / 0.0 % | 100 / 0 / 0 % |
| Wall-lost events | **0** | **0** |
| Collision-avoid episodes | 6 (= the corners) | 0 |
| Tracking RMSE | **0.178 m** | **0.066 m** |
| Within ±0.15 m of target | **93.9 %** | **97.7 %** |
| Contact samples | 0 | 0 |

Room = `wall_follow_world.sdf`, spawn (-2, -2), right-hand lap over six corners including the
partition's outside corner. The Python sibling measures 92.7 % / 0.139 m / 95.4 % and
100 % / 0.066 m / 98.3 % on the same two scenarios.

## Planned improvements
- Front-arc steering term so inside corners become a planned arc instead of a 2 s stop-and-turn.
- gtest on `WallDetector::estimate` with synthetic scans (parallel, converging, wall end).
- Real TurtleBot3 run: the scan QoS and index wrapping already handle the LDS-01 conventions.
