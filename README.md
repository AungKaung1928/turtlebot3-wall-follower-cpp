# 🤖 TurtleBot3 Wall Following Robot

A robust C++ implementation of ROS2-based autonomous wall following robot for TurtleBot3 with intelligent obstacle avoidance and safe navigation. 

## 📋 Prerequisites

Before running this project, ensure you have the following installed:

- [![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/Installation.html) **ROS2 Humble**
- [![C++](https://img.shields.io/badge/C++-17-blue?style=for-the-badge&logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/17) **C++17 or higher**
- [![CMake](https://img.shields.io/badge/CMake-3.8+-red?style=for-the-badge&logo=cmake&logoColor=white)](https://cmake.org/install/) **CMake 3.8+**
- [![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange?style=for-the-badge&logo=gazebo&logoColor=white)](https://classic.gazebosim.org/tutorials?tut=install_ubuntu) **Gazebo Classic**
- [![RViz](https://img.shields.io/badge/RViz2-Visualization-green?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html) **RViz2**

## 📁 Project Structure

```
turtlebot3_wall_follower_cpp_ws/
├── src/
│   └── wall_following_cpp_project/
│       ├── package.xml                         # Package dependencies and metadata
│       ├── CMakeLists.txt                      # CMake build configuration
│       ├── config/
│       │   ├── nav2_params.yaml               # Basic navigation parameters
│       │   └── wall_following_params.yaml     # Wall following parameters
│       ├── launch/
│       │   ├── wall_following_cpp.launch.py   # Main wall follower launch file
│       │   └── wall_follower_gazebo_cpp.launch.py # Gazebo simulation launch
│       ├── rviz/
│       │   └── wall_follower_config.rviz      # RViz visualization configuration
│       ├── include/
│       │   └── wall_following_cpp_project/
│       │       ├── wall_follower_controller.hpp # Main control logic header
│       │       ├── wall_detector.hpp           # Wall detection algorithms header
│       │       └── pid_controller.hpp          # PID control system header
│       └── src/
│           ├── wall_follower_controller.cpp    # Main control logic implementation
│           ├── wall_detector.cpp               # Wall detection algorithms implementation
│           ├── pid_controller.cpp              # PID control system implementation
│           └── main.cpp                        # Main entry point
└── README.md
```

## ✨ Features

- **🛡️ Zero Contact Navigation**: Maintains 0.8m from walls, 0.5m minimum clearance
- **🎯 Simple Proportional Control**: Clean P-control with gain of -2.0 for stable tracking
- **🔍 Smart Wall Detection**: Finds walls within 0.8m range, follows right wall by preference
- **⚠️ Early Obstacle Detection**: Stops at 55cm from front obstacles
- **⚡ Real-time Response**: 20Hz control loop for smooth operation
- **🌍 Continuous Exploration**: Active search mode with forward motion
- **📊 RViz Visualization**: Pre-configured visualization setup

## 🎯 Key Behaviors

### Wall Following Mode
- Maintains **0.8m distance** from walls using proportional control
- **Never gets closer than 0.5m** to walls (safety threshold)
- Uses diagonal sensor (45°) for corner anticipation
- Adjusts speed based on turning intensity

### Obstacle Avoidance  
- **Turns away at 0.55m** from front obstacles
- **Emergency stop at 0.25m** (collision avoidance)
- **Backs up at 0.2m** if really close
- Returns to wall following after clearing

### Search Mode
- **Always moves forward** at 0.1 m/s while searching
- Rotates gently (0.3 rad/s) to find walls
- Detects walls up to 1.0m away
- Prefers right wall when both available

## 📦 Installation

1. **Install TurtleBot3 packages:** 📥
   ```bash
   sudo apt update
   sudo apt install ros-humble-turtlebot3*
   sudo apt install ros-humble-gazebo-ros-pkgs
   ```

2. **Set TurtleBot3 model:** 🤖
   ```bash
   echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Create workspace and build:** 🔧
   ```bash
   cd ~/
   mkdir -p turtlebot3_wall_follower_cpp_ws/src
   cd turtlebot3_wall_follower_cpp_ws/src
   
   # Copy the wall_following_cpp_project folder here
   # (with all files in their respective directories)
   
   cd ~/turtlebot3_wall_follower_cpp_ws
   colcon build --packages-select wall_following_cpp_project
   source install/setup.bash
   ```

## 🚀 Usage

### ⚡ Quick Start (Recommended)

Launch everything with one command:

```bash
source ~/turtlebot3_wall_follower_cpp_ws/install/setup.bash
ros2 launch wall_following_cpp_project wall_follower_gazebo_cpp.launch.py
```

This will:
- 🌍 Start Gazebo with TurtleBot3 world
- 🤖 Spawn robot at position (-2.0, -0.5)
- 🎮 Launch wall following controller
- 🔄 Begin autonomous navigation

### 🔧 Manual Launch

**Terminal 1 - Gazebo:**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2 - Wall Follower:**
```bash
source ~/turtlebot3_wall_follower_cpp_ws/install/setup.bash
ros2 launch wall_following_cpp_project wall_following_cpp.launch.py
```

**Terminal 3 - RViz (optional):**
```bash
rviz2 -d ~/turtlebot3_wall_follower_cpp_ws/src/wall_following_cpp_project/rviz/wall_follower_config.rviz
```

## ⚙️ Configuration

### 🛡️ Safety Parameters (wall_following_params.yaml)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `desired_distance` | **0.8m** | Target distance from wall |
| `wall_min` | **0.5m** | Minimum allowed wall distance |
| `emergency_stop` | **0.25m** | Emergency stop distance |
| `slow_down_dist` | **0.4m** | Start slowing down |
| `wall_lost` | **1.0m** | Lose wall threshold |
| `side_clearance` | **0.3m** | Side obstacle clearance |

### 🎮 Control Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `forward_speed` | **0.15 m/s** | Normal forward speed |
| `search_speed` | **0.1 m/s** | Search mode speed |
| `max_angular_speed` | **0.6 rad/s** | Maximum rotation |
| `kp` | **3.0** | Proportional gain |
| `kd` | **0.5** | Derivative gain |

### 📡 Algorithm Settings

- **Front obstacle check**: 0.55m threshold (line 182 in controller.cpp)
- **Wall detection range**: 0.8m maximum
- **Front scan angle**: -10° to +10° for obstacles
- **Control frequency**: 20Hz
- **P-control gain**: -2.0 (line 195)

## 🎭 Core Algorithm Details

### Simple Proportional Control
```cpp
// Line 195-196 in wall_follower_controller.cpp
double error = side_dist - desired_distance_;
double angular = -2.0 * error;  // Simple P control
```

### Corner Detection
```cpp
// Line 202-208: Use diagonal sensor
if (diag_dist < side_dist * 0.7 && diag_dist < 0.6) {
    // Corner ahead - start turning
    angular = max(angular, 0.3);  // Turn preemptively
}
```

### Safe Distance Check
```cpp
// Line 182: Front obstacle avoidance
if (front_dist < 0.55) {  // 55cm safety threshold
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.5;   // Turn away
}
```

## 🛠️ Troubleshooting

### 🚫 Robot doesn't move
```bash
# Check laser data
ros2 topic echo /scan --once

# Monitor velocity commands  
ros2 topic echo /cmd_vel

# Verify node is running
ros2 node list | grep wall_follower
```

### 💥 Robot still touching walls occasionally
Adjust these values in `wall_follower_controller.cpp`:
- Line 11: `desired_distance_ = 0.9;` (increase from 0.8)
- Line 17: `wall_min_ = 0.55;` (increase from 0.5)  
- Line 182: `if (front_dist < 0.6)` (increase from 0.55)

### 🐌 Wall following not smooth
- Increase `kp` to 3.5 for faster response
- Adjust `desired_distance` for your environment
- Check control rate: `ros2 topic hz /cmd_vel`

## 📡 ROS2 Topics

### 📥 Subscribed
- `/scan` (sensor_msgs/LaserScan): Laser scanner data

### 📤 Published  
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands

### 🔍 Monitoring Commands
```bash
# Watch real-time laser data
ros2 topic echo /scan

# Monitor velocity commands
ros2 topic echo /cmd_vel  

# Check publishing rates
ros2 topic hz /cmd_vel
```

## 👨‍💻 Development

### 🔧 Quick Rebuild
```bash
cd ~/turtlebot3_wall_follower_cpp_ws
colcon build --packages-select wall_following_cpp_project
source install/setup.bash
```

### 🎛️ Fine-tuning Guide

**For zero contact guarantee:**
- Set `desired_distance` to 0.9m
- Set `wall_min` to 0.55m
- Increase front check to 0.6m

**For tighter navigation:**
- Reduce `desired_distance` to 0.7m
- Keep `wall_min` at 0.5m
- Reduce speeds by 20%

## 🦾 Hardware Deployment

For real TurtleBot3:
1. Set `use_sim_time: false` in params
2. Reduce all speeds by 30%
3. Increase safety distances by 0.1m
4. Test in open area first
