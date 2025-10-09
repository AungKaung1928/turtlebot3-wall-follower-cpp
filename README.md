# 🤖 TurtleBot3 Wall Following Robot (C++)

A robust C++ implementation of ROS2-based autonomous wall following robot for TurtleBot3 with intelligent obstacle avoidance and continuous exploration. 🚀

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

- **🛡️ Safe Obstacle Avoidance**: Maintains 0.8m distance from walls and 0.55m from obstacles - zero contact guarantee
- **🌍 Complete Map Exploration**: Actively explores entire environment, moving to new areas when walls are lost
- **🎯 Smooth Wall Following**: PID-controlled tracking with intelligent corner handling
- **🔍 Active Search Mode**: Robot continuously moves forward while searching for walls (never gets stuck)
- **⚠️ Smart Collision Detection**: Wide-angle front scanning (-70° to +70°) detects obstacles early
- **🚦 Dynamic Speed Control**: Automatically adjusts speed based on proximity to obstacles
- **📊 Real-time Visualization**: Pre-configured RViz setup for monitoring robot behavior
- **⚡ High Performance**: C++ implementation for optimal real-time performance at 20Hz
- **🔧 Modular Design**: Clean separation of concerns with header/source file architecture

## 🎯 Key Behaviors

### Wall Following Mode
- Maintains **0.8m distance** from walls using PID control
- Smoothly follows walls around entire environment
- Handles corners and obstacles intelligently

### Obstacle Avoidance
- **Stops at 0.55m** from obstacles (emergency distance)
- **Slows down at 0.85m** (warning distance)
- **Turns away from wall** when obstacle blocks path
- Returns to wall following after clearing obstacle

### Active Exploration
- **Always moves forward** during search (no stationary rotation)
- Detects walls up to **2.5m away**
- Switches direction every 2.5 seconds if no wall found
- Explores new areas instead of repeating same paths

## 📦 Installation

1. **Install TurtleBot3 packages:** 📥
   ```bash
   sudo apt update
   sudo apt install ros-humble-turtlebot3*
   sudo apt install ros-humble-gazebo-ros-pkgs
   ```

2. **Install build tools:** 🔨
   ```bash
   sudo apt install build-essential cmake
   sudo apt install ros-humble-ament-cmake
   ```

3. **Set TurtleBot3 model:** 🤖
   ```bash
   echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
   source ~/.bashrc
   ```

4. **Create and build the workspace:** 🔧
   ```bash
   cd ~/
   mkdir -p turtlebot3_wall_follower_cpp_ws/src
   cd turtlebot3_wall_follower_cpp_ws/src
   
   # Create the project structure
   mkdir -p wall_following_cpp_project
   cd wall_following_cpp_project
   
   # Create directories
   mkdir -p config launch rviz include/wall_following_cpp_project src
   
   # Copy all the files to respective directories
   # - Copy wall_follower_controller.cpp to src/
   # - Copy wall_detector.cpp to src/
   # - Copy pid_controller.cpp to src/
   # - Copy main.cpp to src/
   # - Copy *.hpp files to include/wall_following_cpp_project/
   # - Copy *.yaml files to config/
   # - Copy *.launch.py files to launch/
   # - Copy *.rviz file to rviz/
   # - Copy package.xml and CMakeLists.txt to project root
   
   cd ~/turtlebot3_wall_follower_cpp_ws
   colcon build --packages-select wall_following_cpp_project --symlink-install
   source install/setup.bash
   ```

## 🚀 Usage

### ⚡ Quick Start (Recommended)

Launch the complete simulation environment:

```bash
source ~/turtlebot3_wall_follower_cpp_ws/install/setup.bash
ros2 launch wall_following_cpp_project wall_follower_gazebo_cpp.launch.py
```

This command will:
- 🌍 Start Gazebo with TurtleBot3 world
- 🤖 Spawn the TurtleBot3 robot at position (-2.0, -0.5)
- 🎮 Launch the C++ wall following controller
- 🔄 Begin autonomous exploration and wall following

**Watch the robot:**
- Follow walls maintaining safe 0.8m distance
- Avoid all obstacles without contact
- Explore the entire Gazebo world
- Never get stuck in one area

### 🔧 Manual Launch (Step by Step)

1. **Start Gazebo simulation:** 🌍
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. **Launch C++ wall follower (in new terminal):** 🤖
   ```bash
   source ~/turtlebot3_wall_follower_cpp_ws/install/setup.bash
   ros2 launch wall_following_cpp_project wall_following_cpp.launch.py
   ```

3. **Start RViz for visualization (optional):** 📊
   ```bash
   rviz2 -d ~/turtlebot3_wall_follower_cpp_ws/src/wall_following_cpp_project/rviz/wall_follower_config.rviz
   ```

### 🐛 Debug Mode

Run with debug logging:
```bash
ros2 launch wall_following_cpp_project wall_following_cpp.launch.py log_level:=debug
```

## ⚙️ Configuration

### 🛡️ Safety Parameters (wall_following_params.yaml)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `desired_distance` | 0.8m | Target distance from wall (safe spacing) |
| `emergency_stop` | 0.55m | Stop distance from obstacles |
| `slow_down_dist` | 0.85m | Distance to start slowing down |
| `wall_min` | 0.50m | Minimum allowed distance to wall |
| `wall_lost` | 2.5m | Max distance before losing wall |
| `side_clearance` | 0.45m | Minimum side obstacle clearance |

### 🎮 Control Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `forward_speed` | 0.22 m/s | Normal forward speed |
| `search_speed` | 0.18 m/s | Speed while searching for walls |
| `max_angular_speed` | 0.8 rad/s | Maximum turning speed |
| `kp` | 2.0 | PID proportional gain |
| `kd` | 0.5 | PID derivative gain |

### 📡 Detection Parameters

- **Wall Detection Range**: 2.5m maximum
- **Front Scanning**: -70° to +70° (wide angle for early obstacle detection)
- **Wall Consistency**: 0.4m maximum variation
- **Control Frequency**: 20Hz for real-time response

## 🎭 Robot Behavior States

### 1. 🏃 Wall Following
- **When**: Wall detected within 2.5m
- **Action**: Maintains 0.8m distance using PID control
- **Speed**: 0.22 m/s (adjusts based on obstacles)
- **Priority**: High - continues unless obstacle blocks path

### 2. 🔍 Active Search
- **When**: No wall detected nearby
- **Action**: Moves forward while rotating to find walls
- **Speed**: 0.18 m/s continuous forward motion
- **Behavior**: Changes direction every 2.5 seconds

### 3. ⚠️ Obstacle Avoidance
- **When**: Obstacle detected < 0.55m ahead
- **Action**: Stops and turns away from wall
- **Recovery**: Returns to wall following after clearing

### 4. 🚨 Emergency Stop
- **When**: Immediate collision risk
- **Action**: Full stop, evaluate escape directions
- **Priority**: Highest - overrides all other behaviors

## 🛠️ Troubleshooting

### 🚫 Robot doesn't move
- Check simulation time: `ros2 param get /wall_follower_controller_cpp use_sim_time` (should be `true`)
- Verify laser data: `ros2 topic echo /scan --once`
- Check controller is running: `ros2 node list | grep wall_follower`
- Monitor velocity commands: `ros2 topic echo /cmd_vel`

### 🔄 Robot stuck in one area
- **This should not happen with current implementation!**
- If it does, check: `wall_lost` parameter (should be 2.5m)
- Verify search behavior is active (robot should always move forward)
- Check laser scanner range: `ros2 topic echo /scan | grep range_max`

### 💥 Robot hits obstacles
- Increase `emergency_stop` distance (current: 0.55m)
- Increase `desired_distance` from wall (current: 0.8m)
- Check laser scan coverage: front should cover -70° to +70°
- Reduce `forward_speed` for more reaction time

### 🐌 Wall following not smooth
- Tune PID gains in `wall_following_params.yaml`:
  - Increase `kp` for faster response (current: 2.0)
  - Increase `kd` for smoother motion (current: 0.5)
- Adjust `desired_distance` for your environment
- Check control frequency is 20Hz: `ros2 topic hz /cmd_vel`

### 🏗️ Build errors
- Install dependencies: `rosdep install --from-paths src --ignore-src -r -y`
- Check C++17 support: `gcc --version` (needs 7.0+)
- Clean rebuild: `rm -rf build install log && colcon build`
- Verify CMakeLists.txt has correct dependencies

## 📡 ROS2 Topics

### 📥 Subscribed Topics
- `/scan` (sensor_msgs/LaserScan): Laser scan data for wall and obstacle detection

### 📤 Published Topics
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands for robot movement

### 🔍 Useful Monitoring Commands
```bash
# Watch laser scan
ros2 topic echo /scan

# Monitor velocity commands
ros2 topic echo /cmd_vel

# Check topic rates
ros2 topic hz /scan
ros2 topic hz /cmd_vel

# View computation graph
ros2 run rqt_graph rqt_graph
```

## 👨‍💻 Development

### 🔧 Building from Source

```bash
cd ~/turtlebot3_wall_follower_cpp_ws
colcon build --packages-select wall_following_cpp_project --symlink-install
source install/setup.bash
```

### 🎛️ Parameter Tuning Guide

**For tighter spaces:**
- Decrease `desired_distance` to 0.6m
- Decrease `forward_speed` to 0.18 m/s
- Increase `kp` to 2.5 for quicker response

**For faster exploration:**
- Increase `forward_speed` to 0.25 m/s
- Increase `search_speed` to 0.20 m/s
- Decrease `wall_lost` to 2.0m (find walls sooner)

**For more cautious behavior:**
- Increase `emergency_stop` to 0.65m
- Increase `desired_distance` to 1.0m
- Decrease all speeds by 20%

### 📝 Code Structure

```
wall_follower_controller.cpp (400 lines)
├── Initialization & ROS2 setup
├── Sensor data processing (laser scan)
├── Wall following logic (PID control)
├── Active search behavior
├── Obstacle avoidance
└── Main control loop (20Hz)
```

**Key Functions:**
- `wallFollow()`: Main wall following with PID
- `search()`: Active exploration mode
- `checkCollision()`: Wide-angle obstacle detection
- `escapeCollision()`: Emergency avoidance maneuvers

## 🦾 Hardware Deployment

To run on real TurtleBot3:

1. **🔧 Set up TurtleBot3**: Follow [official setup guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

2. **⚙️ Update parameters for real hardware**:
   ```yaml
   use_sim_time: false
   forward_speed: 0.15  # Slower for real robot
   emergency_stop: 0.60  # More conservative
   desired_distance: 0.9  # More spacing
   ```

3. **🧪 Test gradually**:
   - Start in open area
   - Test wall following first
   - Then test obstacle avoidance
   - Finally test full exploration

4. **👀 Safety first**:
   - Always supervise robot
   - Keep emergency stop ready
   - Start with very low speeds
   - Test in controlled environment

## 🎓 Learning Resources

- [ROS2 C++ Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
- [TurtleBot3 Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [PID Control Theory](https://en.wikipedia.org/wiki/PID_controller)
- [Laser Scan Message Format](https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html)
