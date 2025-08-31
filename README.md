# 🤖 TurtleBot3 Wall Following Robot (C++)

A high-performance C++ implementation of ROS2-based wall following robot for TurtleBot3 with enhanced safety features and collision avoidance. 🚀

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

- **🛡️ Enhanced Safety System**: Multiple collision detection zones with configurable safety margins
- **🎯 Adaptive Wall Following**: PID-controlled wall following with dynamic speed adjustment
- **🔍 Intelligent Search**: Wall detection with stuck prevention and recovery behaviors
- **⚠️ Collision Avoidance**: Emergency stop and escape maneuvers when obstacles detected
- **📊 Real-time Visualization**: Pre-configured RViz setup for monitoring robot behavior
- **⚡ High Performance**: C++ implementation for optimal real-time performance
- **🔧 Modular Design**: Clean separation of concerns with header/source file architecture

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
   # (Copy the files created above to their respective locations)
   
   cd ~/turtlebot3_wall_follower_cpp_ws
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

## 🚀 Usage

### ⚡ Quick Start (Gazebo Simulation)

Launch the complete simulation environment with C++ wall follower:

```bash
source ~/turtlebot3_wall_follower_cpp_ws/install/setup.bash
ros2 launch wall_following_cpp_project wall_follower_gazebo_cpp.launch.py
```

This command will:
- 🌍 Start Gazebo with TurtleBot3 world
- 🤖 Spawn the TurtleBot3 robot
- 🎮 Launch the C++ wall following controller
- 🔄 Begin autonomous wall following behavior

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
| `desired_distance` | 0.6m | Target distance from wall |
| `emergency_stop` | 0.55m | Emergency brake distance |
| `slow_down_dist` | 0.8m | Distance to start slowing down |
| `side_clearance` | 0.4m | Minimum side obstacle clearance |
| `forward_speed` | 0.20 m/s | Normal forward speed |
| `max_angular_speed` | 0.6 rad/s | Maximum turning speed |

### 🎮 Control Parameters

- **🎯 PID Gains**: Kp=1.8, Ki=0.0, Kd=0.7 (optimized for responsive control)
- **📡 Wall Detection Range**: 1.5m maximum wall detection distance
- **🔍 Search Behavior**: Alternating search pattern with stuck detection
- **⚡ Control Frequency**: 20Hz for real-time performance

## 🎭 Behavior Modes

1. **🏃 Wall Following**: Maintains constant distance from detected wall using PID control
2. **🔍 Wall Search**: Rotates to locate nearby walls when none detected
3. **⚠️ Collision Avoidance**: Emergency stop and escape maneuvers
4. **🔄 Recovery**: Stuck detection with aggressive recovery maneuvers

## 🛠️ Troubleshooting

### 🚫 Robot doesn't move
- Check if simulation time is properly set: `use_sim_time: true`
- Verify laser scan topic: `ros2 topic echo /scan`
- Check velocity commands: `ros2 topic echo /cmd_vel`
- Ensure C++ executable is built: `colcon build`

### 💥 Build errors
- Install missing dependencies: `rosdep install --from-paths src --ignore-src -r -y`
- Check C++17 support: `gcc --version` (should be 7.0+)
- Clean and rebuild: `colcon build --cmake-clean-cache`

### 📈 Wall following not smooth
- Adjust PID parameters in `wall_following_params.yaml`
- Modify desired distance from wall
- Check laser scan quality and filtering
- Monitor performance: `ros2 run rqt_graph rqt_graph`

### 🐌 Performance issues
- Build in Release mode: `--cmake-args -DCMAKE_BUILD_TYPE=Release`
- Check CPU usage: `top` or `htop`
- Reduce control frequency if needed
- Use optimized compiler flags

## 📡 ROS2 Topics

### 📥 Subscribed Topics
- `/scan` (sensor_msgs/LaserScan): Laser scan data for wall detection

### 📤 Published Topics
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands for robot movement

## 👨‍💻 Development

### 🔧 Building from Source

```bash
cd ~/turtlebot3_wall_follower_cpp_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 🧪 Running Tests

```bash
colcon test
colcon test-result --verbose
```

### 🎛️ Parameter Tuning

Key parameters for tuning robot behavior:
- 🛡️ Safety distances for different environments
- 🎯 PID gains for smoother or more responsive control
- ⚡ Speed parameters for different robot capabilities
- 📊 Detection thresholds for various wall materials

### 📝 Code Structure

- **Headers (.hpp)**: Class declarations and interfaces in `include/`
- **Implementation (.cpp)**: Core logic and algorithms in `src/`
- **CMake**: Build configuration and dependency management
- **Launch**: Python launch files for different scenarios

## 🦾 Hardware Deployment

To run on real TurtleBot3:

1. **🔧 Set up TurtleBot3**: Follow official TurtleBot3 setup guide
2. **⚙️ Update parameters**: Adjust safety distances for real-world conditions  
3. **🧪 Test gradually**: Start with very conservative parameters
4. **👀 Monitor closely**: Always be ready to emergency stop

## 🔄 Migration from Python

Key differences when migrating from Python version:
- **Build System**: `CMakeLists.txt` instead of `setup.py`
- **Performance**: Significantly faster execution
- **Memory Management**: Manual but more efficient
- **Type Safety**: Compile-time error checking
- **Debugging**: GDB support for advanced debugging
