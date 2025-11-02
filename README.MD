# TurtleBot3 Figure-Eight Controller 
---
# Features
- Autonomous figure-eight path follower
- Continous path mapping , always returns to same path if placed anywhere else
- Manual keyboard control mode ( velocity , direction )
- Real-time velocity publishing to /cmd_vel
- Works with TurtleBot3 Waffle and Waffle Pi models
---
# Requirements 
- ROS2 Humble
- Gazebo classic
---
## Getting Started


### 1. Clone this repository
```bash
cd turtlebot3_ws/src
```

### 2. Build the package
```bash
colcon build
```

### 3. Source the workspace

```bash
source install/setup.bash
```

```bash
source /opt/ros/humble/setup.bash
```

### 4. Launch the robot in Gazebo

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### 5. Run our figure_eight_controller

- Open a new terminal: (Keep this terminal window open along with gazebo)

```bash
ros2 run my_turtlebot3_control figure_eight_controller
```
