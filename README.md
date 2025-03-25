
# Unitree B2 Workspace Setup

## 1. Create and Build the ROS 2 Workspace
mkdir -p unitreeb2_ws/src
cd unitreeb2_ws/src
git clone <repository_link>  # Replace <repository_link> with the actual repository URL
cd ..
colcon build && source install/setup.bash

## 2.Launch the Robot Model in Mujoco and RViz with ROS 2 Effort Controller

```bash
ros2 launch b2_description b2_effort.launch.py



## 3. Make the Robot Stand

```bash
ros2 run b2_movement stand

## Demostration Video Link 
https://youtu.be/-WktVGW36x0
