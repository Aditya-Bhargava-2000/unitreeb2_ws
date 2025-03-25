# Unitree B2 Workspace Setup

This guide outlines the steps to set up a ROS 2 workspace for the Unitree B2 robot, including building the workspace, launching the robot model in Mujoco and RViz, and making the robot stand.

## 1. Create and Build the ROS 2 Workspace

Follow these steps to create and build your ROS 2 workspace:

1.  **Create the workspace directory:**

    ```bash
    mkdir -p unitreeb2_ws/src
    cd unitreeb2_ws/src
    ```

2.  **Clone the robot's repository:**

    ```bash
    git clone https://github.com/Aditya-Bhargava-2000/unitreeb2_ws  
    ```



3.  **Navigate to the workspace root and build:**

    ```bash
    cd ..
    colcon build && source install/setup.bash
    ```

## 2. Launch the Robot Model in Mujoco and RViz with ROS 2 Effort Controller

To visualize and control the Unitree B2 robot model, launch the following ROS 2 launch file:

```bash
ros2 launch b2_description b2_effort.launch.py
 ```

## 3. Make the Robot Stand
```bash
ros2 run b2_movement stand
```


### [Demonstration Video](https://youtu.be/-WktVGW36x0)

### [Documentation File](https://drive.google.com/file/d/1vDkcM6nEfjZHYOceYiUBx_qPhPUSU7BP/view?usp=sharing) 
