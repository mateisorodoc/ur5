# ROS 2 Universal Robots (UR5) Simulation and Teleoperation via a Virtual Controller

This repository contains the necessary packages to simulate a Universal Robots UR5 manipulator using Gazebo and control its joints via a simple drone-like controller interface (xyz, pitch, roll, yaw) in a ROS 2 Humble environment.

This setup uses the official Universal Robots ROS 2 Description and Gazebo Simulation packages, along with a custom node for teleoperation (`ur5_keyboard_teleop`).

## Prerequisites

This project is built and tested for **ROS 2 Humble**. Make sure you have a working ROS 2 Humble environment configured.

## Quick Start: Build and Run

Follow the steps below to set up your workspace, build the packages, launch the simulation, and start the control interface.

### 1. Source the ROS 2 environment

Open a terminal and source your ROS 2 installation. If you are not using a standard installation path, adjust accordingly.

```bash
source /opt/ros/humble/setup.bash
```

### 2. Build the workspace

Navigate to the root of your workspace (the directory containing the src folder) and use colcon to build all packages.

We use --symlink-install for convenience during development.
```bash
colcon build --symlink-install
```

### 3. Source the local workspace
After a successful build, you need to source the local setup file to make the newly compiled packages and executables available in your terminal session.
```bash
source install/setup.bash
```

### 4. Launch the Simulation (Gazebo & MoveIt)

This command launches the Gazebo simulation environment, loads the UR5 robot model, starts the required controllers, and initializes MoveIt 2 along with RViz visualization. This step may take a few moments to complete.
```bash
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
```

### 5. Start Keyboard Teleoperation

Once the simulation and RViz window are fully loaded, open a new terminal tab (or window), source the ROS 2 and workspace setup files again (steps 1 and 3), and run the teleoperation node:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ur5_keyboard_teleop joint_teleop
```
