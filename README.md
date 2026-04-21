# 3-DOF Robotic Arm

A ROS2-based robotic arm system capable of autonomous end-effector positioning using inverse kinematics and real-time obstacle avoidance.

## Overview
This project implements a 3-Degree-of-Freedom (3-DOF) robotic arm that can reach user-defined 3D target positions. The system uses inverse kinematics (IK) to compute joint angles and integrates ultrasonic sensor feedback to detect obstacles and replan trajectories for safe navigation.

## Features
- Inverse kinematics for precise joint angle computation  
- Real-time obstacle detection using ultrasonic sensor  
- Dynamic waypoint-based trajectory replanning  
- ROS2-based modular architecture  
- Visualization using RViz and simulation in Gazebo  

## System Architecture
The system is implemented using ROS2 nodes:
- `ui_goal_node` – accepts target position  
- `planner_node` – plans trajectory and handles obstacle logic  
- `ik_node` – computes inverse kinematics  
- `controller_node` – executes joint motion  
- `actuator_bridge` – interfaces with simulation/hardware  

## Folder Structure

src/
├── package_1/
├── package_2/
└── ...


## Requirements
- Ubuntu (recommended)
- ROS2 (Humble or later)
- Python 3

## How to Run
```bash
# Navigate to workspace
cd ~/your_workspace

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash

# Launch the system
ros2 launch <your_package> <launch_file>.launch.py
Applications
Industrial automation
Pick-and-place systems
Service robotics
Educational robotics platforms
Future Improvements
Multi-sensor fusion for better obstacle detection
Higher DOF for increased flexibility
Advanced path planning algorithms
Real-world hardware implementation
Contributors
Ayush M
Bhavanesh A M
CSR VedVikas
Gagan M M
License

This project is for academic purposes.
