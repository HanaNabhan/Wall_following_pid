# ROS2 Wall Following with PID Control

## Overview

This repository contains a ROS2-based project that implements wall-following behavior using a PID controller. It is designed for autonomous robot applications, such as those used in the F1TENTH racing environment.

The project enables a robot to follow a wall at a desired distance using LiDAR sensor data, with precise control achieved through a Proportional-Integral-Derivative (PID) controller.

## Requirements

### Software

- **ROS2 Foxy** (or later)
- **Python 3.x**

### ROS2 Dependencies

You will need the following ROS2 packages:  
- `rclpy`: ROS2 Python client library.  
- `sensor_msgs`: For processing sensor inputs like LiDAR.  

Running the Project  
Step 1: Launch the ROS2 Nodes  
Wall Following Node  
To run the wall-following node, which processes LiDAR data and computes the necessary steering commands, use the following command:  

```bash
ros2 launch wall_following_pkg wall_following_pid_launch.py
```

### Customization
PID Tuning
You can adjust the PID gains in the control file,to fine-tune the controller behavior. The following parameters can be modified:  
Kp: Proportional gain.  
Ki: Integral gain.  
Kd: Derivative gain.  
desired_distance: The target distance from the wall to be maintained.  

#Directory Structure  
Here's the typical structure of the repository:  
ros2_wall_following_pid/
│
├── src/
│   ├── wall_following_node.py         # Wall following node (Python)
│   ├── pid_control_node.py            # PID control node (Python)
│
├── launch/
│   ├── wall_following_pid_launch.py   # Launch file to start both nodes
│
│
├── setup.py                  
└── package.xml                         # ROS2 package metadata






