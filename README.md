
# Husky Waypoint Navigation & ZED Integration

This repository contains the autonomous navigation system for a Husky A200 robot. It integrates ZED X camera data with custom ROS 2 navigation logic.

##  Project Overview
The goal of this package is to provide reliable waypoint navigation using a PD controller. It currently utilizes the ZED X for positional tracking, with future plans to integrate a thermal camera system.

##  Quick Start (Build & Install)
1. **Navigate to workspace:** `cd ~/ros2_ws`
2. **Build package:** `colcon build --packages-select husky_nav_ros2`
3. **Source:** `source install/setup.bash`

##  Standard Operating Procedure (SOP)

### 1. Main Navigation Workflow
**Terminal 1 (Start Camera):**
```bash
ros2 launch zed_camera.launch.py camera_model:=zedx
Terminal 2 (Start Navigator):

Bash

ros2 run husky_nav_ros2 collection_simple

2. ZED Diagnostic Tools
Positional Tracking: ./ZED_Positional_Tracking (located in ~/zed-examples/build/)

Floor Detection: ./ZED_Floor_Plane_Detection (sets origin plane at the floor)

Global Localization: ./ZED_Live_Global_Localization

Note: GNSS/GPS reader is currently commented out in main.cpp until hardware is acquired.

 Key Files
waypoint_navigator_pd.py: Advanced navigator using a PD controller for smooth movement.

navigation_simple.py: Basic waypoint following logic.

manual_waypoint_recorder.py: Utility to record coordinates to waypoints.csv.

robot_math_utils.py: Mathematical helper functions for navigation.

 Future Work
Integration of a FLIR Boson thermal camera system.



---
