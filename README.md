# Husky Waypoint Navigation & ZED Integration (SCUBA Lab)

This repository contains the autonomous navigation system for the SCUBA Lab's Husky A200 robot. It integrates ZED X camera data with custom ROS 2 navigation logic for waypoint following and path recording.

##  System Overview

This package utilizes a PD controller for precise waypoint navigation. The ZED X camera provides the primary positional tracking and odometry data.

##  Quick Start (Build & Install)

1. **Navigate to workspace:** `cd ~/ros2_ws`
2. **Build package:** `colcon build --packages-select husky_nav_ros2`
3. **Source:** `source install/setup.bash`

##  Standard Operating Procedure (SOP)

### 1. Main Navigation Workflow

**Terminal 1 (Start Camera):**

```bash
ros2 launch zed_camera.launch.py camera_model:=zedx

```

**Terminal 2 (Start Navigator):**

```bash
ros2 run husky_nav_ros2 collection_simple

```

### 2. ZED Diagnostic Tools

These tools are used to verify camera hardware and environment detection before running full navigation cycles.

* **Positional Tracking:** `./ZED_Positional_Tracking` (Tests 3D movement tracking)
* **Floor Detection:** `./ZED_Floor_Plane_Detection` (Automatically sets the origin plane at floor level)
* **Global Localization:** `./ZED_Live_Global_Localization`
* *Note: GNSS/GPS reader is currently disabled in `main.cpp` until an antenna is acquired.*



##  Key Files

* **`waypoint_navigator_pd.py`**: The primary navigator using a Proportional-Derivative controller for stable movement.
* **`navigation_simple.py`**: A baseline waypoint following script.
* **`manual_waypoint_recorder.py`**: A utility to manually drive the robot and record coordinates to `waypoints.csv`.
* **`robot_math_utils.py`**: Helper functions for distance and heading calculations.

##  Planned Integrations

* Integration of a FLIR Boson thermal camera system for multispectral navigation.
* Implementation of YOLOv8 for enhanced object detection and vision-based tracking.

---



