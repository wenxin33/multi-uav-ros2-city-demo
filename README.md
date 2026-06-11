# Multi-UAV ROS2 City Path Planning Demo

This repository provides a public ROS2/RViz2 visualization demo for 3D multi-UAV path planning in structured urban environments.

The demo integrates a simplified 3D city scene, multi-UAV path results, quadrotor-like UAV markers, simplified UAV kinematics, Odometry publishing, velocity arrows, and state text visualization in RViz2.

> This is a public demonstration version.
> The full research implementation, including the complete ViT-guided planner, DP-based path compression module, training pipeline, and full experimental code, is not publicly released.

---

## 1. Project Overview

This project is derived from my research on multi-UAV path planning in 3D urban environments.

The main goal of this public demo is to show how a path planning result can be connected to a ROS2/RViz2 visualization pipeline.

The overall workflow is:

```text
Structured city generation
        ↓
Multi-UAV path result export
        ↓
JSON result file
        ↓
ROS2 visualization node
        ↓
RViz2 city scene, UAV paths, UAV entities, kinematic states
```

The current version focuses on engineering demonstration:

* 3D city environment visualization
* Multi-UAV path visualization
* Quadrotor-like UAV marker display
* Simplified UAV kinematic simulation
* Odometry publishing
* Velocity arrow visualization
* UAV state text visualization
* Interface preparation for future UAV energy modeling

---

## 2. Main Features

### 2.1 Structured 3D City Environment

The demo uses a structured city environment containing:

* Roads
* Buildings
* High-rise blocks
* Start positions
* A shared goal position
* Multiple UAV paths

The environment is visualized in RViz2 through `visualization_msgs/MarkerArray`.

Published map topics include:

```text
/map/buildings
/map/roads
/map/boundary
/map/start_goal_markers
```

---

### 2.2 Multi-UAV Path Visualization

Each UAV path is published as a ROS2 `nav_msgs/Path` topic:

```text
/uav_1/path
/uav_2/path
/uav_3/path
```

RViz2 can display the planned paths in the 3D city environment.

The path data is loaded from:

```text
vit_corrt_ros2_city_demo/data/ros2_demo_result.json
```

The JSON file contains:

```text
map_size
buildings
roads
starts
target
uavs.path
```

---

### 2.3 Quadrotor-Like UAV Visualization

The UAV entity is represented by a simplified quadrotor-like marker model, including:

* Central body
* Cross-shaped arms
* Four rotor markers
* UAV text label

The current UAV positions are published through:

```text
/uav/current_positions
```

This is implemented with `visualization_msgs/MarkerArray`.

---

### 2.4 Simplified UAV Kinematic Simulation

The demo includes a simplified kinematic simulation layer.

For each UAV path, the system performs arc-length interpolation along the planned path. Given a cruise speed, the UAV state is updated over time.

The position is computed as:

```text
p(t) = interpolate(path, s(t))
s(t) = v_cruise * t
```

The velocity direction is estimated from the current path segment:

```text
v(t) = v_cruise * direction
```

The yaw angle is computed from the horizontal velocity direction:

```text
yaw = atan2(v_y, v_x)
```

This simplified model is suitable for path planning visualization and later energy-estimation extension. It is not a full motor-level quadrotor dynamics simulator.

---

### 2.5 Odometry Publishing

Each UAV publishes a `nav_msgs/Odometry` message:

```text
/uav_1/odom
/uav_2/odom
/uav_3/odom
```

The Odometry message includes:

* Position
* Orientation
* Linear velocity

This provides a basic ROS2-compatible UAV state interface for future modules, such as energy estimation, tracking control, or trajectory analysis.

---

### 2.6 Velocity and State Visualization

The demo also publishes:

```text
/uav/velocity_arrows
/uav/state_text
```

These topics are visualized in RViz2 as:

* Velocity arrows
* UAV speed
* Vertical velocity
* UAV mass

This makes the UAV motion state more explicit than simple path-line visualization.

---

## 3. Quadrotor Parameters

The visualizer includes a basic quadrotor parameter interface:

```text
mass_kg = 1.60
rotor_count = 4
arm_length_m = 0.32
rotor_radius_m = 0.12
max_speed_mps = 8.0
cruise_speed_mps = 3.0
max_accel_mps2 = 3.0
max_climb_rate_mps = 2.0
gravity_mps2 = 9.81
```

The hover thrust is computed as:

```text
T_hover = m * g
```

The per-rotor hover thrust is:

```text
T_rotor = T_hover / 4
```

These parameters are included to support future energy-model development.

---

## 4. Future Energy Model Extension

The current kinematic state provides the variables required for a simplified UAV energy model:

```text
position
velocity
speed
vertical speed
yaw
path distance
mass
hover thrust
```

A future energy-estimation module can be built based on:

```text
E_total = Σ E_k
E_k = P_k * Δt_k
```

where the segment-level power can include:

```text
P_k = P_hover + P_move + P_climb + P_turn + P_wind
```

Possible energy components include:

* Hovering power
* Horizontal cruise power
* Climb and descent energy
* Turning cost
* Wind-relative velocity cost

This repository does not yet implement the full energy model. The current version prepares the ROS2 state interface and visualization foundation for that extension.

---

## 5. Project Structure

```text
multi-uav-ros2-city-demo/
├── public_planner_city_no_dp.py
├── README.md
├── .gitignore
└── vit_corrt_ros2_city_demo/
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/
    │   └── vit_corrt_ros2_city_demo
    ├── data/
    │   └── ros2_demo_result.json
    └── vit_corrt_ros2_city_demo/
        ├── __init__.py
        └── city_result_visualizer_node.py
```

---

## 6. Environment

Tested with:

```text
Ubuntu 24.04
ROS2 Jazzy
Python 3.12
RViz2
```

Required ROS2 packages include:

```text
rclpy
geometry_msgs
nav_msgs
visualization_msgs
std_msgs
```

---

## 7. Generate Planning Result

Run:

```bash
python3 public_planner_city_no_dp.py
```

This generates:

```text
ros2_demo_result.json
```

Copy the result file into the ROS2 package:

```bash
cp ros2_demo_result.json vit_corrt_ros2_city_demo/data/ros2_demo_result.json
```

---

## 8. Build the ROS2 Package

Copy the package into your ROS2 workspace:

```bash
mkdir -p ~/uav_ros2_ws/src
cp -r vit_corrt_ros2_city_demo ~/uav_ros2_ws/src/
cd ~/uav_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Check whether the executable is available:

```bash
ros2 pkg executables vit_corrt_ros2_demo
```

For this package, the expected executable is:

```text
vit_corrt_ros2_city_demo city_result_visualizer
```

---

## 9. Run the Visualizer

Terminal 1:

```bash
cd ~/uav_ros2_ws
source install/setup.bash
ros2 run vit_corrt_ros2_city_demo city_result_visualizer
```

Terminal 2:

```bash
source ~/uav_ros2_ws/install/setup.bash
rviz2
```

In RViz2, set:

```text
Fixed Frame = map
```

Recommended topics to add:

```text
/map/roads
/map/buildings
/map/start_goal_markers
/uav_1/path
/uav_2/path
/uav_3/path
/uav/current_positions
/uav/velocity_arrows
/uav/state_text
```

Optional topics:

```text
/map/boundary
/uav_1/odom
/uav_2/odom
/uav_3/odom
```

---

## 10. Published ROS2 Topics

### Map and Scene

```text
/map/buildings
/map/roads
/map/boundary
/map/start_goal_markers
```

### UAV Paths

```text
/uav_1/path
/uav_2/path
/uav_3/path
```

### UAV Motion Visualization

```text
/uav/current_positions
/uav/velocity_arrows
/uav/state_text
```

### UAV Odometry

```text
/uav_1/odom
/uav_2/odom
/uav_3/odom
```

---

## 11. Run with Custom Cruise Speed

The cruise speed can be changed through a ROS2 parameter:

```bash
ros2 run vit_corrt_ros2_city_demo city_result_visualizer --ros-args -p cruise_speed_mps:=4.0
```

Other parameters include:

```text
uav_timer_period
static_timer_period
```

Example:

```bash
ros2 run vit_corrt_ros2_city_demo city_result_visualizer --ros-args -p cruise_speed_mps:=4.0 -p uav_timer_period:=0.2
```

---

## 12. Notes on RViz2 Performance

If RViz2 becomes slow, add only the essential topics first:

```text
/map/roads
/map/buildings
/uav/current_positions
/uav/velocity_arrows
/uav/state_text
```

Then add path and boundary topics after confirming the scene is stable.

The node publishes static scene elements at a lower frequency and UAV states at a higher frequency to reduce RViz2 rendering load.

---

## 13. Research Background

This demo is derived from my research on multi-UAV path planning in 3D urban environments.

The full research version studies:

* Learning-guided sampling
* RRT-based 3D path planning
* Path compression
* Multi-UAV coordination
* Path quality evaluation
* Planning efficiency analysis

This public repository only provides a simplified visualization and engineering demonstration.

---

## 14. Disclaimer

The following components are not included in this public repository:

* Full ViT-guided planner
* Complete training pipeline
* DP-based path compression module
* Full multi-UAV coordination experiments
* Paper experimental code
* Private research data

This repository is intended for project demonstration, ROS2/RViz2 visualization, and internship portfolio purposes.
