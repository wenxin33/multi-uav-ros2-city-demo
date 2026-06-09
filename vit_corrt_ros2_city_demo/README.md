# Structured City Multi-UAV ROS2/RViz2 Demo

This package visualizes a structured 3D city scene and multi-UAV path planning results in RViz2.

It reads:

```text
data/ros2_demo_result.json
```

and publishes:

```text
/map/buildings
/map/roads
/map/boundary
/map/start_goal_markers
/uav_1/path
/uav_2/path
/uav_3/path
```

The full research implementation is not released. This public demo intentionally excludes the DP-based path-compression module.
