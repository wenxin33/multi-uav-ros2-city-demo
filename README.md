# Multi-UAV ROS2 City Path Planning Demo

This repository provides a public ROS2/RViz2 visualization demo for 3D multi-UAV path planning in structured urban environments.

## Features

- Structured 3D city environment visualization
- Multi-UAV path visualization in RViz2
- Moving quadrotor-like UAV markers
- Simplified UAV kinematic simulation
- Odometry publishing for each UAV
- Velocity arrows and UAV state text
- Basic quadrotor parameter interface for future energy-model extension

> This is a public demonstration version.  
> The full research implementation, including the DP-based path compression module, complete ViT-guided planner, training pipeline, and full experimental code, is not publicly released.

## Project Structure

```text
multi-uav-ros2-city-demo/
├── public_planner_city_no_dp.py
├── vit_corrt_ros2_city_demo/
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── data/
│   │   └── ros2_demo_result.json
│   └── vit_corrt_ros2_city_demo/
│       ├── __init__.py
│       └── city_result_visualizer_node.py
├── README.md
└── .gitignore
