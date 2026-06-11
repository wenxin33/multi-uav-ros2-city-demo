#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
generate_gazebo_world.py

Convert ros2_demo_result.json into a Gazebo Harmonic SDF world.

This version includes:
- city buildings
- roads
- start and goal markers
- static visual-only UAV models
- Gazebo path breadcrumbs for each UAV route

UAV models are static visual-only models. Their poses are updated by
scripts/gazebo_uav_path_player.py through Gazebo set_pose service.
"""

from __future__ import annotations

import json
from pathlib import Path
from xml.sax.saxutils import escape


def color_for_building(building_type: str) -> str:
    if building_type == "commercial":
        return "0.20 0.20 0.22 1.0"
    if building_type == "residential":
        return "0.50 0.50 0.52 1.0"
    if building_type == "public":
        return "0.20 0.35 0.70 1.0"
    return "0.45 0.45 0.45 1.0"


def box_model_xml(name: str, center, size, color: str, static: bool = True) -> str:
    cx, cy, cz = center
    sx, sy, sz = size
    static_text = "true" if static else "false"
    return f"""
    <model name="{escape(name)}">
      <static>{static_text}</static>
      <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>
          </geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


def visual_sphere_model_xml(name: str, center, radius: float, color: str, static: bool = True) -> str:
    cx, cy, cz = center
    static_text = "true" if static else "false"
    return f"""
    <model name="{escape(name)}">
      <static>{static_text}</static>
      <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <sphere><radius>{radius:.4f}</radius></sphere>
          </geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


def quadrotor_visual_model_xml(name: str, pose) -> str:
    x, y, z = pose
    return f"""
    <model name="{escape(name)}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="base_link">
        <visual name="body_visual">
          <geometry><box><size>0.8 0.8 0.25</size></box></geometry>
          <material>
            <ambient>1.0 0.45 0.0 1.0</ambient>
            <diffuse>1.0 0.45 0.0 1.0</diffuse>
          </material>
        </visual>
        <visual name="arm_x_visual">
          <pose>0 0 0.05 0 0 0</pose>
          <geometry><box><size>2.2 0.12 0.12</size></box></geometry>
          <material>
            <ambient>1.0 0.45 0.0 1.0</ambient>
            <diffuse>1.0 0.45 0.0 1.0</diffuse>
          </material>
        </visual>
        <visual name="arm_y_visual">
          <pose>0 0 0.06 0 0 1.5708</pose>
          <geometry><box><size>2.2 0.12 0.12</size></box></geometry>
          <material>
            <ambient>1.0 0.45 0.0 1.0</ambient>
            <diffuse>1.0 0.45 0.0 1.0</diffuse>
          </material>
        </visual>
        <visual name="rotor_1_visual">
          <pose>1.1 0 0.12 0 0 0</pose>
          <geometry><cylinder><radius>0.22</radius><length>0.06</length></cylinder></geometry>
          <material><ambient>0.05 0.05 0.05 1.0</ambient><diffuse>0.05 0.05 0.05 1.0</diffuse></material>
        </visual>
        <visual name="rotor_2_visual">
          <pose>-1.1 0 0.12 0 0 0</pose>
          <geometry><cylinder><radius>0.22</radius><length>0.06</length></cylinder></geometry>
          <material><ambient>0.05 0.05 0.05 1.0</ambient><diffuse>0.05 0.05 0.05 1.0</diffuse></material>
        </visual>
        <visual name="rotor_3_visual">
          <pose>0 1.1 0.12 0 0 0</pose>
          <geometry><cylinder><radius>0.22</radius><length>0.06</length></cylinder></geometry>
          <material><ambient>0.05 0.05 0.05 1.0</ambient><diffuse>0.05 0.05 0.05 1.0</diffuse></material>
        </visual>
        <visual name="rotor_4_visual">
          <pose>0 -1.1 0.12 0 0 0</pose>
          <geometry><cylinder><radius>0.22</radius><length>0.06</length></cylinder></geometry>
          <material><ambient>0.05 0.05 0.05 1.0</ambient><diffuse>0.05 0.05 0.05 1.0</diffuse></material>
        </visual>
      </link>
    </model>
"""


def add_path_breadcrumbs(data: dict, stride: int = 2) -> str:
    path_colors = {
        1: "1.0 0.45 0.0 1.0",
        2: "0.0 0.65 1.0 1.0",
        3: "1.0 0.8 0.0 1.0",
        4: "1.0 0.2 0.2 1.0",
        5: "0.8 0.2 1.0 1.0",
    }
    xml = ""
    for uav in data.get("uavs", []):
        uav_id = int(uav.get("id", 0))
        path = uav.get("path", [])
        color = path_colors.get(uav_id, "0.0 1.0 1.0 1.0")
        for j, p in enumerate(path):
            if j % stride != 0 and j != len(path) - 1:
                continue
            xml += visual_sphere_model_xml(
                f"uav_{uav_id}_path_{j}",
                p,
                0.20,
                color,
                static=True,
            )
    return xml


def main() -> None:
    package_dir = Path(__file__).resolve().parents[1]
    json_path = package_dir / "data" / "ros2_demo_result.json"
    world_path = package_dir / "worlds" / "city_uav_demo.sdf"

    if not json_path.exists():
        raise FileNotFoundError(f"Cannot find JSON result file: {json_path}")

    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    map_size = data.get("map_size", [60, 60, 30])
    sx, sy, _ = map_size

    world = """<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="city_uav_demo">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 80 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
"""

    world += box_model_xml("ground_plane", [sx / 2.0, sy / 2.0, -0.05], [sx, sy, 0.1], "0.12 0.12 0.12 1.0", True)

    for r in data.get("roads", []):
        world += box_model_xml(f"road_{r.get('id', 0)}", r["center"], r["size"], "0.02 0.02 0.02 1.0", True)

    for b in data.get("buildings", []):
        world += box_model_xml(f"building_{b.get('id', 0)}", b["center"], b["size"], color_for_building(b.get("type", "unknown")), True)

    for i, s in enumerate(data.get("starts", []), start=1):
        world += visual_sphere_model_xml(f"start_{i}", s, 0.5, "0.0 1.0 0.0 1.0", True)

    goal = data.get("target", [0, 0, 0])
    world += visual_sphere_model_xml("goal", goal, 0.7, "1.0 0.0 0.0 1.0", True)

    # Gazebo-visible route markers.
    world += add_path_breadcrumbs(data, stride=2)

    # UAV visual models at first path points.
    for uav in data.get("uavs", []):
        uav_id = int(uav.get("id", 0))
        path = uav.get("path", [])
        pose = path[0] if path else uav.get("start", [0, 0, 3])
        world += quadrotor_visual_model_xml(f"uav_{uav_id}", pose)

    world += """
  </world>
</sdf>
"""

    world_path.parent.mkdir(parents=True, exist_ok=True)
    world_path.write_text(world, encoding="utf-8")

    print(f"Generated Gazebo world: {world_path}")
    print("Included Gazebo path breadcrumbs for UAV routes.")
    print("UAV models are static visual-only models. Use gazebo_uav_path_player.py to move them.")


if __name__ == "__main__":
    main()
