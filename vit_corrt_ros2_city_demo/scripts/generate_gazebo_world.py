#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
generate_gazebo_world.py

Enhanced semi-realistic Gazebo Harmonic city world generator.

Input:
    vit_corrt_ros2_city_demo/data/ros2_demo_result.json

Output:
    vit_corrt_ros2_city_demo/worlds/city_uav_demo.sdf

Compared with the basic version, this enhanced generator adds:
    - road surfaces
    - sidewalks
    - lane markings
    - crosswalks near intersections
    - building roofs
    - limited window markers
    - parks / green zones
    - trees
    - streetlights
    - UAV path breadcrumbs
    - static visual-only quadrotor UAV models

Important:
    UAV models are generated as static visual-only models.
    Their poses are updated by gazebo_uav_path_player.py through Gazebo set_pose.
    This avoids gravity-induced vertical jitter.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Iterable, List, Tuple
from xml.sax.saxutils import escape


Color = str
Vec3 = Tuple[float, float, float]


# =========================
# Basic XML helper functions
# =========================

def material_xml(color: Color) -> str:
    return f"""
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
          </material>
"""


def visual_box_model_xml(
    name: str,
    center,
    size,
    color: Color,
    static: bool = True,
    yaw: float = 0.0,
) -> str:
    """Visual-only box model. Used for decorative elements."""
    cx, cy, cz = [float(v) for v in center]
    sx, sy, sz = [float(v) for v in size]
    static_text = "true" if static else "false"

    return f"""
    <model name="{escape(name)}">
      <static>{static_text}</static>
      <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 {yaw:.4f}</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>{sx:.4f} {sy:.4f} {sz:.4f}</size>
            </box>
          </geometry>
{material_xml(color)}
        </visual>
      </link>
    </model>
"""


def collision_box_model_xml(
    name: str,
    center,
    size,
    color: Color,
    static: bool = True,
    yaw: float = 0.0,
) -> str:
    """Box model with collision. Used for buildings, roads, and ground."""
    cx, cy, cz = [float(v) for v in center]
    sx, sy, sz = [float(v) for v in size]
    static_text = "true" if static else "false"

    return f"""
    <model name="{escape(name)}">
      <static>{static_text}</static>
      <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 {yaw:.4f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{sx:.4f} {sy:.4f} {sz:.4f}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{sx:.4f} {sy:.4f} {sz:.4f}</size>
            </box>
          </geometry>
{material_xml(color)}
        </visual>
      </link>
    </model>
"""


def visual_sphere_model_xml(
    name: str,
    center,
    radius: float,
    color: Color,
    static: bool = True,
) -> str:
    cx, cy, cz = [float(v) for v in center]
    static_text = "true" if static else "false"

    return f"""
    <model name="{escape(name)}">
      <static>{static_text}</static>
      <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>{float(radius):.4f}</radius>
            </sphere>
          </geometry>
{material_xml(color)}
        </visual>
      </link>
    </model>
"""


def visual_cylinder_model_xml(
    name: str,
    center,
    radius: float,
    length: float,
    color: Color,
    static: bool = True,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
) -> str:
    cx, cy, cz = [float(v) for v in center]
    static_text = "true" if static else "false"

    return f"""
    <model name="{escape(name)}">
      <static>{static_text}</static>
      <pose>{cx:.4f} {cy:.4f} {cz:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{float(radius):.4f}</radius>
              <length>{float(length):.4f}</length>
            </cylinder>
          </geometry>
{material_xml(color)}
        </visual>
      </link>
    </model>
"""


# =====================
# Semantic color helpers
# =====================

def color_for_building(building_type: str) -> Color:
    if building_type == "commercial":
        return "0.18 0.22 0.30 1.0"   # dark blue-gray
    if building_type == "residential":
        return "0.58 0.56 0.52 1.0"   # warm gray
    if building_type == "public":
        return "0.26 0.40 0.68 1.0"   # blue
    return "0.45 0.45 0.45 1.0"


def roof_color_for_building(building_type: str) -> Color:
    if building_type == "commercial":
        return "0.10 0.12 0.16 1.0"
    if building_type == "residential":
        return "0.35 0.32 0.28 1.0"
    if building_type == "public":
        return "0.16 0.25 0.42 1.0"
    return "0.30 0.30 0.30 1.0"


def window_color_for_building(building_type: str) -> Color:
    if building_type == "commercial":
        return "0.20 0.65 0.95 1.0"
    if building_type == "residential":
        return "0.75 0.86 0.95 1.0"
    if building_type == "public":
        return "0.42 0.76 0.95 1.0"
    return "0.70 0.85 0.95 1.0"


# ====================
# Urban scene generator
# ====================

def enhanced_building_xml(building: dict, max_windows_per_facade: int = 18) -> str:
    """Generate building body + roof + limited windows."""
    bid = int(building.get("id", 0))
    btype = building.get("type", "unknown")
    cx, cy, cz = [float(v) for v in building["center"]]
    sx, sy, sz = [float(v) for v in building["size"]]

    xml = ""
    body_color = color_for_building(btype)
    roof_color = roof_color_for_building(btype)
    window_color = window_color_for_building(btype)

    # Main body with collision.
    xml += collision_box_model_xml(
        f"building_{bid}_body",
        [cx, cy, cz],
        [sx, sy, sz],
        body_color,
        static=True,
    )

    # Slightly larger flat roof.
    roof_thickness = 0.18
    xml += collision_box_model_xml(
        f"building_{bid}_roof",
        [cx, cy, sz + roof_thickness / 2.0],
        [sx + 0.25, sy + 0.25, roof_thickness],
        roof_color,
        static=True,
    )

    # Commercial buildings get a small rooftop equipment block.
    if btype == "commercial" and sz > 12:
        xml += visual_box_model_xml(
            f"building_{bid}_roof_equipment",
            [cx, cy, sz + 0.45],
            [max(0.8, sx * 0.22), max(0.8, sy * 0.22), 0.45],
            "0.12 0.12 0.12 1.0",
            static=True,
        )

    # Limited windows on four facades. Visual-only, no collision.
    floor_step = 2.4
    start_z = 2.0
    z_values = []
    z = start_z
    while z < sz - 1.0 and len(z_values) < 7:
        z_values.append(z)
        z += floor_step

    # Window distribution: avoid excessive SDF models.
    nx = max(1, min(4, int(sx // 2)))
    ny = max(1, min(4, int(sy // 2)))

    count = 0

    # Front/back facades along x direction, y constant.
    for zi in z_values:
        for ix in range(nx):
            if count >= max_windows_per_facade:
                break
            wx = cx - sx * 0.35 + (ix + 0.5) * (sx * 0.70 / nx)
            # front
            xml += visual_box_model_xml(
                f"building_{bid}_window_front_{count}",
                [wx, cy - sy / 2.0 - 0.012, zi],
                [0.55, 0.035, 0.55],
                window_color,
                static=True,
            )
            count += 1
        if count >= max_windows_per_facade:
            break

    count = 0
    for zi in z_values:
        for ix in range(nx):
            if count >= max_windows_per_facade:
                break
            wx = cx - sx * 0.35 + (ix + 0.5) * (sx * 0.70 / nx)
            # back
            xml += visual_box_model_xml(
                f"building_{bid}_window_back_{count}",
                [wx, cy + sy / 2.0 + 0.012, zi],
                [0.55, 0.035, 0.55],
                window_color,
                static=True,
            )
            count += 1
        if count >= max_windows_per_facade:
            break

    count = 0
    for zi in z_values[:5]:
        for iy in range(ny):
            if count >= max_windows_per_facade // 2:
                break
            wy = cy - sy * 0.35 + (iy + 0.5) * (sy * 0.70 / ny)
            # left facade
            xml += visual_box_model_xml(
                f"building_{bid}_window_left_{count}",
                [cx - sx / 2.0 - 0.012, wy, zi],
                [0.035, 0.55, 0.55],
                window_color,
                static=True,
            )
            count += 1

    count = 0
    for zi in z_values[:5]:
        for iy in range(ny):
            if count >= max_windows_per_facade // 2:
                break
            wy = cy - sy * 0.35 + (iy + 0.5) * (sy * 0.70 / ny)
            # right facade
            xml += visual_box_model_xml(
                f"building_{bid}_window_right_{count}",
                [cx + sx / 2.0 + 0.012, wy, zi],
                [0.035, 0.55, 0.55],
                window_color,
                static=True,
            )
            count += 1

    return xml


def is_horizontal_road(size: List[float]) -> bool:
    sx, sy, _ = [float(v) for v in size]
    return sx >= sy


def road_and_marking_xml(road: dict) -> str:
    rid = int(road.get("id", 0))
    cx, cy, cz = [float(v) for v in road["center"]]
    sx, sy, sz = [float(v) for v in road["size"]]
    xml = ""

    road_top_z = 0.04
    line_z = 0.095

    # Road surface.
    xml += collision_box_model_xml(
        f"road_{rid}_surface",
        [cx, cy, road_top_z],
        [sx, sy, 0.04],
        "0.025 0.025 0.025 1.0",
        static=True,
    )

    # Sidewalks.
    sidewalk_color = "0.56 0.56 0.54 1.0"
    sidewalk_width = 1.2

    if is_horizontal_road([sx, sy, sz]):
        # Road along X.
        xml += visual_box_model_xml(
            f"road_{rid}_sidewalk_left",
            [cx, cy - sy / 2.0 - sidewalk_width / 2.0, 0.08],
            [sx, sidewalk_width, 0.08],
            sidewalk_color,
            static=True,
        )
        xml += visual_box_model_xml(
            f"road_{rid}_sidewalk_right",
            [cx, cy + sy / 2.0 + sidewalk_width / 2.0, 0.08],
            [sx, sidewalk_width, 0.08],
            sidewalk_color,
            static=True,
        )

        # Center yellow line.
        xml += visual_box_model_xml(
            f"road_{rid}_center_line",
            [cx, cy, line_z],
            [sx * 0.92, 0.08, 0.025],
            "1.0 0.82 0.0 1.0",
            static=True,
        )

        # Edge white lines.
        xml += visual_box_model_xml(
            f"road_{rid}_white_line_1",
            [cx, cy - sy * 0.28, line_z],
            [sx * 0.92, 0.055, 0.025],
            "0.95 0.95 0.95 1.0",
            static=True,
        )
        xml += visual_box_model_xml(
            f"road_{rid}_white_line_2",
            [cx, cy + sy * 0.28, line_z],
            [sx * 0.92, 0.055, 0.025],
            "0.95 0.95 0.95 1.0",
            static=True,
        )
    else:
        # Road along Y.
        xml += visual_box_model_xml(
            f"road_{rid}_sidewalk_left",
            [cx - sx / 2.0 - sidewalk_width / 2.0, cy, 0.08],
            [sidewalk_width, sy, 0.08],
            sidewalk_color,
            static=True,
        )
        xml += visual_box_model_xml(
            f"road_{rid}_sidewalk_right",
            [cx + sx / 2.0 + sidewalk_width / 2.0, cy, 0.08],
            [sidewalk_width, sy, 0.08],
            sidewalk_color,
            static=True,
        )

        # Center yellow line.
        xml += visual_box_model_xml(
            f"road_{rid}_center_line",
            [cx, cy, line_z],
            [0.08, sy * 0.92, 0.025],
            "1.0 0.82 0.0 1.0",
            static=True,
        )

        # Edge white lines.
        xml += visual_box_model_xml(
            f"road_{rid}_white_line_1",
            [cx - sx * 0.28, cy, line_z],
            [0.055, sy * 0.92, 0.025],
            "0.95 0.95 0.95 1.0",
            static=True,
        )
        xml += visual_box_model_xml(
            f"road_{rid}_white_line_2",
            [cx + sx * 0.28, cy, line_z],
            [0.055, sy * 0.92, 0.025],
            "0.95 0.95 0.95 1.0",
            static=True,
        )

    return xml


def crosswalks_xml(roads: List[dict]) -> str:
    """Generate simple white crosswalk stripes at road intersections."""
    horizontal = []
    vertical = []

    for r in roads:
        if is_horizontal_road(r["size"]):
            horizontal.append(r)
        else:
            vertical.append(r)

    xml = ""
    crosswalk_id = 0

    for hr in horizontal:
        hcx, hcy, _ = [float(v) for v in hr["center"]]
        hsx, hsy, _ = [float(v) for v in hr["size"]]

        for vr in vertical:
            vcx, vcy, _ = [float(v) for v in vr["center"]]
            vsx, vsy, _ = [float(v) for v in vr["size"]]

            # Check if road rectangles overlap.
            if abs(vcx - hcx) <= hsx / 2.0 and abs(hcy - vcy) <= vsy / 2.0:
                ix = vcx
                iy = hcy

                # Two groups of stripes.
                for k in range(4):
                    offset = -1.2 + k * 0.8
                    xml += visual_box_model_xml(
                        f"crosswalk_{crosswalk_id}_h_{k}",
                        [ix + offset, iy - hsy * 0.45, 0.12],
                        [0.35, 1.2, 0.025],
                        "0.95 0.95 0.95 1.0",
                        static=True,
                    )
                    xml += visual_box_model_xml(
                        f"crosswalk_{crosswalk_id}_v_{k}",
                        [ix - vsx * 0.45, iy + offset, 0.12],
                        [1.2, 0.35, 0.025],
                        "0.95 0.95 0.95 1.0",
                        static=True,
                    )
                crosswalk_id += 1

    return xml


def trees_and_lights_xml(data: dict, max_trees: int = 28, max_lights: int = 30) -> str:
    """Add a limited number of trees and streetlights around roads."""
    roads = data.get("roads", [])
    xml = ""
    tree_id = 0
    light_id = 0

    # Add park / green zone first.
    map_size = data.get("map_size", [60, 60, 30])
    sx, sy, _ = [float(v) for v in map_size]
    park_center = [sx * 0.78, sy * 0.22, 0.07]
    park_size = [max(6.0, sx * 0.16), max(6.0, sy * 0.14), 0.08]
    xml += visual_box_model_xml(
        "urban_park_green_zone",
        park_center,
        park_size,
        "0.05 0.38 0.08 1.0",
        static=True,
    )

    # Trees inside park.
    px, py, _ = park_center
    psx, psy, _ = park_size
    for ix in range(3):
        for iy in range(3):
            if tree_id >= max_trees:
                break
            tx = px - psx * 0.30 + ix * psx * 0.30
            ty = py - psy * 0.30 + iy * psy * 0.30
            xml += tree_xml(tree_id, tx, ty)
            tree_id += 1

    # Trees and streetlights along selected roads.
    for r in roads:
        if light_id >= max_lights and tree_id >= max_trees:
            break

        cx, cy, _ = [float(v) for v in r["center"]]
        rsx, rsy, _ = [float(v) for v in r["size"]]

        if is_horizontal_road(r["size"]):
            positions = [cx - rsx * 0.35, cx, cx + rsx * 0.35]
            for x in positions:
                if light_id < max_lights:
                    xml += streetlight_xml(light_id, x, cy + rsy / 2.0 + 1.6)
                    light_id += 1
                if tree_id < max_trees:
                    xml += tree_xml(tree_id, x, cy - rsy / 2.0 - 2.0)
                    tree_id += 1
        else:
            positions = [cy - rsy * 0.35, cy, cy + rsy * 0.35]
            for y in positions:
                if light_id < max_lights:
                    xml += streetlight_xml(light_id, cx + rsx / 2.0 + 1.6, y)
                    light_id += 1
                if tree_id < max_trees:
                    xml += tree_xml(tree_id, cx - rsx / 2.0 - 2.0, y)
                    tree_id += 1

    return xml


def tree_xml(tree_id: int, x: float, y: float) -> str:
    xml = ""
    xml += visual_cylinder_model_xml(
        f"tree_{tree_id}_trunk",
        [x, y, 0.75],
        0.12,
        1.5,
        "0.32 0.18 0.08 1.0",
        static=True,
    )
    xml += visual_sphere_model_xml(
        f"tree_{tree_id}_crown",
        [x, y, 1.85],
        0.75,
        "0.05 0.42 0.10 1.0",
        static=True,
    )
    return xml


def streetlight_xml(light_id: int, x: float, y: float) -> str:
    xml = ""
    xml += visual_cylinder_model_xml(
        f"streetlight_{light_id}_pole",
        [x, y, 1.6],
        0.06,
        3.2,
        "0.18 0.18 0.18 1.0",
        static=True,
    )
    xml += visual_sphere_model_xml(
        f"streetlight_{light_id}_lamp",
        [x, y, 3.25],
        0.20,
        "1.0 0.85 0.45 1.0",
        static=True,
    )
    return xml


def quadrotor_visual_model_xml(name: str, pose) -> str:
    """Generate a simplified static visual-only quadrotor model."""
    x, y, z = [float(v) for v in pose]

    return f"""
    <model name="{escape(name)}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>

      <link name="base_link">
        <visual name="body_visual">
          <geometry>
            <box>
              <size>0.8 0.8 0.25</size>
            </box>
          </geometry>
{material_xml("1.0 0.45 0.0 1.0")}
        </visual>

        <visual name="arm_x_visual">
          <pose>0 0 0.05 0 0 0</pose>
          <geometry>
            <box>
              <size>2.2 0.12 0.12</size>
            </box>
          </geometry>
{material_xml("1.0 0.45 0.0 1.0")}
        </visual>

        <visual name="arm_y_visual">
          <pose>0 0 0.06 0 0 1.5708</pose>
          <geometry>
            <box>
              <size>2.2 0.12 0.12</size>
            </box>
          </geometry>
{material_xml("1.0 0.45 0.0 1.0")}
        </visual>

        <visual name="rotor_1_visual">
          <pose>1.1 0 0.12 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.22</radius>
              <length>0.06</length>
            </cylinder>
          </geometry>
{material_xml("0.05 0.05 0.05 1.0")}
        </visual>

        <visual name="rotor_2_visual">
          <pose>-1.1 0 0.12 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.22</radius>
              <length>0.06</length>
            </cylinder>
          </geometry>
{material_xml("0.05 0.05 0.05 1.0")}
        </visual>

        <visual name="rotor_3_visual">
          <pose>0 1.1 0.12 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.22</radius>
              <length>0.06</length>
            </cylinder>
          </geometry>
{material_xml("0.05 0.05 0.05 1.0")}
        </visual>

        <visual name="rotor_4_visual">
          <pose>0 -1.1 0.12 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.22</radius>
              <length>0.06</length>
            </cylinder>
          </geometry>
{material_xml("0.05 0.05 0.05 1.0")}
        </visual>
      </link>
    </model>
"""


def add_path_breadcrumbs(data: dict, stride: int = 2) -> str:
    """Add Gazebo-visible planned routes as small static sphere breadcrumbs."""
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
    sx, sy, _ = [float(v) for v in map_size]

    world = """<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="city_uav_demo">

    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>

    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>

    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 85 0 0 0</pose>
      <diffuse>0.85 0.85 0.78 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
      <direction>-0.45 0.15 -0.88</direction>
    </light>
"""

    # Ground plane.
    world += collision_box_model_xml(
        "ground_plane",
        [sx / 2.0, sy / 2.0, -0.05],
        [sx, sy, 0.1],
        "0.18 0.18 0.18 1.0",
        static=True,
    )

    # Roads, sidewalks, lane markings.
    for r in data.get("roads", []):
        world += road_and_marking_xml(r)

    # Crosswalks at intersections.
    world += crosswalks_xml(data.get("roads", []))

    # Enhanced buildings.
    for b in data.get("buildings", []):
        world += enhanced_building_xml(b)

    # Park, trees, lights.
    world += trees_and_lights_xml(data)

    # Start markers.
    for i, s in enumerate(data.get("starts", []), start=1):
        world += visual_sphere_model_xml(
            f"start_{i}",
            s,
            0.5,
            "0.0 1.0 0.0 1.0",
            static=True,
        )

    # Goal marker.
    goal = data.get("target", [0, 0, 0])
    world += visual_sphere_model_xml(
        "goal",
        goal,
        0.7,
        "1.0 0.0 0.0 1.0",
        static=True,
    )

    # Planned route breadcrumbs.
    world += add_path_breadcrumbs(data, stride=2)

    # UAV models.
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

    print(f"Generated enhanced Gazebo world: {world_path}")
    print("Included roads, sidewalks, lane markings, crosswalks, roofs, windows, trees, streetlights, and UAV path breadcrumbs.")
    print("UAV models are static visual-only models. Use gazebo_uav_path_player.py to move them.")


if __name__ == "__main__":
    main()
