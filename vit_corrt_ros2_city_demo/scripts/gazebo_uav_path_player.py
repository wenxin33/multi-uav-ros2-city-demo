#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
gazebo_uav_path_player.py

Move Gazebo UAV models along paths from ros2_demo_result.json.

This script directly calls Gazebo Harmonic service:

    /world/city_uav_demo/set_pose

It is a lightweight visualization-level Gazebo synchronizer.
It does not implement PX4, motor control, or full quadrotor dynamics.

Run after Gazebo world is opened:

    python3 scripts/gazebo_uav_path_player.py
"""

from __future__ import annotations

import json
import math
import subprocess
import time
from pathlib import Path
from typing import Dict, List


WORLD_NAME = "city_uav_demo"
SERVICE_NAME = f"/world/{WORLD_NAME}/set_pose"


CRUISE_SPEED = 3.0      # m/s, visualization speed
DT = 0.2                # update period
Z_OFFSET = 0.0          # increase if UAV appears too close to ground/buildings


def load_json(path: Path) -> Dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def build_path_cache(path: List[List[float]]) -> Dict:
    points = [[float(v) for v in p] for p in path]
    seg_lengths = []
    cum_lengths = [0.0]

    for i in range(len(points) - 1):
        p0 = points[i]
        p1 = points[i + 1]
        d = math.sqrt(
            (p1[0] - p0[0]) ** 2
            + (p1[1] - p0[1]) ** 2
            + (p1[2] - p0[2]) ** 2
        )
        seg_lengths.append(d)
        cum_lengths.append(cum_lengths[-1] + d)

    return {
        "points": points,
        "seg_lengths": seg_lengths,
        "cum_lengths": cum_lengths,
        "total_length": cum_lengths[-1] if cum_lengths else 0.0,
    }


def interpolate_state(cache: Dict, t: float, speed: float) -> Dict:
    points = cache["points"]
    seg_lengths = cache["seg_lengths"]
    cum_lengths = cache["cum_lengths"]
    total_length = cache["total_length"]

    if len(points) == 0:
        return {
            "position": [0.0, 0.0, 3.0],
            "yaw": 0.0,
        }

    if len(points) == 1 or total_length <= 1e-9:
        return {
            "position": points[0],
            "yaw": 0.0,
        }

    distance = (t * speed) % total_length

    seg_idx = 0
    for i in range(len(seg_lengths)):
        if cum_lengths[i] <= distance <= cum_lengths[i + 1]:
            seg_idx = i
            break

    p0 = points[seg_idx]
    p1 = points[seg_idx + 1]
    seg_len = max(seg_lengths[seg_idx], 1e-9)

    alpha = (distance - cum_lengths[seg_idx]) / seg_len
    alpha = max(0.0, min(1.0, alpha))

    x = p0[0] * (1.0 - alpha) + p1[0] * alpha
    y = p0[1] * (1.0 - alpha) + p1[1] * alpha
    z = p0[2] * (1.0 - alpha) + p1[2] * alpha + Z_OFFSET

    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    yaw = math.atan2(dy, dx) if abs(dx) + abs(dy) > 1e-9 else 0.0

    return {
        "position": [x, y, z],
        "yaw": yaw,
    }


def call_gazebo_set_pose(model_name: str, x: float, y: float, z: float, yaw: float) -> None:
    """
    Calls Gazebo set_pose service.

    Pose format:
        name: "uav_1"
        position { x: ... y: ... z: ... }
        orientation { x: 0 y: 0 z: sin(yaw/2) w: cos(yaw/2) }
    """
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)

    req = (
        f'name: "{model_name}" '
        f'position {{ x: {x:.4f} y: {y:.4f} z: {z:.4f} }} '
        f'orientation {{ x: 0.0 y: 0.0 z: {qz:.6f} w: {qw:.6f} }}'
    )

    cmd = [
        "gz",
        "service",
        "-s",
        SERVICE_NAME,
        "--reqtype",
        "gz.msgs.Pose",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        "1000",
        "--req",
        req,
    ]

    subprocess.run(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def main() -> None:
    package_dir = Path(__file__).resolve().parents[1]
    json_path = package_dir / "data" / "ros2_demo_result.json"

    data = load_json(json_path)

    caches = {}
    for uav in data.get("uavs", []):
        uav_id = int(uav["id"])
        model_name = f"uav_{uav_id}"
        path = uav.get("path", [])
        caches[model_name] = build_path_cache(path)

    if not caches:
        raise RuntimeError("No UAV paths found in ros2_demo_result.json")

    print(f"Using Gazebo service: {SERVICE_NAME}")
    print(f"Loaded UAV models: {list(caches.keys())}")
    print("Press Ctrl+C to stop.")

    t = 0.0

    try:
        while True:
            for model_name, cache in caches.items():
                state = interpolate_state(cache, t, CRUISE_SPEED)
                x, y, z = state["position"]
                yaw = state["yaw"]

                call_gazebo_set_pose(model_name, x, y, z, yaw)

            t += DT
            time.sleep(DT)

    except KeyboardInterrupt:
        print("\nStopped Gazebo UAV path player.")


if __name__ == "__main__":
    main()
