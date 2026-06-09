#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
public_planner_city_no_dp.py

Public ROS2/RViz2 demo generator for 3D multi-UAV city path planning.

Purpose:
    1. Generate a structured city scene: buildings + roads + narrow corridors.
    2. Run a simplified learning-guided RRT-style planner without DP compression.
    3. Export ros2_demo_result.json for ROS2/RViz2 visualization.

Important:
    - This public demo intentionally DOES NOT include the DP path-compression module.
    - It does not expose the full research implementation.
    - The planner keeps the sampling-based path planning structure and outputs raw planned paths.

Run:
    python3 public_planner_city_no_dp.py

Output:
    ros2_demo_result.json
"""

from __future__ import annotations

import argparse
import json
import math
import random
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


Point3D = Tuple[float, float, float]
GridPoint = Tuple[int, int, int]


@dataclass
class Building:
    building_id: int
    building_type: str
    center: Point3D
    size: Point3D


@dataclass
class Road:
    road_id: int
    center: Point3D
    size: Point3D


class CityEnvironment3D:
    """
    Structured 3D urban environment.

    The occupancy grid is used for collision checking.
    Building and road objects are exported separately for better RViz2 visualization.
    """

    def __init__(
        self,
        size: Tuple[int, int, int] = (60, 60, 30),
        num_drones: int = 3,
        seed: int = 7,
        building_spacing: int = 9,
        road_width: int = 4,
    ) -> None:
        self.size = size
        self.num_drones = num_drones
        self.seed = seed
        self.building_spacing = building_spacing
        self.road_width = road_width

        random.seed(seed)
        np.random.seed(seed)

        self.grid = np.zeros(size, dtype=np.uint8)
        self.buildings: List[Building] = []
        self.roads: List[Road] = []

        self.starts: List[GridPoint] = []
        self.target: GridPoint = (size[0] - 4, size[1] // 2, 5)

        self._generate_roads()
        self._generate_buildings()
        self._add_bottleneck_corridor()
        self._rasterize_buildings_to_grid()
        self._generate_start_and_goal_points()

    def _generate_roads(self) -> None:
        sx, sy, _ = self.size
        rw = self.road_width

        vertical_roads = [sx // 4, sx // 2, 3 * sx // 4]
        horizontal_roads = [sy // 4, sy // 2, 3 * sy // 4]

        road_id = 0
        for x in vertical_roads:
            self.roads.append(
                Road(
                    road_id=road_id,
                    center=(float(x), float(sy / 2), 0.02),
                    size=(float(rw), float(sy), 0.04),
                )
            )
            road_id += 1

        for y in horizontal_roads:
            self.roads.append(
                Road(
                    road_id=road_id,
                    center=(float(sx / 2), float(y), 0.02),
                    size=(float(sx), float(rw), 0.04),
                )
            )
            road_id += 1

    def _near_road(self, x: int, y: int) -> bool:
        for road in self.roads:
            cx, cy, _ = road.center
            sx, sy, _ = road.size
            if abs(x - cx) <= sx / 2 + 1 and abs(y - cy) <= sy / 2 + 1:
                return True
        return False

    def _generate_buildings(self) -> None:
        sx, sy, sz = self.size
        bid = 0

        for x in range(4, sx - 8, self.building_spacing):
            for y in range(4, sy - 8, self.building_spacing):
                if self._near_road(x, y):
                    continue

                width = random.randint(4, 7)
                depth = random.randint(4, 7)

                building_type = random.choices(
                    ["residential", "commercial", "public"],
                    weights=[0.55, 0.30, 0.15],
                    k=1,
                )[0]

                if building_type == "residential":
                    height = random.randint(7, 15)
                elif building_type == "commercial":
                    height = random.randint(15, min(26, sz - 2))
                else:
                    height = random.randint(5, 12)

                ox = random.randint(-1, 1)
                oy = random.randint(-1, 1)

                cx = x + width / 2 + ox
                cy = y + depth / 2 + oy
                cz = height / 2

                if not self._inside_map_box(cx, cy, width, depth):
                    continue

                self.buildings.append(
                    Building(
                        building_id=bid,
                        building_type=building_type,
                        center=(float(cx), float(cy), float(cz)),
                        size=(float(width), float(depth), float(height)),
                    )
                )
                bid += 1

    def _add_bottleneck_corridor(self) -> None:
        sx, sy, sz = self.size
        y_mid = sy // 2
        corridor_width = 5
        height = min(24, sz - 2)

        bid = len(self.buildings)
        for x in range(sx // 3, sx // 3 + 18, 7):
            self.buildings.append(
                Building(
                    building_id=bid,
                    building_type="commercial",
                    center=(float(x), float(y_mid - corridor_width - 4), float(height / 2)),
                    size=(5.0, 6.0, float(height)),
                )
            )
            bid += 1
            self.buildings.append(
                Building(
                    building_id=bid,
                    building_type="commercial",
                    center=(float(x), float(y_mid + corridor_width + 4), float(height / 2)),
                    size=(5.0, 6.0, float(height)),
                )
            )
            bid += 1

    def _inside_map_box(self, cx: float, cy: float, width: float, depth: float) -> bool:
        sx, sy, _ = self.size
        return (
            0 <= cx - width / 2
            and cx + width / 2 < sx
            and 0 <= cy - depth / 2
            and cy + depth / 2 < sy
        )

    def _rasterize_buildings_to_grid(self) -> None:
        sx, sy, sz = self.size
        self.grid.fill(0)

        for b in self.buildings:
            cx, cy, _ = b.center
            wx, wy, wz = b.size

            x_min = max(0, int(math.floor(cx - wx / 2)))
            x_max = min(sx - 1, int(math.ceil(cx + wx / 2)))
            y_min = max(0, int(math.floor(cy - wy / 2)))
            y_max = min(sy - 1, int(math.ceil(cy + wy / 2)))
            z_min = 0
            z_max = min(sz - 1, int(math.ceil(wz)))

            self.grid[x_min:x_max + 1, y_min:y_max + 1, z_min:z_max + 1] = 1

    def _is_free(self, p: GridPoint, clearance: int = 1) -> bool:
        x, y, z = p
        sx, sy, sz = self.size

        if not (0 <= x < sx and 0 <= y < sy and 0 <= z < sz):
            return False

        for dx in range(-clearance, clearance + 1):
            for dy in range(-clearance, clearance + 1):
                for dz in range(-clearance, clearance + 1):
                    xx, yy, zz = x + dx, y + dy, z + dz
                    if 0 <= xx < sx and 0 <= yy < sy and 0 <= zz < sz:
                        if self.grid[xx, yy, zz] == 1:
                            return False
        return True

    def _generate_start_and_goal_points(self) -> None:
        sx, sy, sz = self.size

        candidate_starts = [
            (3, 5, 5),
            (3, sy // 2, 5),
            (3, sy - 6, 5),
            (sx // 2, 3, 5),
            (sx // 2, sy - 4, 5),
        ]

        self.starts = []
        for p in candidate_starts:
            pp = self._nearest_free_point(p)
            if pp is not None:
                self.starts.append(pp)
            if len(self.starts) >= self.num_drones:
                break

        while len(self.starts) < self.num_drones:
            p = (random.randint(2, 8), random.randint(3, sy - 4), random.randint(4, 8))
            pp = self._nearest_free_point(p)
            if pp is not None and pp not in self.starts:
                self.starts.append(pp)

        goal_candidate = (sx - 5, sy // 2, 5)
        goal = self._nearest_free_point(goal_candidate)
        if goal is None:
            goal = (sx - 5, sy // 2, 10)
        self.target = goal

    def _nearest_free_point(self, p: GridPoint, max_radius: int = 8) -> Optional[GridPoint]:
        x0, y0, z0 = p

        for r in range(max_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    for dz in range(-r, r + 1):
                        candidate = (x0 + dx, y0 + dy, z0 + dz)
                        if self._is_free(candidate, clearance=1):
                            return candidate
        return None


class LearningGuidedRRTStarNoDP:
    """
    Public no-DP path planner.

    It keeps the sampling-based path planning structure:
        - guided sampling distribution
        - nearest-node extension
        - collision checking
        - raw path reconstruction

    It intentionally does not contain the DP path-compression module.
    """

    def __init__(
        self,
        env_map: np.ndarray,
        start: GridPoint,
        goal: GridPoint,
        step_size: float = 2.5,
        goal_bias: float = 0.18,
        guided_bias: float = 0.70,
        safety_distance: float = 1.0,
        seed: int = 0,
    ) -> None:
        self.env_map = env_map
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.step_size = step_size
        self.goal_bias = goal_bias
        self.guided_bias = guided_bias
        self.safety_distance = safety_distance
        self.rng = np.random.default_rng(seed)

        self.tree: Dict[GridPoint, Optional[GridPoint]] = {self.start: None}
        self.promising_region = self._build_promising_region()

    def _build_promising_region(self) -> np.ndarray:
        sx, sy, sz = self.env_map.shape
        region = np.zeros_like(self.env_map, dtype=np.uint8)

        s = np.array(self.start, dtype=float)
        g = np.array(self.goal, dtype=float)
        sg = g - s
        sg_norm = np.linalg.norm(sg) + 1e-9

        radius = 8.0

        z_low = max(2, int(min(s[2], g[2])) - 4)
        z_high = min(sz, max(8, int(max(s[2], g[2])) + 14))

        for x in range(sx):
            for y in range(sy):
                for z in range(z_low, z_high):
                    if self.env_map[x, y, z] == 1:
                        continue

                    p = np.array([x, y, z], dtype=float)
                    t = np.dot(p - s, sg) / (sg_norm ** 2)
                    t = np.clip(t, 0.0, 1.0)
                    projection = s + t * sg
                    dist_to_line = np.linalg.norm(p - projection)

                    if dist_to_line <= radius:
                        region[x, y, z] = 1

        if np.sum(region) == 0:
            region[self.env_map == 0] = 1

        return region

    def sample(self) -> GridPoint:
        if self.rng.random() < self.goal_bias:
            return self.goal

        if self.rng.random() < self.guided_bias:
            indices = np.argwhere(self.promising_region == 1)
            if len(indices) > 0:
                idx = self.rng.integers(0, len(indices))
                x, y, z = indices[idx]
                return (int(x), int(y), int(z))

        sx, sy, sz = self.env_map.shape
        return (
            int(self.rng.integers(0, sx)),
            int(self.rng.integers(0, sy)),
            int(self.rng.integers(3, sz)),
        )

    def find_nearest(self, rand_pos: GridPoint) -> GridPoint:
        rp = np.array(rand_pos, dtype=float)
        nearest = None
        min_dist = float("inf")

        for node in self.tree.keys():
            dist = np.linalg.norm(np.array(node, dtype=float) - rp)
            if dist < min_dist:
                min_dist = dist
                nearest = node

        assert nearest is not None
        return nearest

    def steer(self, from_pos: GridPoint, to_pos: GridPoint) -> GridPoint:
        f = np.array(from_pos, dtype=float)
        t = np.array(to_pos, dtype=float)
        direction = t - f
        norm = np.linalg.norm(direction)

        if norm < 1e-9:
            return from_pos

        new_pos = f + direction / norm * min(self.step_size, norm)

        sx, sy, sz = self.env_map.shape
        new_pos = np.clip(new_pos, [0, 0, 0], [sx - 1, sy - 1, sz - 1])

        return tuple(int(round(v)) for v in new_pos)

    def is_collision_free(self, from_pos: GridPoint, to_pos: GridPoint) -> bool:
        f = np.array(from_pos, dtype=float)
        t = np.array(to_pos, dtype=float)
        direction = t - f
        norm = np.linalg.norm(direction)

        if norm < 1e-9:
            return False

        steps = max(2, int(math.ceil(norm * 2)))
        sx, sy, sz = self.env_map.shape
        clearance = int(math.ceil(self.safety_distance))

        for i in range(steps + 1):
            alpha = i / steps
            p = f * (1 - alpha) + t * alpha
            x, y, z = [int(round(v)) for v in p]

            for dx in range(-clearance, clearance + 1):
                for dy in range(-clearance, clearance + 1):
                    for dz in range(-clearance, clearance + 1):
                        xx, yy, zz = x + dx, y + dy, z + dz

                        if not (0 <= xx < sx and 0 <= yy < sy and 0 <= zz < sz):
                            return False

                        if self.env_map[xx, yy, zz] == 1:
                            return False

        return True

    def reconstruct_path(self, end_pos: GridPoint) -> List[GridPoint]:
        path: List[GridPoint] = []
        current: Optional[GridPoint] = end_pos

        while current is not None:
            path.append(current)
            current = self.tree[current]

        return path[::-1]

    def is_path_valid(self, path: Sequence[GridPoint]) -> bool:
        if len(path) < 2:
            return False

        for i in range(len(path) - 1):
            if not self.is_collision_free(path[i], path[i + 1]):
                return False
        return True

    def calculate_path_length(self, path: Sequence[GridPoint]) -> float:
        if len(path) < 2:
            return 0.0

        total = 0.0
        for i in range(len(path) - 1):
            total += float(np.linalg.norm(np.array(path[i + 1]) - np.array(path[i])))
        return total

    def plan(self, max_iterations: int = 12000, timeout: float = 30.0) -> Optional[List[GridPoint]]:
        start_time = time.time()

        for iteration in range(max_iterations):
            if time.time() - start_time > timeout:
                print("Planning exceeded time limit.")
                return None

            rand_pos = self.sample()
            nearest_pos = self.find_nearest(rand_pos)
            new_pos = self.steer(nearest_pos, rand_pos)

            if new_pos in self.tree:
                continue

            if not self.is_collision_free(nearest_pos, new_pos):
                continue

            self.tree[new_pos] = nearest_pos

            if np.linalg.norm(np.array(new_pos) - np.array(self.goal)) <= max(3.5, self.step_size + 1.0):
                if self.is_collision_free(new_pos, self.goal):
                    self.tree[self.goal] = new_pos
                    path = self.reconstruct_path(self.goal)

                    if self.is_path_valid(path):
                        elapsed = time.time() - start_time
                        length = self.calculate_path_length(path)
                        print(
                            f"Path found. Iter={iteration}, Time={elapsed:.3f}s, "
                            f"Length={length:.2f}, Nodes={len(path)}"
                        )
                        return path

        print("Planning reached maximum iterations.")
        return None


class MultiUAVCoordinator:
    def __init__(self, env: CityEnvironment3D, seed: int = 0) -> None:
        self.env = env
        self.seed = seed

    def coordinate(self, max_iterations: int, timeout: float) -> Optional[List[List[GridPoint]]]:
        paths: List[List[GridPoint]] = []

        for drone_id, start in enumerate(self.env.starts):
            print(f"Planning UAV {drone_id + 1}: start={start}, goal={self.env.target}")

            planner = LearningGuidedRRTStarNoDP(
                env_map=self.env.grid,
                start=start,
                goal=self.env.target,
                step_size=2.8,
                goal_bias=0.20,
                guided_bias=0.72,
                safety_distance=1.0,
                seed=self.seed + drone_id * 100,
            )

            path = planner.plan(max_iterations=max_iterations, timeout=timeout)

            if path is None:
                print(f"UAV {drone_id + 1} failed to find a path.")
                return None

            paths.append(path)

        return paths


def export_ros2_city_demo_result(
    save_path: str | Path,
    env: CityEnvironment3D,
    paths: Sequence[Sequence[GridPoint]],
) -> None:
    data = {
        "scene_type": "structured_city",
        "map_size": [int(v) for v in env.size],
        "cell_size": 1.0,
        "buildings": [],
        "roads": [],
        "starts": [[float(v) for v in s] for s in env.starts],
        "target": [float(v) for v in env.target],
        "uavs": [],
        "note": (
            "Public ROS2/RViz2 demo. The full research version, including "
            "DP-based path compression, is not released in this file."
        ),
    }

    for b in env.buildings:
        data["buildings"].append(
            {
                "id": int(b.building_id),
                "type": b.building_type,
                "center": [float(v) for v in b.center],
                "size": [float(v) for v in b.size],
            }
        )

    for r in env.roads:
        data["roads"].append(
            {
                "id": int(r.road_id),
                "center": [float(v) for v in r.center],
                "size": [float(v) for v in r.size],
            }
        )

    for i, path in enumerate(paths):
        data["uavs"].append(
            {
                "id": int(i + 1),
                "start": [float(v) for v in env.starts[i]],
                "goal": [float(v) for v in env.target],
                "path": [[float(x), float(y), float(z)] for x, y, z in path],
                "delay": 0.0,
            }
        )

    save_path = Path(save_path)
    with open(save_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

    print(f"ROS2 city demo result saved to: {save_path.resolve()}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=str, default="ros2_demo_result.json")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--num-uavs", type=int, default=3)
    parser.add_argument("--max-iterations", type=int, default=15000)
    parser.add_argument("--timeout", type=float, default=35.0)
    parser.add_argument("--size", type=int, nargs=3, default=[60, 60, 30])
    args = parser.parse_args()

    env = CityEnvironment3D(
        size=tuple(args.size),
        num_drones=args.num_uavs,
        seed=args.seed,
        building_spacing=9,
        road_width=4,
    )

    coordinator = MultiUAVCoordinator(env, seed=args.seed)
    paths = coordinator.coordinate(
        max_iterations=args.max_iterations,
        timeout=args.timeout,
    )

    if paths is None:
        raise RuntimeError("Failed to find valid paths for all UAVs.")

    export_ros2_city_demo_result(args.out, env, paths)


if __name__ == "__main__":
    main()
