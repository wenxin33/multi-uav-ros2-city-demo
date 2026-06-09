#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
city_result_visualizer_node.py

ROS2/RViz2 visualizer with simplified UAV kinematics and quadrotor parameters.

This file is designed to replace:
    ~/uav_ros2_ws/src/vit_corrt_ros2_city_demo/vit_corrt_ros2_city_demo/city_result_visualizer_node.py

Inputs:
    ~/uav_ros2_ws/src/vit_corrt_ros2_city_demo/data/ros2_demo_result.json

Published topics:
    /map/buildings
    /map/roads
    /map/boundary
    /map/start_goal_markers
    /uav_1/path, /uav_2/path, ...
    /uav/current_positions
    /uav/velocity_arrows
    /uav/state_text
    /uav_1/odom, /uav_2/odom, ...

Model scope:
    - This is a simplified kinematic simulation, not a full quadrotor physics simulator.
    - Quadrotor parameters are included for engineering display and later energy-model extension.
    - The UAV moves along each planned path with cruise-speed and segment interpolation.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as RosPath
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class QuadrotorParams:
    """
    Simplified quadrotor parameters.

    These parameters are not used for full motor-level dynamics yet.
    They provide a physically meaningful interface for the next energy-model stage.
    """
    mass_kg: float = 1.60
    rotor_count: int = 4
    arm_length_m: float = 0.32
    rotor_radius_m: float = 0.12
    max_speed_mps: float = 8.0
    cruise_speed_mps: float = 3.0
    max_accel_mps2: float = 3.0
    max_climb_rate_mps: float = 2.0
    gravity_mps2: float = 9.81

    @property
    def hover_thrust_n(self) -> float:
        return self.mass_kg * self.gravity_mps2

    @property
    def hover_thrust_per_rotor_n(self) -> float:
        return self.hover_thrust_n / float(self.rotor_count)


class CityResultVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__('city_result_visualizer')

        default_file = str(
            Path.home()
            / 'uav_ros2_ws'
            / 'src'
            / 'vit_corrt_ros2_city_demo'
            / 'data'
            / 'ros2_demo_result.json'
        )

        self.declare_parameter('result_file', default_file)
        self.declare_parameter('cruise_speed_mps', 3.0)
        self.declare_parameter('uav_timer_period', 0.2)
        self.declare_parameter('static_timer_period', 5.0)

        self.result_file = self.get_parameter('result_file').value
        self.cruise_speed_mps = float(self.get_parameter('cruise_speed_mps').value)
        self.uav_timer_period = float(self.get_parameter('uav_timer_period').value)
        self.static_timer_period = float(self.get_parameter('static_timer_period').value)

        self.data = self._load_json(self.result_file)

        self.quad_params = QuadrotorParams(cruise_speed_mps=self.cruise_speed_mps)

        # Static scene publishers
        self.building_pub = self.create_publisher(MarkerArray, '/map/buildings', 10)
        self.road_pub = self.create_publisher(MarkerArray, '/map/roads', 10)
        self.start_goal_pub = self.create_publisher(MarkerArray, '/map/start_goal_markers', 10)
        self.boundary_pub = self.create_publisher(MarkerArray, '/map/boundary', 10)

        # Dynamic visualization publishers
        self.uav_marker_pub = self.create_publisher(MarkerArray, '/uav/current_positions', 10)
        self.velocity_arrow_pub = self.create_publisher(MarkerArray, '/uav/velocity_arrows', 10)
        self.state_text_pub = self.create_publisher(MarkerArray, '/uav/state_text', 10)

        # Path and odometry publishers
        self.path_publishers = {}
        self.odom_publishers = {}

        for uav in self.data.get('uavs', []):
            uav_id = int(uav['id'])
            self.path_publishers[uav_id] = self.create_publisher(
                RosPath,
                f'/uav_{uav_id}/path',
                10,
            )
            self.odom_publishers[uav_id] = self.create_publisher(
                Odometry,
                f'/uav_{uav_id}/odom',
                10,
            )

        # Precompute path arc-length information.
        self.path_cache = {}
        for uav in self.data.get('uavs', []):
            uav_id = int(uav['id'])
            self.path_cache[uav_id] = self._build_path_cache(uav.get('path', []))

        self.sim_time = 0.0

        # Static scene: low frequency, avoids RViz2 overload.
        self.static_timer = self.create_timer(self.static_timer_period, self.publish_static_scene)

        # UAV kinematic state: higher frequency.
        self.uav_timer = self.create_timer(self.uav_timer_period, self.publish_dynamic_uavs)

        self.get_logger().info('Structured city ROS2 result visualizer with kinematics started.')
        self.get_logger().info(f'Loaded result file: {self.result_file}')
        self.get_logger().info(f'UAV count: {len(self.data.get("uavs", []))}')
        self.get_logger().info(f'Building count: {len(self.data.get("buildings", []))}')
        self.get_logger().info(
            'Quadrotor params: '
            f'mass={self.quad_params.mass_kg} kg, '
            f'rotors={self.quad_params.rotor_count}, '
            f'arm_length={self.quad_params.arm_length_m} m, '
            f'rotor_radius={self.quad_params.rotor_radius_m} m, '
            f'cruise_speed={self.quad_params.cruise_speed_mps} m/s, '
            f'hover_thrust={self.quad_params.hover_thrust_n:.2f} N'
        )

    def _load_json(self, path: str) -> Dict:
        with open(path, 'r', encoding='utf-8') as f:
            return json.load(f)

    def _stamp(self) -> Time:
        return self.get_clock().now().to_msg()

    def _set_color(self, marker: Marker, rgba: Tuple[float, float, float, float]) -> None:
        marker.color.r = float(rgba[0])
        marker.color.g = float(rgba[1])
        marker.color.b = float(rgba[2])
        marker.color.a = float(rgba[3])

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _building_color(self, building_type: str) -> Tuple[float, float, float, float]:
        if building_type == 'commercial':
            return (0.22, 0.22, 0.24, 0.82)
        if building_type == 'residential':
            return (0.55, 0.55, 0.58, 0.72)
        if building_type == 'public':
            return (0.25, 0.42, 0.70, 0.68)
        return (0.48, 0.48, 0.50, 0.70)

    def _build_path_cache(self, path: List[List[float]]) -> Dict:
        points = [[float(v) for v in p] for p in path]
        seg_lengths = []
        cum_lengths = [0.0]

        for i in range(len(points) - 1):
            p0 = points[i]
            p1 = points[i + 1]
            length = math.sqrt(
                (p1[0] - p0[0]) ** 2
                + (p1[1] - p0[1]) ** 2
                + (p1[2] - p0[2]) ** 2
            )
            seg_lengths.append(length)
            cum_lengths.append(cum_lengths[-1] + length)

        return {
            'points': points,
            'seg_lengths': seg_lengths,
            'cum_lengths': cum_lengths,
            'total_length': cum_lengths[-1] if cum_lengths else 0.0,
        }

    def _interpolate_state_on_path(self, uav_id: int, t: float) -> Dict:
        cache = self.path_cache[uav_id]
        points = cache['points']
        seg_lengths = cache['seg_lengths']
        total_length = cache['total_length']

        if len(points) == 0:
            return {
                'position': [0.0, 0.0, 0.0],
                'velocity': [0.0, 0.0, 0.0],
                'speed': 0.0,
                'yaw': 0.0,
                'distance': 0.0,
                'loop_time': 0.0,
            }

        if len(points) == 1 or total_length <= 1e-9:
            return {
                'position': points[0],
                'velocity': [0.0, 0.0, 0.0],
                'speed': 0.0,
                'yaw': 0.0,
                'distance': 0.0,
                'loop_time': 0.0,
            }

        speed = max(0.1, min(self.quad_params.cruise_speed_mps, self.quad_params.max_speed_mps))
        loop_time = total_length / speed
        distance = (t * speed) % total_length

        current_seg = 0
        for i in range(len(seg_lengths)):
            if cache['cum_lengths'][i] <= distance <= cache['cum_lengths'][i + 1]:
                current_seg = i
                break

        p0 = points[current_seg]
        p1 = points[current_seg + 1]
        seg_len = max(seg_lengths[current_seg], 1e-9)

        local_d = distance - cache['cum_lengths'][current_seg]
        alpha = max(0.0, min(1.0, local_d / seg_len))

        pos = [
            p0[0] * (1.0 - alpha) + p1[0] * alpha,
            p0[1] * (1.0 - alpha) + p1[1] * alpha,
            p0[2] * (1.0 - alpha) + p1[2] * alpha,
        ]

        direction = [
            (p1[0] - p0[0]) / seg_len,
            (p1[1] - p0[1]) / seg_len,
            (p1[2] - p0[2]) / seg_len,
        ]

        vz = speed * direction[2]
        if abs(vz) > self.quad_params.max_climb_rate_mps:
            vz = math.copysign(self.quad_params.max_climb_rate_mps, vz)

        vel = [
            speed * direction[0],
            speed * direction[1],
            vz,
        ]

        yaw = math.atan2(vel[1], vel[0]) if abs(vel[0]) + abs(vel[1]) > 1e-9 else 0.0

        return {
            'position': pos,
            'velocity': vel,
            'speed': math.sqrt(vel[0] ** 2 + vel[1] ** 2 + vel[2] ** 2),
            'yaw': yaw,
            'distance': distance,
            'loop_time': loop_time,
        }

    def make_building_markers(self) -> MarkerArray:
        arr = MarkerArray()

        for i, b in enumerate(self.data.get('buildings', [])):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self._stamp()
            marker.ns = 'buildings'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            center = b['center']
            size = b['size']

            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])
            marker.pose.orientation.w = 1.0

            marker.scale.x = float(size[0])
            marker.scale.y = float(size[1])
            marker.scale.z = float(size[2])

            self._set_color(marker, self._building_color(b.get('type', 'unknown')))
            arr.markers.append(marker)

        return arr

    def make_road_markers(self) -> MarkerArray:
        arr = MarkerArray()

        for i, r in enumerate(self.data.get('roads', [])):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self._stamp()
            marker.ns = 'roads'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            center = r['center']
            size = r['size']

            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])
            marker.pose.orientation.w = 1.0

            marker.scale.x = float(size[0])
            marker.scale.y = float(size[1])
            marker.scale.z = float(size[2])

            self._set_color(marker, (0.05, 0.05, 0.05, 0.85))
            arr.markers.append(marker)

        return arr

    def make_boundary_markers(self) -> MarkerArray:
        arr = MarkerArray()
        map_size = self.data.get('map_size', [60, 60, 30])
        sx, sy, _ = [float(v) for v in map_size]

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self._stamp()
        marker.ns = 'boundary'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = sx / 2.0
        marker.pose.position.y = sy / 2.0
        marker.pose.position.z = -0.03
        marker.pose.orientation.w = 1.0

        marker.scale.x = sx
        marker.scale.y = sy
        marker.scale.z = 0.04

        self._set_color(marker, (0.12, 0.12, 0.12, 0.22))
        arr.markers.append(marker)

        return arr

    def make_start_goal_markers(self) -> MarkerArray:
        arr = MarkerArray()
        marker_id = 0

        for i, start in enumerate(self.data.get('starts', [])):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self._stamp()
            marker.ns = 'starts'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(start[0])
            marker.pose.position.y = float(start[1])
            marker.pose.position.z = float(start[2])
            marker.pose.orientation.w = 1.0

            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            self._set_color(marker, (0.0, 1.0, 0.0, 1.0))
            arr.markers.append(marker)

            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = self._stamp()
            text.ns = 'start_labels'
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(start[0])
            text.pose.position.y = float(start[1])
            text.pose.position.z = float(start[2]) + 1.4
            text.pose.orientation.w = 1.0
            text.scale.z = 1.2
            text.text = f'S{i + 1}'
            self._set_color(text, (0.0, 1.0, 0.0, 1.0))
            arr.markers.append(text)

        goal = self.data.get('target', [0, 0, 0])

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self._stamp()
        marker.ns = 'goal'
        marker.id = marker_id
        marker_id += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(goal[0])
        marker.pose.position.y = float(goal[1])
        marker.pose.position.z = float(goal[2])
        marker.pose.orientation.w = 1.0

        marker.scale.x = 1.3
        marker.scale.y = 1.3
        marker.scale.z = 1.3
        self._set_color(marker, (1.0, 0.0, 0.0, 1.0))
        arr.markers.append(marker)

        text = Marker()
        text.header.frame_id = 'map'
        text.header.stamp = self._stamp()
        text.ns = 'goal_label'
        text.id = marker_id
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(goal[0])
        text.pose.position.y = float(goal[1])
        text.pose.position.z = float(goal[2]) + 1.6
        text.pose.orientation.w = 1.0
        text.scale.z = 1.3
        text.text = 'Goal'
        self._set_color(text, (1.0, 0.0, 0.0, 1.0))
        arr.markers.append(text)

        return arr

    def make_path_msg(self, path_points: List[List[float]]) -> RosPath:
        msg = RosPath()
        msg.header.frame_id = 'map'
        msg.header.stamp = self._stamp()

        for point in path_points:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = msg.header.stamp
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = float(point[2])
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        return msg

    def make_uav_markers(self, states: Dict[int, Dict]) -> MarkerArray:
        arr = MarkerArray()

        colors = [
            (1.0, 0.45, 0.0, 1.0),
            (0.0, 0.65, 1.0, 1.0),
            (1.0, 0.8, 0.0, 1.0),
            (1.0, 0.2, 0.2, 1.0),
            (0.8, 0.2, 1.0, 1.0),
        ]

        marker_id = 0

        for i, uav in enumerate(self.data.get('uavs', [])):
            uav_id = int(uav['id'])
            if uav_id not in states:
                continue

            state = states[uav_id]
            pos = state['position']
            yaw = state['yaw']
            color = colors[i % len(colors)]

            x, y, z = float(pos[0]), float(pos[1]), float(pos[2])

            # Body
            body = Marker()
            body.header.frame_id = 'map'
            body.header.stamp = self._stamp()
            body.ns = 'uav_body'
            body.id = marker_id
            marker_id += 1
            body.type = Marker.SPHERE
            body.action = Marker.ADD
            body.pose.position.x = x
            body.pose.position.y = y
            body.pose.position.z = z
            body.pose.orientation = self._yaw_to_quaternion(yaw)
            body.scale.x = 2.2
            body.scale.y = 2.2
            body.scale.z = 0.7
            self._set_color(body, color)
            arr.markers.append(body)

            # Arms
            arm_length = 3.5
            arm_thickness = 0.25

            arm_x = Marker()
            arm_x.header.frame_id = 'map'
            arm_x.header.stamp = self._stamp()
            arm_x.ns = 'uav_arm_x'
            arm_x.id = marker_id
            marker_id += 1
            arm_x.type = Marker.CUBE
            arm_x.action = Marker.ADD
            arm_x.pose.position.x = x
            arm_x.pose.position.y = y
            arm_x.pose.position.z = z
            arm_x.pose.orientation = self._yaw_to_quaternion(yaw)
            arm_x.scale.x = arm_length
            arm_x.scale.y = arm_thickness
            arm_x.scale.z = arm_thickness
            self._set_color(arm_x, color)
            arr.markers.append(arm_x)

            arm_y = Marker()
            arm_y.header.frame_id = 'map'
            arm_y.header.stamp = self._stamp()
            arm_y.ns = 'uav_arm_y'
            arm_y.id = marker_id
            marker_id += 1
            arm_y.type = Marker.CUBE
            arm_y.action = Marker.ADD
            arm_y.pose.position.x = x
            arm_y.pose.position.y = y
            arm_y.pose.position.z = z
            arm_y.pose.orientation = self._yaw_to_quaternion(yaw + math.pi / 2.0)
            arm_y.scale.x = arm_length
            arm_y.scale.y = arm_thickness
            arm_y.scale.z = arm_thickness
            self._set_color(arm_y, color)
            arr.markers.append(arm_y)

            # Four rotors as small cylinders/spheres at arm ends
            rotor_offsets = [
                (math.cos(yaw) * arm_length / 2.0, math.sin(yaw) * arm_length / 2.0),
                (-math.cos(yaw) * arm_length / 2.0, -math.sin(yaw) * arm_length / 2.0),
                (math.cos(yaw + math.pi / 2.0) * arm_length / 2.0, math.sin(yaw + math.pi / 2.0) * arm_length / 2.0),
                (-math.cos(yaw + math.pi / 2.0) * arm_length / 2.0, -math.sin(yaw + math.pi / 2.0) * arm_length / 2.0),
            ]

            for dx, dy in rotor_offsets:
                rotor = Marker()
                rotor.header.frame_id = 'map'
                rotor.header.stamp = self._stamp()
                rotor.ns = 'uav_rotors'
                rotor.id = marker_id
                marker_id += 1
                rotor.type = Marker.SPHERE
                rotor.action = Marker.ADD
                rotor.pose.position.x = x + dx
                rotor.pose.position.y = y + dy
                rotor.pose.position.z = z
                rotor.pose.orientation.w = 1.0
                rotor.scale.x = 0.55
                rotor.scale.y = 0.55
                rotor.scale.z = 0.18
                self._set_color(rotor, (0.05, 0.05, 0.05, 1.0))
                arr.markers.append(rotor)

            # UAV label
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = self._stamp()
            text.ns = 'uav_labels'
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = z + 1.8
            text.pose.orientation.w = 1.0
            text.scale.z = 1.2
            text.text = f'UAV {uav_id}'
            self._set_color(text, color)
            arr.markers.append(text)

        return arr

    def make_velocity_arrows(self, states: Dict[int, Dict]) -> MarkerArray:
        arr = MarkerArray()
        marker_id = 0

        for uav_id, state in states.items():
            pos = state['position']
            vel = state['velocity']

            start = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
            end = Point(
                x=float(pos[0] + vel[0] * 0.8),
                y=float(pos[1] + vel[1] * 0.8),
                z=float(pos[2] + vel[2] * 0.8),
            )

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self._stamp()
            marker.ns = 'velocity_arrows'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.points = [start, end]
            marker.scale.x = 0.25
            marker.scale.y = 0.55
            marker.scale.z = 0.55
            self._set_color(marker, (0.0, 1.0, 1.0, 1.0))
            arr.markers.append(marker)

        return arr

    def make_state_text_markers(self, states: Dict[int, Dict]) -> MarkerArray:
        arr = MarkerArray()
        marker_id = 0

        for uav_id, state in states.items():
            pos = state['position']
            speed = state['speed']
            vel = state['velocity']

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self._stamp()
            marker.ns = 'uav_state_text'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD

            marker.pose.position.x = float(pos[0])
            marker.pose.position.y = float(pos[1])
            marker.pose.position.z = float(pos[2]) + 3.2
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.95

            marker.text = (
                f'UAV {uav_id}\\n'
                f'v={speed:.2f} m/s\\n'
                f'vz={vel[2]:.2f} m/s\\n'
                f'm={self.quad_params.mass_kg:.1f} kg'
            )

            self._set_color(marker, (1.0, 1.0, 1.0, 1.0))
            arr.markers.append(marker)

        return arr

    def make_odometry_msg(self, uav_id: int, state: Dict) -> Odometry:
        pos = state['position']
        vel = state['velocity']
        yaw = state['yaw']

        msg = Odometry()
        msg.header.frame_id = 'map'
        msg.header.stamp = self._stamp()
        msg.child_frame_id = f'uav_{uav_id}/base_link'

        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.position.z = float(pos[2])
        msg.pose.pose.orientation = self._yaw_to_quaternion(yaw)

        msg.twist.twist.linear.x = float(vel[0])
        msg.twist.twist.linear.y = float(vel[1])
        msg.twist.twist.linear.z = float(vel[2])

        return msg

    def compute_all_states(self) -> Dict[int, Dict]:
        states = {}

        for uav in self.data.get('uavs', []):
            uav_id = int(uav['id'])
            states[uav_id] = self._interpolate_state_on_path(uav_id, self.sim_time)

        return states

    def publish_static_scene(self) -> None:
        self.boundary_pub.publish(self.make_boundary_markers())
        self.road_pub.publish(self.make_road_markers())
        self.building_pub.publish(self.make_building_markers())
        self.start_goal_pub.publish(self.make_start_goal_markers())

        for uav in self.data.get('uavs', []):
            uav_id = int(uav['id'])
            path_msg = self.make_path_msg(uav['path'])
            self.path_publishers[uav_id].publish(path_msg)

    def publish_dynamic_uavs(self) -> None:
        states = self.compute_all_states()

        self.uav_marker_pub.publish(self.make_uav_markers(states))
        self.velocity_arrow_pub.publish(self.make_velocity_arrows(states))
        self.state_text_pub.publish(self.make_state_text_markers(states))

        for uav_id, state in states.items():
            self.odom_publishers[uav_id].publish(self.make_odometry_msg(uav_id, state))

        self.sim_time += self.uav_timer_period


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CityResultVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
