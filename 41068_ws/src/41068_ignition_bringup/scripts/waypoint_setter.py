#!/usr/bin/env python3
"""
Trail Waypoint Setter
---------------------

Reads a Gazebo world, samples waypoint locations that avoid existing
objects, optionally spawns a marker model for each location, and publishes
the locations as a PoseArray in the Nav2 map frame.  Use this to seed the
maintenance GUI with ad‑hoc inspection targets.
"""

from __future__ import annotations

import json
import math
import os
import random
import subprocess
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header

try:
    # Prefer Gazebo services when available
    from ros_gz_interfaces.srv import SpawnEntity, RemoveEntity

    HAS_GZ = True
except Exception:  # pragma: no cover - ros_gz not installed
    HAS_GZ = False


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
@dataclass
class WorldEntity:
    name: str
    x: float
    y: float
    radius: float


def _parse_pose(pose_text: str) -> Tuple[float, float, float, float]:
    vals = [float(p) for p in (pose_text or "0 0 0 0 0 0").split()]
    vals += [0.0] * (6 - len(vals))
    x, y, z, _, _, yaw = vals[:6]
    return x, y, z, yaw


def _collect_includes(world_sdf: str) -> List[Tuple[str, str, str]]:
    tree = ET.parse(world_sdf)
    root = tree.getroot()
    out: List[Tuple[str, str, str]] = []
    for inc in root.findall(".//include"):
        name = (inc.findtext("name") or "").strip()
        uri = (inc.findtext("uri") or "").strip()
        pose = (inc.findtext("pose") or "0 0 0 0 0 0").strip()
        out.append((name, uri, pose))
    return out


def _entity_radius(name: str) -> float:
    if name.startswith(("oak", "pine", "tree")):
        return 1.4
    if name.startswith(("rock", "boulder")):
        return 0.8
    if name.startswith("forest_wall"):
        return 0.9
    if name.startswith("maintenance_marker"):
        return 0.2
    return 0.8


def _world_bounds_from_walls(includes: List[Tuple[str, str, str]]) -> Tuple[float, float, float, float]:
    xs, ys = [], []
    for name, _, pose in includes:
        if name.startswith("forest_wall"):
            x, y, *_ = _parse_pose(pose)
            xs.append(x)
            ys.append(y)
    if not xs or not ys:
        return -12.0, 12.0, -12.0, 12.0
    return min(xs), max(xs), min(ys), max(ys)


def _build_entities(includes: List[Tuple[str, str, str]]) -> List[WorldEntity]:
    out: List[WorldEntity] = []
    for name, uri, pose in includes:
        if name in {"forest_plane", "ground_plane"}:
            continue
        x, y, *_ = _parse_pose(pose)
        out.append(WorldEntity(name=name, x=x, y=y, radius=_entity_radius(name)))
    return out


def _clashes_entity(x: float, y: float, entities: Iterable[WorldEntity], pad: float = 0.0) -> Optional[str]:
    for ent in entities:
        if math.hypot(x - ent.x, y - ent.y) < (ent.radius + pad):
            return ent.name
    return None


def _clashes_waypoints(x: float, y: float, others: Iterable[Tuple[float, float]], min_spacing: float) -> bool:
    for ox, oy in others:
        if math.hypot(x - ox, y - oy) < min_spacing:
            return True
    return False


def _include_xml(model_uri: str, name: str) -> str:
    return f"""<sdf version='1.8'>
  <include>
    <name>{name}</name>
    <uri>{model_uri}</uri>
  </include>
</sdf>
"""


# --------------------------------------------------------------------------- #
class WaypointSetter(Node):
    """Sample and publish maintenance waypoints derived from the Gazebo world."""

    def __init__(self) -> None:
        super().__init__("waypoint_setter")

        # --- parameters ---
        self.declare_parameter("world_name", "large_demo")
        self.declare_parameter("num_waypoints", 6)
        self.declare_parameter("seed", 0)
        self.declare_parameter("min_spacing", 1.8)
        self.declare_parameter("wall_margin", 0.5)
        self.declare_parameter("robot_keepout_radius", 2.5)
        self.declare_parameter("z_height", 0.0)
        self.declare_parameter("spawn_models", True)
        self.declare_parameter("model_choices", ["model://maintenance_marker_red", "model://maintenance_marker_green"])

        # map-frame offset (from tf2_echo map->odom)
        self.declare_parameter("map_offset_x", 0.0)
        self.declare_parameter("map_offset_y", 0.0)
        self.declare_parameter("map_offset_yaw", 0.0)

        self.world_name = str(self.get_parameter("world_name").value)
        self.num_waypoints = int(self.get_parameter("num_waypoints").value)
        self.seed = int(self.get_parameter("seed").value)
        self.min_spacing = float(self.get_parameter("min_spacing").value)
        self.wall_margin = float(self.get_parameter("wall_margin").value)
        self.robot_keepout_radius = float(self.get_parameter("robot_keepout_radius").value)
        self.z_height = float(self.get_parameter("z_height").value)
        self.spawn_models = bool(self.get_parameter("spawn_models").value)
        self.model_choices = [str(u) for u in self.get_parameter("model_choices").value]

        self.map_offset_x = float(self.get_parameter("map_offset_x").value)
        self.map_offset_y = float(self.get_parameter("map_offset_y").value)
        self.map_offset_yaw = float(self.get_parameter("map_offset_yaw").value)

        random.seed(self.seed or time.time())

        # Pre-compute rotation helpers
        self._cos_offset = math.cos(self.map_offset_yaw)
        self._sin_offset = math.sin(self.map_offset_yaw)

        # QoS so Nav2 GUI picks up the list after launch
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub_targets = self.create_publisher(PoseArray, "/maintenance/targets", qos)

        # Load world description
        share_dir = get_package_share_directory("41068_ignition_bringup")
        sdf_path = os.path.join(share_dir, "worlds", f"{self.world_name}.sdf")
        if not os.path.isfile(sdf_path):
            raise FileNotFoundError(f"World SDF not found: {sdf_path}")

        includes = _collect_includes(sdf_path)
        xmin, xmax, ymin, ymax = _world_bounds_from_walls(includes)
        xmin += self.wall_margin
        xmax -= self.wall_margin
        ymin += self.wall_margin
        ymax -= self.wall_margin

        entities = _build_entities(includes)
        entities.append(WorldEntity("husky_spawn", 0.0, 0.0, self.robot_keepout_radius))

        if self.spawn_models:
            self._remove_old_markers(entities)

        # Sample waypoints
        world_waypoints: List[Tuple[float, float]] = []
        model_uris: List[str] = []
        attempts, max_attempts = 0, 300
        while len(world_waypoints) < self.num_waypoints and attempts < max_attempts:
            attempts += 1
            wx = random.uniform(xmin, xmax)
            wy = random.uniform(ymin, ymax)

            if _clashes_entity(wx, wy, entities, pad=0.2):
                continue
            if _clashes_waypoints(wx, wy, world_waypoints, self.min_spacing):
                continue

            choice = random.choice(self.model_choices) if self.model_choices else ""
            world_waypoints.append((wx, wy))
            model_uris.append(choice)

        if len(world_waypoints) < self.num_waypoints:
            self.get_logger().warn(
                f"Placed {len(world_waypoints)}/{self.num_waypoints} after {attempts} attempts. "
                "Adjust min_spacing or margins for denser coverage."
            )

        # Convert to map frame for Nav2
        map_waypoints = [self._to_map_frame(wx, wy) for (wx, wy) in world_waypoints]

        if self.spawn_models:
            self._spawn_markers(world_waypoints, model_uris)

        self._publish_pose_array(map_waypoints)
        self.get_logger().info(f"✅ Published {len(map_waypoints)} targets (frame='map'). No repeating timer.")

        summary = [
            {"name": f"waypoint_rand_{i+1:02d}", "world": {"x": float(wx), "y": float(wy)}, "map": {"x": float(mx), "y": float(my)}}
            for i, ((wx, wy), (mx, my)) in enumerate(zip(world_waypoints, map_waypoints))
        ]
        self.get_logger().info("Waypoint summary:\n" + json.dumps(summary, indent=2))

    # ------------------------------------------------------------------
    def _to_map_frame(self, wx: float, wy: float) -> Tuple[float, float]:
        dx = wx - self.map_offset_x
        dy = wy - self.map_offset_y
        mx = self._cos_offset * dx + self._sin_offset * dy
        my = -self._sin_offset * dx + self._cos_offset * dy
        return mx, my

    def _publish_pose_array(self, points_xy: List[Tuple[float, float]]) -> None:
        msg = PoseArray()
        msg.header = Header()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in points_xy:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = self.z_height
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.pub_targets.publish(msg)

    # ------------------------------------------------------------------
    def _available_spawn_cmds(self) -> List[List[str]]:
        candidates = [
            ["ros2", "run", "ros_gz_sim", "create"],
            ["ros2", "run", "ros_ign_gazebo", "create"],
        ]
        usable: List[List[str]] = []
        for cmd in candidates:
            pkg = cmd[2]
            try:
                out = subprocess.run(["ros2", "pkg", "executables", pkg],
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
                if b"create" in out.stdout:
                    usable.append(cmd)
            except Exception:
                pass
        return usable or candidates

    def _spawn_markers(self, world_xy: List[Tuple[float, float]], model_uris: List[str]) -> None:
        if not world_xy:
            return

        if HAS_GZ:
            try:
                client = self.create_client(SpawnEntity, f"/world/{self.world_name}/create")
                if client.wait_for_service(timeout_sec=5.0):
                    for idx, ((wx, wy), uri) in enumerate(zip(world_xy, model_uris), start=1):
                        req = SpawnEntity.Request()
                        req.name = f"maintenance_marker_{idx:02d}"
                        req.xml = _include_xml(uri, req.name)
                        pose = Pose()
                        pose.position.x = wx
                        pose.position.y = wy
                        pose.position.z = self.z_height
                        if hasattr(req, "initial_pose"):
                            req.initial_pose = pose
                        elif hasattr(req, "pose"):
                            setattr(req, "pose", pose)
                        if hasattr(req, "allow_renaming"):
                            req.allow_renaming = False
                        fut = client.call_async(req)
                        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
                        self.get_logger().info(f"spawn[{req.name}] via service (uri={uri})")
                    return
            except Exception as exc:  # pragma: no cover - service failure path
                self.get_logger().warn(f"SpawnEntity service failed: {exc}")

        # Fallback to CLI runner
        for runner in self._available_spawn_cmds():
            for idx, ((wx, wy), uri) in enumerate(zip(world_xy, model_uris), start=1):
                name = f"maintenance_marker_{idx:02d}"
                xml = _include_xml(uri, name)
                cmd = runner + [
                    "-world", self.world_name,
                    "-name", name,
                    "-x", f"{wx:.3f}", "-y", f"{wy:.3f}", "-z", f"{self.z_height:.3f}",
                    "-string", xml,
                ]
                try:
                    subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    self.get_logger().info(f"spawn[{name}] via CLI {' '.join(runner)}")
                except subprocess.CalledProcessError as exc:
                    self.get_logger().error(f"spawn[{name}] failed:\n{exc.stderr.decode(errors='ignore')}")

    def _remove_old_markers(self, entities: List[WorldEntity]) -> None:
        if not HAS_GZ:
            return
        old_markers = [e.name for e in entities if e.name.startswith("maintenance_marker_")]
        if not old_markers:
            return
        cli = self.create_client(RemoveEntity, f"/world/{self.world_name}/remove")
        if not cli.wait_for_service(timeout_sec=3.0):
            return
        for marker in old_markers:
            try:
                req = RemoveEntity.Request()
                req.name = marker
                fut = cli.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
            except Exception:
                pass


# --------------------------------------------------------------------------- #
def main() -> None:
    rclpy.init()
    node = WaypointSetter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
