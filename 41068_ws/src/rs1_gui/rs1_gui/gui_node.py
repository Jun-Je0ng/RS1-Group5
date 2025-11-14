#!/usr/bin/env python3
import sys, math
from dataclasses import dataclass
from typing import List

import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

from PySide6 import QtCore, QtWidgets, QtGui
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import cv2
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import time
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from rclpy.task import Future
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2  # ROS2 helper to decode PointCloud2
from ros_ign_interfaces.srv import SetEntityPose
from ros_ign_interfaces.msg import Entity
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# Default path-planning waypoints (x, y, yaw_deg). Adjust to match your trail.
DEFAULT_TRAIL = [
    (20.793, 59.426, math.degrees(-1.472)),
    (29.696, 51.403, math.degrees(-1.038)),
]


# ---------- helpers ----------
def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


@dataclass
class Topics:
    cmd_vel: str = '/cmd_vel'
    frame_id: str = 'map'      # change to 'odom' if your global frame is odom
    scan_topic: str = '/scan'  # update if your lidar topic is different


# ---------- ROS backend ----------
class RosBackend(Node):
    def __init__(self, topics: Topics):
        super().__init__('rs1_gui_node')
        self.topics = topics

        # parameters describing initial spawn + gazebo entity
        self.initial_pose = (
            float(self.declare_parameter('initial_x', 0.0).value),
            float(self.declare_parameter('initial_y', 0.0).value),
            float(self.declare_parameter('initial_z', 0.0).value),
            float(self.declare_parameter('initial_yaw', 0.0).value)  # radians
        )
        self.world_name = self.declare_parameter('world', 'simple_trees').value
        self.entity_name = self.declare_parameter('entity_name', 'husky').value

        # publishers (teleop)
        self.cmd_pub = self.create_publisher(Twist, topics.cmd_vel, 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # subs (readouts)
        self.latest_pose = (0.0, 0.0, 0.0)  # x, y, yaw (rad)
        self.min_range = float('inf')
        self._nav_pose_ready = False
        self._nav_path_ready = False

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, topics.scan_topic, self.scan_cb, 10)

        # Nav2 action clients
        self.nav_to_pose_ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_through_poses_ac = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self._nav_ready_timer = self.create_timer(0.5, self._poll_nav_servers)
        self.reset_client = self.create_client(
            SetEntityPose, f'/world/{self.world_name}/set_entity_pose'
        )
        self.map_wp_client = self.create_client(
            GetParameters, '/map_waypoint_manager/get_parameters'
        )
        self.latest_external_targets: List[tuple] = []
        targets_qos = QoSProfile(depth=1)
        targets_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        targets_qos.reliability = ReliabilityPolicy.RELIABLE
        self.targets_sub = self.create_subscription(
            PoseArray, '/maintenance/targets', self._targets_cb, targets_qos
        )

    # ---- sensors ----
    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(t3, t4)
        self.latest_pose = (x, y, yaw)

    def scan_cb(self, msg: LaserScan):
        vals = [r for r in msg.ranges if r == r and r != float('inf')]
        self.min_range = min(vals) if vals else float('inf')

    # ---- teleop ----
    def publish_cmd(self, lin: float, ang: float, vert: float = 0.0):
        msg = Twist()
        msg.linear.x = lin
        msg.linear.z = vert
        msg.angular.z = ang
        self.cmd_pub.publish(msg)
    
    def emergency_stop(self):
        self._cancel_all_nav_goals()
        self.publish_cmd(0.0, 0.0, 0.0)

    # ---- Nav2 actions ----
    def _poll_nav_servers(self):
        if not self._nav_pose_ready and self.nav_to_pose_ac.server_is_ready():
            self._nav_pose_ready = True
            self.get_logger().info('NavigateToPose action server ready.')
        if not self._nav_path_ready and self.nav_through_poses_ac.server_is_ready():
            self._nav_path_ready = True
            self.get_logger().info('NavigateThroughPoses action server ready.')
        if self._nav_pose_ready and self._nav_path_ready:
            # Both servers available; stop polling
            self._nav_ready_timer.cancel()

    def send_nav_goal(self, x: float, y: float, yaw_deg: float) -> Future:
        outer = Future()
        if not self._nav_pose_ready:
            outer.set_result((False, 'NavigateToPose action server not ready yet'))
            return outer
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self.topics.frame_id
        goal.pose.pose.position = Point(x=x, y=y, z=0.0)
        goal.pose.pose.orientation = yaw_to_quat(math.radians(yaw_deg))

        send_future = self.nav_to_pose_ac.send_goal_async(goal, feedback_callback=self._nav_feedback)
        send_future.add_done_callback(lambda fut: self._handle_nav_goal_response(fut, outer))
        return outer

    def send_nav_through(self, pts: List[tuple]) -> Future:
        outer = Future()
        if not self._nav_path_ready:
            outer.set_result((False, 'NavigateThroughPoses action server not ready yet'))
            return outer
        g = NavigateThroughPoses.Goal()
        g.poses = []
        for (x, y, yaw_deg) in pts:
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = self.topics.frame_id
            ps.pose.position = Point(x=x, y=y, z=0.0)
            ps.pose.orientation = yaw_to_quat(math.radians(yaw_deg))
            g.poses.append(ps)

        send_future = self.nav_through_poses_ac.send_goal_async(g, feedback_callback=self._nav_feedback)
        send_future.add_done_callback(lambda fut: self._handle_nav_goal_response(fut, outer))
        return outer

    def reset_robot(self) -> Future:
        outer = Future()
        # Ensure nav is halted before teleport
        self._cancel_all_nav_goals()
        if not self.reset_client.service_is_ready():
            try:
                ready = self.reset_client.wait_for_service(timeout_sec=0.5)
            except Exception:
                ready = False
            if not ready:
                outer.set_result((False, 'Reset service not available (check gazebo bridge)'))
                return outer

        req = SetEntityPose.Request()
        req.entity = Entity()
        req.entity.name = self.entity_name
        req.entity.type = Entity.MODEL
        req.pose.position.x = self.initial_pose[0]
        req.pose.position.y = self.initial_pose[1]
        req.pose.position.z = self.initial_pose[2]
        req.pose.orientation = yaw_to_quat(self.initial_pose[3])

        future = self.reset_client.call_async(req)
        future.add_done_callback(lambda fut: self._handle_reset_response(fut, outer))
        return outer
    
    def fetch_trail_from_manager(self) -> Future:
        outer = Future()
        if self.map_wp_client is None:
            outer.set_result((False, 'No parameter service client for map_waypoint_manager'))
            return outer
        if not self.map_wp_client.service_is_ready():
            try:
                ready = self.map_wp_client.wait_for_service(timeout_sec=0.5)
            except Exception:
                ready = False
            if not ready:
                outer.set_result((False, 'map_waypoint_manager parameters unavailable'))
                return outer

        req = GetParameters.Request()
        req.names = ['manual_waypoints']
        future = self.map_wp_client.call_async(req)
        future.add_done_callback(lambda fut: self._handle_trail_param(fut, outer))
        return outer

    def _targets_cb(self, msg: PoseArray) -> None:
        pts: List[tuple] = []
        for pose in msg.poses:
            x = float(pose.position.x)
            y = float(pose.position.y)
            q = pose.orientation
            yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                          1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            pts.append((x, y, yaw))
        if pts:
            self.latest_external_targets = pts
            self.get_logger().info(f"Received {len(pts)} external targets on {msg.header.frame_id}.")

    def get_external_targets(self) -> List[tuple]:
        return list(self.latest_external_targets)

    def _handle_nav_goal_response(self, send_future, outer: Future):
        try:
            goal_handle = send_future.result()
        except Exception as exc:  # pragma: no cover - defensive logging path
            outer.set_result((False, f'Failed to send goal: {exc}'))
            return

        if goal_handle is None or not goal_handle.accepted:
            outer.set_result((False, 'Goal rejected by server'))
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._handle_nav_result(fut, outer))

    def _handle_nav_result(self, result_future, outer: Future):
        try:
            result = result_future.result()
        except Exception as exc:  # pragma: no cover - defensive logging path
            outer.set_result((False, f'Failed to get result: {exc}'))
            return

        status = getattr(result, 'status', 0)
        ok = (status == 4)
        outer.set_result((ok, 'SUCCEEDED' if ok else f'Ended with status={status}'))

    def _nav_feedback(self, feedback_msg):
        # Hook for live updates if you want (distance remaining, etc.)
        # feedback_msg.feedback is NavigateToPose_Feedback / NavigateThroughPoses_Feedback
        pass

    def _cancel_all_nav_goals(self):
        try:
            if self._nav_pose_ready:
                self.nav_to_pose_ac.cancel_all_goals_async()
            if self._nav_path_ready:
                self.nav_through_poses_ac.cancel_all_goals_async()
        except Exception:
            pass

    def _handle_reset_response(self, future, outer: Future):
        try:
            result = future.result()
        except Exception as exc:
            outer.set_result((False, f'Reset failed: {exc}'))
            return

        success = getattr(result, 'success', True)
        if not success:
            msg = getattr(result, 'status_message', 'reset service returned failure')
            outer.set_result((False, msg))
            return

        # publish zero velocity and reset initial pose for nav2
        self.publish_cmd(0.0, 0.0, 0.0)
        self._publish_initialpose()
        outer.set_result((True, 'Robot reset to spawn pose'))

    def _publish_initialpose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.topics.frame_id
        msg.pose.pose.position.x = self.initial_pose[0]
        msg.pose.pose.position.y = self.initial_pose[1]
        msg.pose.pose.position.z = self.initial_pose[2]
        msg.pose.pose.orientation = yaw_to_quat(self.initial_pose[3])

        # Simple diagonal covariance: ~10 cm position, 10 deg heading
        cov = msg.pose.covariance
        cov[0] = cov[7] = cov[14] = 0.1 * 0.1
        cov[35] = math.radians(10.0) ** 2
        self.initialpose_pub.publish(msg)
        # --- camera subscription management ---
    
    def _handle_trail_param(self, future, outer: Future):
        try:
            resp = future.result()
        except Exception as exc:
            outer.set_result((False, f'Failed to query map waypoints: {exc}'))
            return

        if not hasattr(resp, 'values') or len(resp.values) == 0:
            outer.set_result((False, 'map_waypoint_manager returned no manual_waypoints'))
            return

        param = resp.values[0]
        if getattr(param, 'type', ParameterType.PARAMETER_NOT_SET) != ParameterType.PARAMETER_DOUBLE_ARRAY:
            outer.set_result((False, 'manual_waypoints parameter is not a double array'))
            return

        values = list(getattr(param, 'double_array_value', []))
        if not values:
            outer.set_result((False, 'map_waypoint_manager returned no manual_waypoints'))
            return

        if len(values) % 3 != 0:
            self.get_logger().warn('manual_waypoints length not divisible by 3; ignoring trailing values')
        triplets = []
        for i in range(0, len(values) - 2, 3):
            x = float(values[i])
            y = float(values[i+1])
            yaw_deg = math.degrees(float(values[i+2]))
            triplets.append((x, y, yaw_deg))

        if not triplets:
            outer.set_result((False, 'Parsed trail is empty'))
            return

        outer.set_result((True, triplets))
    def start_camera_sub(self, topic: str):
        # kill any existing sub
        try:
            if hasattr(self, "_cam_sub") and self._cam_sub is not None:
                self.destroy_subscription(self._cam_sub)
        except Exception:
            pass

        self._bridge = getattr(self, "_bridge", CvBridge())
        self._latest_bgr = None
        self._cam_topic = topic

        # Prefer raw Image; if user points at a compressed topic, we'll handle it too
        if topic.endswith("/compressed"):
            self._cam_sub = self.create_subscription(
                CompressedImage, topic, self._cam_compressed_cb, 10
            )
        else:
            self._cam_sub = self.create_subscription(
                Image, topic, self._cam_raw_cb, 10
            )

    def _cam_raw_cb(self, msg: Image):
        # Convert to BGR8 if needed
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._latest_bgr = img
        except Exception:
            self._latest_bgr = None

    def _cam_compressed_cb(self, msg: CompressedImage):
        try:
            # Decode JPEG/PNG buffer to BGR
            data = np.frombuffer(msg.data, dtype=np.uint8)
            img = cv2.imdecode(data, cv2.IMREAD_COLOR)
            self._latest_bgr = img
        except Exception:
            self._latest_bgr = None

    def get_latest_bgr(self):
        return getattr(self, "_latest_bgr", None)
    
    # --- LiDAR subscription management ---
    def start_lidar_sub(self, topic: str):
        # kill any existing sub for lidar
        try:
            if hasattr(self, "_lidar_sub") and self._lidar_sub is not None:
                self.destroy_subscription(self._lidar_sub)
        except Exception:
            pass
        self._lidar_topic = topic
        self._latest_scan_pts_xy = None  # Nx2 float array in meters (x,y)
        self._lidar_sub = self.create_subscription(
            LaserScan, topic, self._lidar_cb, 10
        )

    def _lidar_cb(self, msg: LaserScan):
        # cache min range for your label (you already show this)
        vals = [r for r in msg.ranges if r == r and np.isfinite(r)]
        self.min_range = min(vals) if vals else float('inf')

        # convert to XY points in laser frame
        angle = msg.angle_min
        inc = msg.angle_increment
        ranges = np.asarray(msg.ranges, dtype=np.float32)

        # mask invalid
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        if not np.any(valid):
            self._latest_scan_pts_xy = None
            return

        r = ranges[valid]
        a = angle + inc * np.nonzero(valid)[0]
        # polar -> cart
        x = r * np.cos(a)
        y = r * np.sin(a)
        self._latest_scan_pts_xy = np.stack([x, y], axis=1)

    def get_latest_scan_xy(self):
        return getattr(self, "_latest_scan_pts_xy", None)
    
    def start_cloud_sub(self, topic: str):
        try:
            if hasattr(self, "_cloud_sub") and self._cloud_sub is not None:
                self.destroy_subscription(self._cloud_sub)
        except Exception:
            pass

        self._cloud_topic = topic
        self._latest_cloud_xyz = None      # Nx3 (float32)
        self._latest_cloud_intensity = None  # optional, Nx1
        self._cloud_sub = self.create_subscription(
            PointCloud2, topic, self._cloud_cb, 5
        )

    def _cloud_cb(self, msg: PointCloud2):
        # Decode x,y,z (+ intensity if present) using ROS helper
        has_intensity = any(f.name == "intensity" for f in msg.fields)
        try:
            if has_intensity:
                pts = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
                xyz = []
                inten = []
                for x, y, z, i in pts:
                    xyz.append((x, y, z))
                    inten.append(i)
                self._latest_cloud_xyz = np.asarray(xyz, dtype=np.float32) if xyz else None
                self._latest_cloud_intensity = np.asarray(inten, dtype=np.float32) if inten else None
            else:
                pts = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
                xyz = list(pts)
                self._latest_cloud_xyz = np.asarray(xyz, dtype=np.float32) if xyz else None
                self._latest_cloud_intensity = None
        except Exception:
            self._latest_cloud_xyz = None
            self._latest_cloud_intensity = None

    def get_latest_cloud(self):
        # returns (xyz Nx3, intensity Nx1 or None)
        return getattr(self, "_latest_cloud_xyz", None), getattr(self, "_latest_cloud_intensity", None)


class CameraViewer(QtWidgets.QWidget):
    """Simple Qt widget that shows a ROS image topic in real-time."""
    def __init__(self, node: RosBackend, default_topic="/camera/image"):
        super().__init__()
        self.node = node

        self.setLayout(QtWidgets.QVBoxLayout())
        ctrl = QtWidgets.QHBoxLayout()
        self.topic_edit = QtWidgets.QLineEdit(default_topic)
        self.btn_set = QtWidgets.QPushButton("Subscribe")
        self.fps_label = QtWidgets.QLabel("— fps")
        ctrl.addWidget(QtWidgets.QLabel("Topic:"))
        ctrl.addWidget(self.topic_edit, 1)
        ctrl.addWidget(self.btn_set)
        ctrl.addStretch(1)
        ctrl.addWidget(self.fps_label)
        self.layout().addLayout(ctrl)

        self.view = QtWidgets.QLabel("No image")
        self.view.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.view.setMinimumHeight(180)
        self.view.setStyleSheet("background:#111; color:#aaa;")
        self.layout().addWidget(self.view, 1)

        self.btn_set.clicked.connect(self._apply_topic)

        # start
        self._apply_topic()

        # UI refresh timer
        self._t = QtCore.QTimer(self)
        self._t.timeout.connect(self._draw_latest)
        self._t.start(33)  # ~30 Hz

        self._last_ts = None
        self._fps_acc = 0
        self._fps_n = 0

    def _apply_topic(self):
        topic = self.topic_edit.text().strip()
        if topic:
            self.node.start_camera_sub(topic)

    def _draw_latest(self):
        img = self.node.get_latest_bgr()
        if img is None:
            return
        # BGR -> RGB for Qt
        rgb = img[:, :, ::-1].copy()
        h, w, ch = rgb.shape
        qimg = QtGui.QImage(rgb.data, w, h, ch * w, QtGui.QImage.Format.Format_RGB888)
        self.view.setPixmap(QtGui.QPixmap.fromImage(qimg).scaled(
            self.view.size(),
            QtCore.Qt.AspectRatioMode.KeepAspectRatio,
            QtCore.Qt.TransformationMode.SmoothTransformation
        ))

        # crude fps
        now = QtCore.QTime.currentTime()
        if self._last_ts is None:
            self._last_ts = now
            return
        dt_ms = self._last_ts.msecsTo(now)
        self._last_ts = now
        if dt_ms > 0:
            fps = 1000.0 / dt_ms
            self._fps_acc += fps
            self._fps_n += 1
            if self._fps_n >= 10:
                self.fps_label.setText(f"{self._fps_acc / self._fps_n:.1f} fps")
                self._fps_acc = 0
                self._fps_n = 0


class LidarViewer(QtWidgets.QWidget):
    """
    Simple 2D LiDAR viewer using QPainter.
    Origin at center; +x to the right, +y up. Points plotted in meters with a zoom.
    """
    def __init__(self, node: RosBackend, default_topic="/scan"):
        super().__init__()
        self.node = node
        self._zoom = 10.0  # pixels per meter (bigger => zoom in)
        self._max_points = 5000

        # UI
        v = QtWidgets.QVBoxLayout(self)
        ctrl = QtWidgets.QHBoxLayout()
        self.topic_edit = QtWidgets.QLineEdit(default_topic)
        self.btn_set = QtWidgets.QPushButton("Subscribe")
        self.zoom_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.zoom_slider.setRange(10, 200)   # px/m
        self.zoom_slider.setValue(int(self._zoom))
        self.zoom_label = QtWidgets.QLabel(f"Zoom: {self._zoom:.0f} px/m")
        ctrl.addWidget(QtWidgets.QLabel("LiDAR Topic:"))
        ctrl.addWidget(self.topic_edit, 1)
        ctrl.addWidget(self.btn_set)
        ctrl.addStretch(1)
        ctrl.addWidget(self.zoom_label)
        ctrl.addWidget(self.zoom_slider)
        v.addLayout(ctrl)

        self.view = _LidarCanvas(self)
        self.view.setMinimumHeight(180)
        v.addWidget(self.view, 1)

        self.btn_set.clicked.connect(self._apply_topic)
        self.zoom_slider.valueChanged.connect(self._set_zoom)

        # Start sub
        self._apply_topic()

        # Repaint timer
        self._t = QtCore.QTimer(self)
        self._t.timeout.connect(self._tick)
        self._t.start(33)  # ~30 fps

    def _apply_topic(self):
        topic = self.topic_edit.text().strip()
        if topic:
            self.node.start_lidar_sub(topic)

    def _set_zoom(self, v):
        self._zoom = float(v)
        self.zoom_label.setText(f"Zoom: {self._zoom:.0f} px/m")

    def _tick(self):
        # pull latest data; pass to canvas
        pts = self.node.get_latest_scan_xy()
        if pts is not None and len(pts) > self._max_points:
            pts = pts[:: int(np.ceil(len(pts) / self._max_points))]
        self.view.set_points(pts, self._zoom)
        self.view.update()


class _LidarCanvas(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._pts = None
        self._zoom = 10.0
        self.setAutoFillBackground(True)
        pal = self.palette()
        pal.setColor(self.backgroundRole(), QtGui.QColor(17, 17, 17))
        self.setPalette(pal)

    def set_points(self, pts_xy: np.ndarray | None, zoom: float):
        self._pts = pts_xy
        self._zoom = zoom

    def paintEvent(self, ev: QtGui.QPaintEvent):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)

        w = self.width()
        h = self.height()
        cx, cy = w / 2.0, h / 2.0

        # draw grid
        pen_grid = QtGui.QPen(QtGui.QColor(60, 60, 60))
        pen_grid.setStyle(QtCore.Qt.PenStyle.DotLine)
        painter.setPen(pen_grid)
        # 1 m rings
        max_r_pix = int(min(w, h) / 2) - 5
        meters = int(max_r_pix / self._zoom)
        for m in range(1, meters + 1):
            r = m * self._zoom
            painter.drawEllipse(QtCore.QPointF(cx, cy), r, r)

        # axes
        pen_axes = QtGui.QPen(QtGui.QColor(120, 120, 120))
        painter.setPen(pen_axes)
        painter.drawLine(0, cy, w, cy)
        painter.drawLine(cx, 0, cx, h)

        # robot center
        painter.setBrush(QtGui.QColor(180, 180, 180))
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.drawEllipse(QtCore.QPointF(cx, cy), 4, 4)

        # points
        if self._pts is not None and len(self._pts) > 0:
            pen_pts = QtGui.QPen(QtGui.QColor(0, 220, 255))
            pen_pts.setWidth(2)
            painter.setPen(pen_pts)
            # transform: meters -> pixels, y up -> screen y down
            for x, y in self._pts:
                sx = cx + x * self._zoom
                sy = cy - y * self._zoom
                painter.drawPoint(int(sx), int(sy))

        painter.end()

class PointCloudViewer(QtWidgets.QWidget):
    """
    Lightweight 2D viewer for PointCloud2.
    Shows a projection (XY, XZ, or YZ), with zoom and topic controls.
    Colors by intensity if available; otherwise a solid color.
    """
    def __init__(self, node: RosBackend, default_topic="/camera/depth/points", default_proj="XY"):
        super().__init__()
        self.node = node
        self._zoom = 10.0               # pixels per meter
        self._max_points = 20000        # cap for speed
        self._proj = default_proj       # "XY" | "XZ" | "YZ"

        v = QtWidgets.QVBoxLayout(self)

        # Controls
        ctrl = QtWidgets.QHBoxLayout()
        self.topic_edit = QtWidgets.QLineEdit(default_topic)
        self.btn_set = QtWidgets.QPushButton("Subscribe")
        self.zoom_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.zoom_slider.setRange(10, 300); self.zoom_slider.setValue(int(self._zoom))
        self.zoom_label = QtWidgets.QLabel(f"Zoom: {self._zoom:.0f} px/m")
        self.proj_combo = QtWidgets.QComboBox()
        self.proj_combo.addItems(["XY", "XZ", "YZ"])
        self.proj_combo.setCurrentText(self._proj)
        ctrl.addWidget(QtWidgets.QLabel("Cloud Topic:"))
        ctrl.addWidget(self.topic_edit, 1)
        ctrl.addWidget(self.btn_set)
        ctrl.addStretch(1)
        ctrl.addWidget(QtWidgets.QLabel("Proj:"))
        ctrl.addWidget(self.proj_combo)
        ctrl.addWidget(self.zoom_label)
        ctrl.addWidget(self.zoom_slider)
        v.addLayout(ctrl)

        # Canvas
        self.view = _CloudCanvas(self)
        self.view.setMinimumHeight(160)
        v.addWidget(self.view, 1)

        # Hooks
        self.btn_set.clicked.connect(self._apply_topic)
        self.zoom_slider.valueChanged.connect(self._set_zoom)
        self.proj_combo.currentTextChanged.connect(self._set_proj)

        # Start
        self._apply_topic()

        # Timer
        self._t = QtCore.QTimer(self)
        self._t.timeout.connect(self._tick)
        self._t.start(33)

    def _apply_topic(self):
        topic = self.topic_edit.text().strip()
        if topic:
            self.node.start_cloud_sub(topic)

    def _set_zoom(self, v):
        self._zoom = float(v)
        self.zoom_label.setText(f"Zoom: {self._zoom:.0f} px/m")

    def _set_proj(self, p):
        self._proj = p

    def _tick(self):
        xyz, intensity = self.node.get_latest_cloud()
        if xyz is None or len(xyz) == 0:
            self.view.set_points(None, None, self._proj, self._zoom)
            self.view.update()
            return

        # Downsample if needed (uniform stride)
        n = len(xyz)
        if n > self._max_points:
            step = int(np.ceil(n / self._max_points))
            xyz = xyz[::step]
            if intensity is not None:
                intensity = intensity[::step]

        self.view.set_points(xyz, intensity, self._proj, self._zoom)
        self.view.update()


class _CloudCanvas(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._xyz = None
        self._intensity = None
        self._proj = "XY"
        self._zoom = 10.0
        self.setAutoFillBackground(True)
        pal = self.palette()
        pal.setColor(self.backgroundRole(), QtGui.QColor(17, 17, 17))
        self.setPalette(pal)

    def set_points(self, xyz: np.ndarray | None, intensity: np.ndarray | None, proj: str, zoom: float):
        self._xyz = xyz
        self._intensity = intensity
        self._proj = proj
        self._zoom = zoom

    def paintEvent(self, ev: QtGui.QPaintEvent):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
        w, h = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0

        # grid rings
        pen_grid = QtGui.QPen(QtGui.QColor(60, 60, 60)); pen_grid.setStyle(QtCore.Qt.PenStyle.DotLine)
        painter.setPen(pen_grid)
        max_r_pix = int(min(w, h) / 2) - 5
        meters = int(max_r_pix / self._zoom)
        for m in range(1, meters + 1):
            r = m * self._zoom
            painter.drawEllipse(QtCore.QPointF(cx, cy), r, r)

        # axes
        pen_axes = QtGui.QPen(QtGui.QColor(120, 120, 120)); painter.setPen(pen_axes)
        painter.drawLine(0, cy, w, cy)
        painter.drawLine(cx, 0, cx, h)

        # origin
        painter.setBrush(QtGui.QColor(180, 180, 180))
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.drawEllipse(QtCore.QPointF(cx, cy), 4, 4)

        if self._xyz is not None and len(self._xyz) > 0:
            # choose projection indices
            if self._proj == "XY":
                a, b = 0, 1   # x, y
            elif self._proj == "XZ":
                a, b = 0, 2   # x, z
            else:  # "YZ"
                a, b = 1, 2   # y, z

            # intensity color mapping (simple grayscale)
            if self._intensity is not None and len(self._intensity) == len(self._xyz):
                i = self._intensity
                # normalize to 0..255
                if np.ptp(i) > 1e-6:
                    i_norm = ((i - i.min()) / (i.max() - i.min()) * 255.0).astype(np.uint8)
                else:
                    i_norm = np.full_like(i, 200, dtype=np.uint8)
                # draw
                for (x, y, z), ii in zip(self._xyz, i_norm):
                    sx = cx + self._xyz.dtype.type(self._xyz[:, a][0]).item().__class__(x if a==0 else (y if a==1 else z)) * self._zoom  # ensure float
                    sy = cy - (self._xyz[:, b][0] if False else ( [x,y,z][b] )) * self._zoom  # placeholder, we’ll compute below (cleaner way below)
                # Cleaner loop:
                painter.setPen(QtGui.QPen(QtGui.QColor(0, 220, 255)))
                for p, ii in zip(self._xyz, i_norm):
                    sx = cx + float(p[a]) * self._zoom
                    sy = cy - float(p[b]) * self._zoom
                    painter.setPen(QtGui.QPen(QtGui.QColor(ii, ii, ii)))
                    painter.drawPoint(int(sx), int(sy))
            else:
                # solid cyan
                pen_pts = QtGui.QPen(QtGui.QColor(0, 220, 255)); pen_pts.setWidth(2)
                painter.setPen(pen_pts)
                for p in self._xyz:
                    sx = cx + float(p[a]) * self._zoom
                    sy = cy - float(p[b]) * self._zoom
                    painter.drawPoint(int(sx), int(sy))

        painter.end()


class Gui(QtWidgets.QWidget):
    def __init__(self, node: RosBackend):
        super().__init__()
        self.node = node
        self.setWindowTitle('RS1 Waypoint GUI (Nav2)')
        self.resize(1100, 640)
        self.setMinimumSize(900, 540)
        self._apply_styles()

        # ----- LEFT: controls panel -----
        left = QtWidgets.QFrame()
        left.setObjectName('controlPanel')
        grid = QtWidgets.QGridLayout(left)
        grid.setContentsMargins(18, 18, 18, 18)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(10)
        r = 0

        # Teleop
        self.lin_slider = self._slider(-200, 200, 0)
        self.vert_slider = self._slider(-200, 200, 0)
        self.ang_slider = self._slider(-300, 300, 0)
        self.lin_label = QtWidgets.QLabel('Linear: 0.00 m/s')
        self.vert_label = QtWidgets.QLabel('Vertical: 0.00 m/s')
        self.ang_label = QtWidgets.QLabel('Angular: 0.00 rad/s')
        self.btn_zero  = QtWidgets.QPushButton('Zero Velocity')

        grid.addWidget(QtWidgets.QLabel('Teleop (optional)'), r, 0, 1, 6); r += 1
        grid.addWidget(self.lin_label, r, 0, 1, 2); grid.addWidget(self.lin_slider, r, 2, 1, 4); r += 1
        grid.addWidget(self.vert_label, r, 0, 1, 2); grid.addWidget(self.vert_slider, r, 2, 1, 4); r += 1
        grid.addWidget(self.ang_label, r, 0, 1, 2); grid.addWidget(self.ang_slider, r, 2, 1, 4); r += 1
        grid.addWidget(self.btn_zero, r, 0, 1, 6); r += 1

        # Single waypoint
        self.x_in = self._dspin(-1e6, 1e6, 8.0, 0.01)
        self.y_in = self._dspin(-1e6, 1e6, 62.0, 0.01)
        self.yaw_in = self._dspin(-360, 360, 0.0, 0.1)
        self.btn_send_wp = QtWidgets.QPushButton('Go To Pose')
        grid.addWidget(QtWidgets.QLabel('Single Goal (map frame)'), r, 0, 1, 6); r += 1
        grid.addWidget(QtWidgets.QLabel('x'), r, 0); grid.addWidget(self.x_in, r, 1)
        grid.addWidget(QtWidgets.QLabel('y'), r, 2); grid.addWidget(self.y_in, r, 3)
        grid.addWidget(QtWidgets.QLabel('yaw°'), r, 4); grid.addWidget(self.yaw_in, r, 5); r += 1
        grid.addWidget(self.btn_send_wp, r, 0, 1, 6); r += 1

        # Multi-waypoint
        self.path_edit = QtWidgets.QPlainTextEdit()
        self.path_edit.setObjectName('pathEditor')
        self.path_edit.setMinimumHeight(100)
        self.path_edit.setPlaceholderText("x,y,yaw_deg per line")
        self.btn_plan_trail = QtWidgets.QPushButton('Start Path Planning')
        self.btn_add_trail = QtWidgets.QPushButton('Add To Trail')
        self.btn_clear_trail = QtWidgets.QPushButton('Clear Trail')
        self.btn_start_follower = QtWidgets.QPushButton('Start Path Follower')
        path_btns_top = QtWidgets.QHBoxLayout()
        path_btns_top.setSpacing(8)
        path_btns_top.addWidget(self.btn_plan_trail)
        path_btns_top.addWidget(self.btn_add_trail)
        path_btns_top.addWidget(self.btn_clear_trail)
        path_btns_bottom = QtWidgets.QHBoxLayout()
        path_btns_bottom.setSpacing(8)
        path_btns_bottom.addStretch(1)
        path_btns_bottom.addWidget(self.btn_start_follower)
        grid.addWidget(QtWidgets.QLabel('Waypoints'), r, 0, 1, 6); r += 1
        grid.addWidget(self.path_edit, r, 0, 1, 6); r += 1
        grid.addLayout(path_btns_top, r, 0, 1, 6); r += 1
        grid.addLayout(path_btns_bottom, r, 0, 1, 6); r += 1

        # Readouts
        self.pose_label = QtWidgets.QLabel('Pose: x=0.00, y=0.00, yaw=0.00°')
        self.scan_label = QtWidgets.QLabel('Lidar min: ∞ m')
        self.status     = QtWidgets.QLabel('Status: idle')
        grid.addWidget(self.pose_label, r, 0, 1, 6); r += 1
        grid.addWidget(self.scan_label, r, 0, 1, 6); r += 1
        grid.addWidget(self.status, r, 0, 1, 6); r += 1
        reset_row = QtWidgets.QHBoxLayout()
        reset_row.setSpacing(8)
        self.btn_reset = QtWidgets.QPushButton('Reset Robot')
        self.btn_estop = QtWidgets.QPushButton('Emergency Stop')
        self.btn_estop.setStyleSheet('background-color: #d1524b; color: #ffffff; font-weight: 700;')
        reset_row.addWidget(self.btn_reset)
        reset_row.addWidget(self.btn_estop)
        grid.addLayout(reset_row, r, 0, 1, 6); r += 1
        self.pose_detail = QtWidgets.QPlainTextEdit()
        self.pose_detail.setReadOnly(True)
        self.pose_detail.setMaximumHeight(100)
        self.pose_detail.setObjectName('poseDetail')
        grid.addWidget(self.pose_detail, r, 0, 1, 6); r += 1

        # ----- RIGHT: Camera (top) + LiDAR (bottom) with a vertical splitter -----
        right = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)

        # Camera
        cam_group = QtWidgets.QGroupBox("Camera")
        cam_group.setObjectName('cameraGroup')
        cam_v = QtWidgets.QVBoxLayout(cam_group)
        self.cam = CameraViewer(self.node, default_topic="/camera/image")  # <- your new topic
        cam_v.addWidget(self.cam, 1)

        # LiDAR
        lidar_group = QtWidgets.QGroupBox("LiDAR")
        lidar_v = QtWidgets.QVBoxLayout(lidar_group)
        self.lidar = LidarViewer(self.node, default_topic=self.node.topics.scan_topic)  # usually '/scan'
        lidar_v.addWidget(self.lidar, 1)

        # Point Cloud
        cloud_group = QtWidgets.QGroupBox("Point Cloud")
        cloud_v = QtWidgets.QVBoxLayout(cloud_group)
        self.cloud = PointCloudViewer(self.node, default_topic="/camera/depth/points", default_proj="XY")
        cloud_v.addWidget(self.cloud, 1)

        right.addWidget(cam_group)
        right.addWidget(lidar_group)
        right.addWidget(cloud_group)
        right.setSizes([260, 200, 160])  # initial heights (px); adjust as you like
        right.setStretchFactor(0, 1)
        right.setStretchFactor(1, 1)
        right.setStretchFactor(2, 1)

        # ----- Main horizontal splitter -----
        main_split = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        controls_scroll = QtWidgets.QScrollArea()
        controls_scroll.setWidgetResizable(True)
        controls_scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        controls_scroll.setWidget(left)
        main_split.addWidget(controls_scroll)  # controls (scrollable for small screens)
        main_split.addWidget(right)            # camera+lidar
        main_split.setStretchFactor(0, 0)
        main_split.setStretchFactor(1, 1)

        outer = QtWidgets.QVBoxLayout(self)
        outer.addWidget(main_split, 1)

        # Signals
        self.lin_slider.valueChanged.connect(self._lin_changed)
        self.vert_slider.valueChanged.connect(self._vert_changed)
        self.ang_slider.valueChanged.connect(self._ang_changed)
        self.btn_zero.clicked.connect(self._zero)
        self.btn_send_wp.clicked.connect(self._send_wp)
        self.btn_plan_trail.clicked.connect(self._load_default_trail)
        self.btn_add_trail.clicked.connect(self._add_trail_waypoint)
        self.btn_clear_trail.clicked.connect(self._clear_trail)
        self.btn_start_follower.clicked.connect(self._send_path)
        self.btn_reset.clicked.connect(self._reset_robot)
        self.btn_estop.clicked.connect(self._estop)

        # Timers
        self.cmd_timer = QtCore.QTimer(self)
        self.cmd_timer.timeout.connect(self._tick_cmd)
        self.cmd_timer.start(50)  # 20 Hz

        self.ui_timer = QtCore.QTimer(self)
        self.ui_timer.timeout.connect(self._refresh_ui)
        self.ui_timer.start(100)  # 10 Hz


    def _apply_styles(self):
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_StyledBackground, True)
        pal = self.palette()
        pal.setColor(QtGui.QPalette.ColorRole.Window, QtGui.QColor('#0f111a'))
        pal.setColor(QtGui.QPalette.ColorRole.Base, QtGui.QColor('#161826'))
        pal.setColor(QtGui.QPalette.ColorRole.Text, QtGui.QColor('#f0f3ff'))
        pal.setColor(QtGui.QPalette.ColorRole.WindowText, QtGui.QColor('#f0f3ff'))
        self.setPalette(pal)

        self.setStyleSheet("""
            QWidget {
                font-family: 'Segoe UI', 'Ubuntu', sans-serif;
                font-size: 13px;
                color: #f0f3ff;
            }
            QFrame#controlPanel {
                background-color: #161826;
                border-radius: 18px;
                border: 1px solid #22263a;
            }
            QGroupBox {
                background-color: #161826;
                border: 1px solid #22263a;
                border-radius: 16px;
                margin-top: 16px;
                padding: 14px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 18px;
                padding: 0 4px;
                color: #8aa5ff;
                font-weight: 600;
            }
            QLabel {
                color: #e6e9ff;
            }
            QPushButton {
                background-color: #2f6bff;
                border-radius: 10px;
                padding: 8px 14px;
                border: none;
                color: #ffffff;
                font-weight: 600;
            }
            QPushButton:hover {
                background-color: #4b83ff;
            }
            QPushButton:pressed {
                background-color: #2556d6;
            }
            QPushButton:disabled {
                background-color: #364a80;
                color: #9caacf;
            }
            QLineEdit,
            QPlainTextEdit,
            QDoubleSpinBox,
            QComboBox,
            QTextEdit {
                background-color: #1f2233;
                border-radius: 10px;
                border: 1px solid #2a3048;
                padding: 6px 8px;
                selection-background-color: #4b83ff;
            }
            QPlainTextEdit#pathEditor {
                font-family: 'JetBrains Mono', 'Fira Code', monospace;
            }
            QPlainTextEdit#poseDetail {
                font-family: 'JetBrains Mono', 'Fira Code', monospace;
                background-color: #161826;
            }
            QSlider::groove:horizontal {
                height: 8px;
                background: #2a3048;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                width: 18px;
                background: #2f6bff;
                border: 2px solid #0f111a;
                border-radius: 9px;
                margin: -6px 0;
            }
            QScrollBar:vertical, QScrollBar:horizontal {
                background: #1b1f2e;
                border-radius: 8px;
                margin: 4px;
            }
            QScrollBar::handle:vertical, QScrollBar::handle:horizontal {
                background: #2f6bff;
                border-radius: 8px;
            }
            QSplitter::handle {
                background-color: #1f2233;
                border-radius: 3px;
            }
            QSplitter::handle:horizontal {
                height: 8px;
                margin: 6px 12px;
            }
            QSplitter::handle:vertical {
                width: 8px;
                margin: 12px 6px;
            }
        """)

    # ---- widgets helpers ----
    def _slider(self, mn, mx, val):
        s = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        s.setRange(mn, mx); s.setValue(val); s.setSingleStep(5)
        return s

    def _dspin(self, mn, mx, val, step):
        w = QtWidgets.QDoubleSpinBox()
        w.setRange(mn, mx); w.setValue(val)
        w.setDecimals(3 if step < 0.1 else 1)
        w.setSingleStep(step)
        return w

    
    # ---- teleop handlers ----
    def _lin_changed(self, v): self.lin_label.setText(f'Linear: {v/100.0:.2f} m/s')
    def _vert_changed(self, v): self.vert_label.setText(f'Vertical: {v/100.0:.2f} m/s')
    def _ang_changed(self, v): self.ang_label.setText(f'Angular: {v/100.0:.2f} rad/s')
    def _zero(self):
        self.lin_slider.setValue(0); self.vert_slider.setValue(0); self.ang_slider.setValue(0)
        self.node.publish_cmd(0.0, 0.0, 0.0)
    def _tick_cmd(self):
        lin = self.lin_slider.value()/100.0
        ang = self.ang_slider.value()/100.0
        vert = self.vert_slider.value()/100.0
        self.node.publish_cmd(lin, ang, vert)

    # ---- Nav2 actions handlers ----
    def _send_wp(self):
        x = float(self.x_in.value()); y = float(self.y_in.value()); yaw = float(self.yaw_in.value())
        self.status.setText('Status: sending NavigateToPose goal…')
        fut = self.node.send_nav_goal(x, y, yaw)
        def _done(_):
            ok, msg = fut.result()
            self.status.setText('Goal reached' if ok else f'NavigateToPose error: {msg}')
        fut.add_done_callback(_done)

    def _parse_path_text(self) -> List[tuple]:
        pts = []
        for line in self.path_edit.toPlainText().splitlines():
            line = line.strip()
            if not line or line.startswith('#'): continue
            parts = [p.strip() for p in line.split(',')]
            try:
                x = float(parts[0]); y = float(parts[1])
                yaw = float(parts[2]) if len(parts) >= 3 else 0.0
                pts.append((x, y, yaw))
            except Exception:
                continue
        return pts

    def _send_path(self):
        pts = self._parse_path_text()
        if not pts:
            self.status.setText('Status: no valid path lines')
            return
        self.status.setText(f'Status: sending {len(pts)} poses…')
        fut = self.node.send_nav_through(pts)
        def _done(_):
            ok, msg = fut.result()
            self.status.setText('Path completed' if ok else f'NavigateThroughPoses error: {msg}')
        fut.add_done_callback(_done)

    def _load_default_trail(self):
        ext = self.node.get_external_targets()
        if ext:
            self._set_trail_lines(ext)
            self.status.setText(f'Status: loaded {len(ext)} external targets')
            return

        self.status.setText('Status: requesting trail from map_waypoint_manager…')
        fut = self.node.fetch_trail_from_manager()
        def _done(_):
            ok, payload = fut.result()
            if ok:
                pts = payload
                self._set_trail_lines(pts)
                self.status.setText(f'Status: loaded {len(pts)} waypoints from map_waypoint_manager')
            else:
                self._set_trail_lines(DEFAULT_TRAIL)
                self.status.setText(f'Status: {payload}; using built-in trail ({len(DEFAULT_TRAIL)} poses)')
        fut.add_done_callback(_done)

    def _set_trail_lines(self, pts: List[tuple]):
        lines = [f"{x:.3f}, {y:.3f}, {yaw:.1f}" for (x, y, yaw) in pts]
        self.path_edit.setPlainText('\n'.join(lines))

    def _add_trail_waypoint(self):
        x = float(self.x_in.value())
        y = float(self.y_in.value())
        yaw = float(self.yaw_in.value())
        line = f"{x:.3f}, {y:.3f}, {yaw:.1f}"

        text = self.path_edit.toPlainText().strip()
        if text:
            self.path_edit.appendPlainText(line)
        else:
            self.path_edit.setPlainText(line)

        cursor = self.path_edit.textCursor()
        cursor.movePosition(QtGui.QTextCursor.MoveOperation.End)
        self.path_edit.setTextCursor(cursor)

        total = len(self._parse_path_text())
        self.status.setText(f'Status: added trail waypoint ({total} queued)')

    def _clear_trail(self):
        self.path_edit.clear()
        self.status.setText('Status: cleared trail waypoints')

    def _reset_robot(self):
        self._zero()
        self.lin_slider.setValue(0)
        self.vert_slider.setValue(0)
        self.ang_slider.setValue(0)
        self._load_default_trail()
        self.status.setText('Status: resetting robot to spawn…')
        fut = self.node.reset_robot()
        def _done(_):
            ok, msg = fut.result()
            self.status.setText('Robot reset complete' if ok else f'Reset failed: {msg}')
        fut.add_done_callback(_done)

    def _estop(self):
        self._zero()
        self.node.emergency_stop()
        self.status.setText('Status: emergency stop engaged')

    # ---- UI refresh ----
    def _refresh_ui(self):
        x, y, yaw = self.node.latest_pose
        yaw_deg = math.degrees(yaw)
        self.pose_label.setText(f'Pose: x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.2f}°')
        mr = self.node.min_range
        self.scan_label.setText(f'Lidar min: {"∞" if not math.isfinite(mr) else f"{mr:.2f} m"}')
        detail_lines = [
            f"Map pose: x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.2f}°"
        ]
        trail = self._parse_path_text()
        if trail:
            gx, gy, gyaw = trail[0]
            dx = gx - x
            dy = gy - y
            dist = math.hypot(dx, dy)
            detail_lines.append(f"Next goal: x={gx:.3f}, y={gy:.3f}, yaw={gyaw:.1f}°")
            detail_lines.append(f"Δ to goal: dx={dx:.3f}, dy={dy:.3f}, dist={dist:.3f} m")
        else:
            detail_lines.append("No planned trail loaded.")
        self.pose_detail.setPlainText('\n'.join(detail_lines))


# ---------- main ----------
def main():
    rclpy.init(args=None)
    node = RosBackend(Topics())

    app = QtWidgets.QApplication(sys.argv)
    gui = Gui(node); gui.show()

    # Spin ROS with Qt
    ros_timer = QtCore.QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    ros_timer.start(10)

    code = app.exec()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
