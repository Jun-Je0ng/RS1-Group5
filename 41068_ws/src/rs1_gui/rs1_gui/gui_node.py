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
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import time
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2  # ROS2 helper to decode PointCloud2


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

        # publishers (teleop)
        self.cmd_pub = self.create_publisher(Twist, topics.cmd_vel, 10)

        # subs (readouts)
        self.latest_pose = (0.0, 0.0, 0.0)  # x, y, yaw (rad)
        self.min_range = float('inf')

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, topics.scan_topic, self.scan_cb, 10)

        # Nav2 action clients
        self.nav_to_pose_ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_through_poses_ac = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

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
    def publish_cmd(self, lin: float, ang: float):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.cmd_pub.publish(msg)

    # ---- Nav2 actions ----
    async def ensure_servers(self):
        # wait for action servers to be ready (poll)
        if not self.nav_to_pose_ac.server_is_ready():
            await self.nav_to_pose_ac.wait_for_server()
        if not self.nav_through_poses_ac.server_is_ready():
            await self.nav_through_poses_ac.wait_for_server()

    async def send_nav_goal(self, x: float, y: float, yaw_deg: float):
        await self.ensure_servers()
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self.topics.frame_id
        goal.pose.pose.position = Point(x=x, y=y, z=0.0)
        goal.pose.pose.orientation = yaw_to_quat(math.radians(yaw_deg))

        send_future = self.nav_to_pose_ac.send_goal_async(goal, feedback_callback=self._nav_feedback)
        goal_handle = await send_future
        if not goal_handle.accepted:
            return False, 'Goal rejected by server'
        result = await goal_handle.get_result_async()
        # result.status is an int enum; 4 means SUCCEEDED in rclpy action API
        ok = (getattr(result, 'status', 0) == 4)
        return ok, ('SUCCEEDED' if ok else f'Ended with status={result.status}')

    async def send_nav_through(self, pts: List[tuple]):
        await self.ensure_servers()
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
        goal_handle = await send_future
        if not goal_handle.accepted:
            return False, 'Path goal rejected by server'
        result = await goal_handle.get_result_async()
        ok = (getattr(result, 'status', 0) == 4)
        return ok, ('SUCCEEDED' if ok else f'Ended with status={result.status}')

    def _nav_feedback(self, feedback_msg):
        # Hook for live updates if you want (distance remaining, etc.)
        # feedback_msg.feedback is NavigateToPose_Feedback / NavigateThroughPoses_Feedback
        pass
        # --- camera subscription management ---
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
        self.view.setMinimumHeight(240)
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
        self.view.setMinimumHeight(250)
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
        self.view.setMinimumHeight(250)
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
        self.resize(1200, 700)
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
        self.ang_slider = self._slider(-300, 300, 0)
        self.lin_label = QtWidgets.QLabel('Linear: 0.00 m/s')
        self.ang_label = QtWidgets.QLabel('Angular: 0.00 rad/s')
        self.btn_zero  = QtWidgets.QPushButton('Zero Velocity')

        grid.addWidget(QtWidgets.QLabel('Teleop (optional)'), r, 0, 1, 6); r += 1
        grid.addWidget(self.lin_label, r, 0, 1, 2); grid.addWidget(self.lin_slider, r, 2, 1, 4); r += 1
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
        self.path_edit.setMinimumHeight(120)
        self.path_edit.setPlaceholderText("x,y,yaw_deg per line")
        self.btn_send_path = QtWidgets.QPushButton('Go Through Poses')
        grid.addWidget(QtWidgets.QLabel('Waypoints'), r, 0, 1, 6); r += 1
        grid.addWidget(self.path_edit, r, 0, 1, 6); r += 1
        grid.addWidget(self.btn_send_path, r, 0, 1, 6); r += 1

        # Readouts
        self.pose_label = QtWidgets.QLabel('Pose: x=0.00, y=0.00, yaw=0.00°')
        self.scan_label = QtWidgets.QLabel('Lidar min: ∞ m')
        self.status     = QtWidgets.QLabel('Status: idle')
        grid.addWidget(self.pose_label, r, 0, 1, 6); r += 1
        grid.addWidget(self.scan_label, r, 0, 1, 6); r += 1
        grid.addWidget(self.status, r, 0, 1, 6); r += 1

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
        right.setSizes([350, 250, 250])  # initial heights (px); adjust as you like
        right.setStretchFactor(0, 1)
        right.setStretchFactor(1, 1)
        right.setStretchFactor(2, 1)

        # ----- Main horizontal splitter -----
        main_split = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        main_split.addWidget(left)     # controls
        main_split.addWidget(right)    # camera+lidar
        main_split.setStretchFactor(0, 0)
        main_split.setStretchFactor(1, 1)

        outer = QtWidgets.QVBoxLayout(self)
        outer.addWidget(main_split, 1)

        # Signals
        self.lin_slider.valueChanged.connect(self._lin_changed)
        self.ang_slider.valueChanged.connect(self._ang_changed)
        self.btn_zero.clicked.connect(self._zero)
        self.btn_send_wp.clicked.connect(self._send_wp)
        self.btn_send_path.clicked.connect(self._send_path)

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
    def _ang_changed(self, v): self.ang_label.setText(f'Angular: {v/100.0:.2f} rad/s')
    def _zero(self):
        self.lin_slider.setValue(0); self.ang_slider.setValue(0)
        self.node.publish_cmd(0.0, 0.0)
    def _tick_cmd(self):
        self.node.publish_cmd(self.lin_slider.value()/100.0, self.ang_slider.value()/100.0)

    # ---- Nav2 actions handlers ----
    def _send_wp(self):
        x = float(self.x_in.value()); y = float(self.y_in.value()); yaw = float(self.yaw_in.value())
        self.status.setText('Status: sending NavigateToPose goal…')
        fut = self.node.send_nav_goal(x, y, yaw)
        def _done(_):
            ok, msg = fut.result()
            self.status.setText('✅ Reached goal' if ok else f'❌ {msg}')
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
            self.status.setText('✅ Completed path' if ok else f'❌ {msg}')
        fut.add_done_callback(_done)

    # ---- UI refresh ----
    def _refresh_ui(self):
        x, y, yaw = self.node.latest_pose
        self.pose_label.setText(f'Pose: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.2f}°')
        mr = self.node.min_range
        self.scan_label.setText(f'Lidar min: {"∞" if not math.isfinite(mr) else f"{mr:.2f} m"}')


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

