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


class CameraViewer(QtWidgets.QWidget):
    """Simple Qt widget that shows a ROS image topic in real-time."""
    def __init__(self, node: RosBackend, default_topic="/camera/image_raw"):
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



class Gui(QtWidgets.QWidget):
    def __init__(self, node: RosBackend):
        super().__init__()
        self.node = node
        self.setWindowTitle('RS1 Waypoint GUI (Nav2)')
        self.resize(1000, 600)

        # ----- LEFT: controls panel -----
        left = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(left)

        # Teleop
        self.lin_slider = self._slider(-200, 200, 0)
        self.ang_slider = self._slider(-300, 300, 0)
        self.lin_label = QtWidgets.QLabel('Linear: 0.00 m/s')
        self.ang_label = QtWidgets.QLabel('Angular: 0.00 rad/s')
        self.btn_zero  = QtWidgets.QPushButton('Zero Velocity')

        # Single waypoint
        self.x_in = self._dspin(-1e6, 1e6, 8.0, 0.01)
        self.y_in = self._dspin(-1e6, 1e6, 62.0, 0.01)
        self.yaw_in = self._dspin(-360, 360, 0.0, 0.1)
        self.btn_send_wp = QtWidgets.QPushButton('Go To Pose')

        # Multi-waypoint
        self.path_edit = QtWidgets.QPlainTextEdit()
        self.path_edit.setPlaceholderText(
            "One waypoint per line: x,y,yaw_deg\n"
            "Example:\n10.0, 62.0, 0\n15.5, 60.0, 90"
        )
        self.btn_send_path = QtWidgets.QPushButton('Go Through Poses')

        # Readouts
        self.pose_label = QtWidgets.QLabel('Pose: x=0.00, y=0.00, yaw=0.00°')
        self.scan_label = QtWidgets.QLabel('Lidar min: ∞ m')
        self.status     = QtWidgets.QLabel('Status: idle')

        r = 0
        grid.addWidget(QtWidgets.QLabel('Teleop (optional)'), r, 0, 1, 6); r += 1
        grid.addWidget(self.lin_label, r, 0, 1, 2); grid.addWidget(self.lin_slider, r, 2, 1, 4); r += 1
        grid.addWidget(self.ang_label, r, 0, 1, 2); grid.addWidget(self.ang_slider, r, 2, 1, 4); r += 1
        grid.addWidget(self.btn_zero, r, 0, 1, 6); r += 1

        grid.addWidget(QtWidgets.QLabel('Single Goal (map frame)'), r, 0, 1, 6); r += 1
        grid.addWidget(QtWidgets.QLabel('x'), r, 0); grid.addWidget(self.x_in, r, 1)
        grid.addWidget(QtWidgets.QLabel('y'), r, 2); grid.addWidget(self.y_in, r, 3)
        grid.addWidget(QtWidgets.QLabel('yaw°'), r, 4); grid.addWidget(self.yaw_in, r, 5); r += 1
        grid.addWidget(self.btn_send_wp, r, 0, 1, 6); r += 1

        grid.addWidget(QtWidgets.QLabel('Waypoints (x,y,yaw° per line)'), r, 0, 1, 6); r += 1
        grid.addWidget(self.path_edit, r, 0, 1, 6); r += 1
        grid.addWidget(self.btn_send_path, r, 0, 1, 6); r += 1

        grid.addWidget(self.pose_label, r, 0, 1, 6); r += 1
        grid.addWidget(self.scan_label, r, 0, 1, 6); r += 1
        grid.addWidget(self.status, r, 0, 1, 6); r += 1

        # ----- RIGHT: tabs (Camera; optionally RViz if you added RvizEmbedder) -----
        tabs = QtWidgets.QTabWidget()

        # Camera tab
        cam_group = QtWidgets.QWidget()
        cam_layout = QtWidgets.QVBoxLayout(cam_group)
        self.cam = CameraViewer(self.node, default_topic="/camera/image_raw")
        cam_layout.addWidget(self.cam, 1)
        tabs.addTab(cam_group, "Camera")

        # If you previously added RvizEmbedder class, you can include:
        # rviz_group = QtWidgets.QWidget()
        # rviz_layout = QtWidgets.QVBoxLayout(rviz_group)
        # self.rviz = RvizEmbedder(rviz_group)
        # rviz_layout.addWidget(self.rviz, 1)
        # tabs.addTab(rviz_group, "RViz")

        # ----- Splitter -----
        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        splitter.addWidget(left)
        splitter.addWidget(tabs)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        outer = QtWidgets.QVBoxLayout(self)
        outer.addWidget(splitter, 1)

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
