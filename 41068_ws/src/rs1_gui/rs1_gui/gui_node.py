import sys, math
from dataclasses import dataclass
from typing import List

from PySide6 import QtCore, QtWidgets
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
from std_srvs.srv import Trigger

def yaw_to_quat(yaw: float) -> Quaternion:
    # planar yaw -> quaternion
    import math
    q = Quaternion()
    q.z = math.sin(yaw/2.0)
    q.w = math.cos(yaw/2.0)
    return q

@dataclass
class Topics:
    cmd_vel: str = '/cmd_vel'
    single_waypoint: str = '/waypoint'     # adapt if your topic name differs
    path_waypoints: str = '/waypoints'     # adapt if your topic name differs
    follower_start: str = '/waypoint_follower/start'
    follower_stop: str = '/waypoint_follower/stop'
    frame_id: str = 'map'                  # or 'odom' if that’s what you use

class RosBackend(Node):
    def __init__(self, topics: Topics):
        super().__init__('rs1_gui_node')
        self.topics = topics
        self.cmd_pub = self.create_publisher(Twist, topics.cmd_vel, 10)
        self.wp_pub  = self.create_publisher(PoseStamped, topics.single_waypoint, 10)
        self.path_pub = self.create_publisher(Path, topics.path_waypoints, 10)

        self.start_cli = self.create_client(Trigger, topics.follower_start)
        self.stop_cli  = self.create_client(Trigger, topics.follower_stop)

    def publish_cmd(self, lin: float, ang: float):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.cmd_pub.publish(msg)

    def publish_waypoint(self, x: float, y: float, yaw_deg: float):
        yaw = math.radians(yaw_deg)
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.topics.frame_id
        ps.pose.position = Point(x=x, y=y, z=0.0)
        ps.pose.orientation = yaw_to_quat(yaw)
        self.wp_pub.publish(ps)
        self.get_logger().info(f'Published waypoint: ({x:.2f},{y:.2f},{yaw_deg:.1f}°) -> {self.topics.single_waypoint}')

    def publish_path(self, pts: List[tuple]):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.topics.frame_id
        for (x,y,yaw_deg) in pts:
            yaw = math.radians(yaw_deg)
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position = Point(x=x, y=y, z=0.0)
            ps.pose.orientation = yaw_to_quat(yaw)
            path.poses.append(ps)
        self.path_pub.publish(path)
        self.get_logger().info(f'Published {len(path.poses)} waypoints -> {self.topics.path_waypoints}')

    async def call_trigger(self, client):
        if not client.service_is_ready():
            await client.wait_for_service()
        req = Trigger.Request()
        return await client.call_async(req)

class Gui(QtWidgets.QWidget):
    def __init__(self, node: RosBackend):
        super().__init__()
        self.node = node
        self.setWindowTitle('RS1 Waypoint GUI')
        self.resize(560, 420)

        # ---- Teleop area (optional, handy for testing) ----
        self.lin_slider = self._slider(-200, 200, 0)
        self.ang_slider = self._slider(-300, 300, 0)
        self.lin_label = QtWidgets.QLabel('Linear: 0.00 m/s')
        self.ang_label = QtWidgets.QLabel('Angular: 0.00 rad/s')
        self.btn_zero  = QtWidgets.QPushButton('Zero Velocity')

        # ---- Single waypoint ----
        self.x_in = QtWidgets.QDoubleSpinBox(); self.x_in.setRange(-1e6, 1e6); self.x_in.setDecimals(3)
        self.y_in = QtWidgets.QDoubleSpinBox(); self.y_in.setRange(-1e6, 1e6); self.y_in.setDecimals(3)
        self.yaw_in = QtWidgets.QDoubleSpinBox(); self.yaw_in.setRange(-360, 360); self.yaw_in.setDecimals(1)
        self.btn_send_wp = QtWidgets.QPushButton('Send Waypoint')

        # ---- Path (multi-waypoint) ----
        self.path_edit = QtWidgets.QPlainTextEdit()
        self.path_edit.setPlaceholderText("One waypoint per line: x,y,yaw_deg\nExample:\n10.0, 62.0, 0\n15.5, 60.0, 90")
        self.btn_send_path = QtWidgets.QPushButton('Send Path')

        # ---- Follower controls ----
        self.btn_start = QtWidgets.QPushButton('Start Auto-Drive')
        self.btn_stop  = QtWidgets.QPushButton('Stop Auto-Drive')
        self.status    = QtWidgets.QLabel('Status: idle')

        # ---- Layout ----
        grid = QtWidgets.QGridLayout()
        r = 0
        grid.addWidget(QtWidgets.QLabel('Teleop (optional)'), r, 0, 1, 3); r+=1
        grid.addWidget(self.lin_label, r, 0, 1, 2); grid.addWidget(self.lin_slider, r, 2); r+=1
        grid.addWidget(self.ang_label, r, 0, 1, 2); grid.addWidget(self.ang_slider, r, 2); r+=1
        grid.addWidget(self.btn_zero, r, 0, 1, 3); r+=1

        grid.addWidget(QtWidgets.QLabel('Single Waypoint'), r, 0, 1, 3); r+=1
        grid.addWidget(QtWidgets.QLabel('x'), r, 0); grid.addWidget(self.x_in, r, 1)
        grid.addWidget(QtWidgets.QLabel('y'), r, 2); grid.addWidget(self.y_in, r, 3)
        grid.addWidget(QtWidgets.QLabel('yaw°'), r, 4); grid.addWidget(self.yaw_in, r, 5); r+=1
        grid.addWidget(self.btn_send_wp, r, 0, 1, 6); r+=1

        grid.addWidget(QtWidgets.QLabel('Path (x,y,yaw° per line)'), r, 0, 1, 6); r+=1
        grid.addWidget(self.path_edit, r, 0, 1, 6); r+=1
        grid.addWidget(self.btn_send_path, r, 0, 1, 6); r+=1

        grid.addWidget(self.btn_start, r, 0, 1, 3)
        grid.addWidget(self.btn_stop,  r, 3, 1, 3); r+=1
        grid.addWidget(self.status, r, 0, 1, 6); r+=1

        self.setLayout(grid)

        # ---- Signals ----
        self.lin_slider.valueChanged.connect(self._lin_changed)
        self.ang_slider.valueChanged.connect(self._ang_changed)
        self.btn_zero.clicked.connect(self._zero)

        self.btn_send_wp.clicked.connect(self._send_wp)
        self.btn_send_path.clicked.connect(self._send_path)
        self.btn_start.clicked.connect(self._start_follow)
        self.btn_stop.clicked.connect(self._stop_follow)

        # defaults for fast testing
        self.x_in.setValue(8.0); self.y_in.setValue(62.0); self.yaw_in.setValue(0.0)

        # timer: publish current slider values at 20 Hz
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick_cmd)
        self.timer.start(50)

    # -------- helpers ----------
    def _slider(self, mn, mx, val):
        s = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        s.setRange(mn, mx); s.setValue(val); s.setSingleStep(5)
        return s

    def _lin_changed(self, v): self._set_lin_label(v/100.0)
    def _ang_changed(self, v): self._set_ang_label(v/100.0)

    def _set_lin_label(self, val): self.lin_label.setText(f'Linear: {val:.2f} m/s')
    def _set_ang_label(self, val): self.ang_label.setText(f'Angular: {val:.2f} rad/s')

    def _zero(self):
        self.lin_slider.setValue(0); self.ang_slider.setValue(0)
        self.node.publish_cmd(0.0, 0.0)

    def _tick_cmd(self):
        self.node.publish_cmd(self.lin_slider.value()/100.0, self.ang_slider.value()/100.0)

    def _send_wp(self):
        self.node.publish_waypoint(self.x_in.value(), self.y_in.value(), self.yaw_in.value())
        self.status.setText('Status: waypoint published')

    def _parse_path_text(self) -> List[tuple]:
        pts = []
        for line in self.path_edit.toPlainText().splitlines():
            line = line.strip()
            if not line or line.startswith('#'): continue
            parts = [p.strip() for p in line.split(',')]
            if len(parts) < 2: continue
            x = float(parts[0]); y = float(parts[1])
            yaw = float(parts[2]) if len(parts) >= 3 else 0.0
            pts.append((x,y,yaw))
        return pts

    def _send_path(self):
        pts = self._parse_path_text()
        if not pts:
            self.status.setText('Status: no valid path lines')
            return
        self.node.publish_path(pts)
        self.status.setText(f'Status: path with {len(pts)} waypoints published')

    def _start_follow(self):
        self.status.setText('Status: starting…')
        fut = self.node.call_trigger(self.node.start_cli)
        fut.add_done_callback(lambda _: self.status.setText('Status: follower START requested'))

    def _stop_follow(self):
        self.status.setText('Status: stopping…')
        fut = self.node.call_trigger(self.node.stop_cli)
        fut.add_done_callback(lambda _: self.status.setText('Status: follower STOP requested'))

def main():
    rclpy.init()
    topics = Topics()  # change names here if your follower uses different topics/services
    node = RosBackend(topics)

    app = QtWidgets.QApplication(sys.argv)
    gui = Gui(node); gui.show()

    ros_timer = QtCore.QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    ros_timer.start(10)

    code = app.exec()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)

if __name__ == '__main__':
    main()
