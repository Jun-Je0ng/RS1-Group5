import sys, math
from dataclasses import dataclass
from typing import List

from PySide6 import QtCore, QtWidgets
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
from std_srvs.srv import Trigger
from sensor_msgs.msg import LaserScan
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue


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
    scan_topic: str = '/scan'        # change if your Lidar topic is different
    move_bot_node: str = '/move_bot' # as per README/usage
class RosBackend(Node):
    def __init__(self, topics: Topics):
        super().__init__('rs1_gui_node')
        self.topics = topics
        self.cmd_pub = self.create_publisher(Twist, topics.cmd_vel, 10)
        self.wp_pub  = self.create_publisher(PoseStamped, topics.single_waypoint, 10)
        self.path_pub = self.create_publisher(Path, topics.path_waypoints, 10)

        self.start_cli = self.create_client(Trigger, topics.follower_start)
        self.stop_cli  = self.create_client(Trigger, topics.follower_stop)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, topics.scan_topic, self.scan_cb, 10
        )
        self.min_range = float('inf')

        # --- NEW: Parameter client for /move_bot ---
        self.param_cli = self.create_client(SetParameters, f'{topics.move_bot_node}/set_parameters')

    def scan_cb(self, msg: LaserScan):
        # quick-and-safe min range (ignore inf/nan)
        vals = [r for r in msg.ranges if r == r and r != float('inf')]
        self.min_range = min(vals) if vals else float('inf')

    async def set_move_bot_speeds(self, lin_x: float, ang_z: float):
        # Build parameter array request
        if not self.param_cli.service_is_ready():
            await self.param_cli.wait_for_service()

        def make_param(name, value):
            pv = ParameterValue()
            pv.type = ParameterValue.TYPE_DOUBLE
            pv.double_value = float(value)
            p = Parameter(name=name, value=pv)
            return p

        req = SetParameters.Request()
        req.parameters = [
            make_param('lin_x', lin_x),
            make_param('ang_z', ang_z),
        ]
        fut = self.param_cli.call_async(req)
        resp = await fut
        # Optionally, inspect resp.results to confirm success
        return resp

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
        # --- Move_Bot param controls ---
        self.mb_lin = QtWidgets.QDoubleSpinBox(); self.mb_lin.setRange(-5, 5); self.mb_lin.setDecimals(2); self.mb_lin.setSingleStep(0.05)
        self.mb_ang = QtWidgets.QDoubleSpinBox(); self.mb_ang.setRange(-6, 6); self.mb_ang.setDecimals(2); self.mb_ang.setSingleStep(0.05)
        self.btn_apply_mb = QtWidgets.QPushButton('Apply to /move_bot (lin_x, ang_z)')

        # --- Sensor readouts ---
        self.lbl_min_range = QtWidgets.QLabel('Lidar min: ∞ m')

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

        grid.addWidget(QtWidgets.QLabel('move_bot Speeds (params)'), r, 0, 1, 6); r+=1
        grid.addWidget(QtWidgets.QLabel('lin_x'), r, 0); grid.addWidget(self.mb_lin, r, 1)
        grid.addWidget(QtWidgets.QLabel('ang_z'), r, 2); grid.addWidget(self.mb_ang, r, 3)
        grid.addWidget(self.btn_apply_mb, r, 4, 1, 2); r+=1

        grid.addWidget(self.lbl_min_range, r, 0, 1, 6); r+=1

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

        self.btn_apply_mb.clicked.connect(self._apply_move_bot)

        # existing ui refresh timer… add:
        self.ui_timer = QtCore.QTimer(self)
        self.ui_timer.timeout.connect(self._refresh_sensors)
        self.ui_timer.start(100)  # 10 Hz

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
    def _refresh_sensors(self):
        # show min laser range
        mr = self.node.min_range
        txt = f'{mr:.2f} m' if math.isfinite(mr) else '∞ m'
        self.lbl_min_range.setText(f'Lidar min: {txt}')

    @QtCore.Slot()
    def _apply_move_bot(self):
        lin = float(self.mb_lin.value())
        ang = float(self.mb_ang.value())
        self.status.setText('Status: applying /move_bot params…')

        # fire-and-forget; update status when done
        fut = self.node.set_move_bot_speeds(lin, ang)
        def done(_):
            self.status.setText(f'Status: /move_bot lin_x={lin:.2f}, ang_z={ang:.2f} applied')
        try:
            fut.add_done_callback(done)
        except Exception:
            # if running older PySide, just ignore callback errors
            pass
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
