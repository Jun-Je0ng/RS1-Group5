#!/usr/bin/env python3
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import yaml


class ClickToYaml(Node):
    def __init__(self):
        super().__init__('click_to_yaml')

        # Parameters
        self.declare_parameter('filename', 'waypoints.yaml')
        self.declare_parameter('target_frame', 'map')  # use 'odom' if you prefer

        filename_param = self.get_parameter('filename').get_parameter_value().string_value
        self.filename = os.path.expanduser(filename_param)
        self.frame = self.get_parameter('target_frame').get_parameter_value().string_value

        # Waypoint storage
        self.points = []

        # Subscription
        self.create_subscription(PointStamped, '/clicked_point', self.cb, 10)

        self.get_logger().info(
            f"Click points in RViz (frame={self.frame}). "
            f"Press Ctrl+C to save to {self.filename}"
        )

    def cb(self, msg: PointStamped):
        if msg.header.frame_id != self.frame:
            self.get_logger().warn(
                f"Got {msg.header.frame_id}, expected {self.frame}. "
                "Change RViz Fixed Frame or set target_frame param."
            )
            return
        self.points.append([float(msg.point.x), float(msg.point.y)])
        self.get_logger().info(f"Added: {self.points[-1]} (total={len(self.points)})")

    def _write_file(self):
        data = {
            'frame_id': self.frame,
            'waypoints': self.points,
            'generated': datetime.now().isoformat(),
        }

        directory = os.path.dirname(self.filename)
        if directory:
            os.makedirs(directory, exist_ok=True)

        with open(self.filename, 'w') as f:
            yaml.safe_dump(data, f)

        self.get_logger().info(f"✅ Saved {len(self.points)} waypoints to {self.filename}")
        self.get_logger().info(
            "Next steps:\n"
            "  1) source /opt/ros/humble/setup.bash\n"
            "  2) source ~/git/Robotics-Studio-1/41068_ws/install/setup.bash\n"
            "  3) Run:\n"
            f"     python3 ~/git/Robotics-Studio-1/41068_ws/src/41068_ignition_bringup/scripts/"
            f"send_yaml_through_poses.py {self.filename}"
        )

    def destroy_node(self):
        # Save on shutdown (Ctrl+C)
        try:
            self._write_file()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = ClickToYaml()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as exc:
            if 'rcl_shutdown already called' not in str(exc):
                raise


if __name__ == '__main__':
    main()

