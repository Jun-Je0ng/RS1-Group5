#!/usr/bin/env python3
import rclpy, yaml
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from datetime import datetime

class ClickToYaml(Node):
    def __init__(self):
        super().__init__('click_to_yaml')
        self.declare_parameter('filename', 'waypoints.yaml')
        self.declare_parameter('target_frame', 'map')  # use 'odom' if you prefer
        self.filename = self.get_parameter('filename').get_parameter_value().string_value
        self.frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.points = []
        self.create_subscription(PointStamped, '/clicked_point', self.cb, 10)
        self.get_logger().info(
            f"Click points in RViz (frame={self.frame}). Ctrl+C to save to {self.filename}")

    def cb(self, msg: PointStamped):
        if msg.header.frame_id != self.frame:
            self.get_logger().warn(f"Got {msg.header.frame_id}, expected {self.frame}. "
                                   "Change RViz Fixed Frame or set target_frame param.")
            return
        self.points.append([float(msg.point.x), float(msg.point.y)])
        self.get_logger().info(f"Added: {self.points[-1]} (total={len(self.points)})")

    def destroy_node(self):
        data = {'frame_id': self.frame, 'waypoints': self.points,
                'generated': datetime.now().isoformat()}
        with open(self.filename, 'w') as f:
            yaml.safe_dump(data, f)
        self.get_logger().info(f"Saved {len(self.points)} waypoints to {self.filename}")
        super().destroy_node()

def main():
    rclpy.init()
    n = ClickToYaml()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

