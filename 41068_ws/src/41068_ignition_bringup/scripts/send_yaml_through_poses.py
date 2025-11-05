#!/usr/bin/env python3
import math, yaml, sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses

def yaw_to_quat(yaw: float):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(0.5*yaw)
    q.w = math.cos(0.5*yaw)
    return q

class YamlThroughPoses(Node):
    def __init__(self, yaml_file: str):
        super().__init__('yaml_through_poses')
        self.cli = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)

        self.frame = data.get('frame_id', 'map')
        pts = data.get('waypoints', [])
        if not pts:
            raise RuntimeError('No waypoints in YAML')

        # Build poses with heading along the path
        self.poses = []
        for i, (x, y) in enumerate(pts):
            # heading: from this point to the next; for last point, keep previous
            if i < len(pts)-1:
                nx, ny = pts[i+1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                if len(pts) >= 2:
                    px, py = pts[i-1]
                    yaw = math.atan2(y - py, x - px)
                else:
                    yaw = 0.0
            ps = PoseStamped()
            ps.header.frame_id = self.frame
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation = yaw_to_quat(yaw)
            self.poses.append(ps)

        self.get_logger().info(f'Loaded {len(self.poses)} poses in frame {self.frame}')

    def send(self):
        self.get_logger().info('Waiting for Nav2 action server…')
        self.cli.wait_for_server()
        goal = NavigateThroughPoses.Goal()
        goal.poses = self.poses
        goal.behavior_tree = ''  # use default
        self.get_logger().info('Sending goal…')
        send_future = self.cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        self.get_logger().info('Goal accepted, waiting for result…')
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result()
        self.get_logger().info(f'Result: {result.result} (status {goal_handle.status})')

def main():
    rclpy.init()
    yaml_file = sys.argv[1] if len(sys.argv) > 1 else '/home/%s/waypoints.yaml' % (os.environ.get('USER',''))
    node = YamlThroughPoses(yaml_file)
    try:
        node.send()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

