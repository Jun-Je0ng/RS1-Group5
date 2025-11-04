#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToTwistNode(Node):
    def __init__(self):
        super().__init__('joy_to_twist')
        self.declare_parameter('axis_linear_x', 1)
        self.declare_parameter('axis_angular_yaw', 3)
        self.declare_parameter('scale_linear', 0.5)
        self.declare_parameter('scale_angular', 1.0)
        self.declare_parameter('enable_button', 0)   # e.g., A button on Xbox

        self.axis_linear_x = self.get_parameter('axis_linear_x').get_parameter_value().integer_value
        self.axis_angular_yaw = self.get_parameter('axis_angular_yaw').get_parameter_value().integer_value
        self.scale_linear = self.get_parameter('scale_linear').get_parameter_value().double_value
        self.scale_angular = self.get_parameter('scale_angular').get_parameter_value().double_value
        self.enable_button = self.get_parameter('enable_button').get_parameter_value().integer_value

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info(f"JoyToTwist started: linear_axis={self.axis_linear_x}, angular_axis={self.axis_angular_yaw}")

    def joy_callback(self, msg: Joy):
        # Check enable button
        if msg.buttons[self.enable_button] == 0:
            # button not pressed → zero velocities
            twist = Twist()
        else:
            # Read axes
            linear_input = msg.axes[self.axis_linear_x]
            angular_input = msg.axes[self.axis_angular_yaw]

            # Apply scales
            linear = self.scale_linear * linear_input
            angular = self.scale_angular * angular_input

            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular

        self.twist_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToTwistNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

