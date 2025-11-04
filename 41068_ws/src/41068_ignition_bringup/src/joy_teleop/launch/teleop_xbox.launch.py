from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    teleop_node = Node(
        package='your_package_name',  # replace with your package
        executable='joy_to_twist.py',
        name='joy_to_twist',
        output='screen',
        parameters=[
            { 'axis_linear_x': 1 },
            { 'axis_angular_yaw': 3 },
            { 'scale_linear': 0.5 },
            { 'scale_angular': 1.0 },
            { 'enable_button': 0 }
        ]
    )

    move_bot_node = Node(
        package='41068_ignition_bringup',
        executable='move_bot',
        name='move_bot',
        output='screen',
        parameters=[
            # include any existing parameters you use for your Husky robot
        ]
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
        move_bot_node,
    ])
