from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rs1_gui',
            executable='gui_node',
            name='rs1_gui',
            output='screen',
            emulate_tty=True
        )
    ])