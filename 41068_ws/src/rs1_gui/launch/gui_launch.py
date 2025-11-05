from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    initial_x = DeclareLaunchArgument('initial_x', default_value='0.0')
    initial_y = DeclareLaunchArgument('initial_y', default_value='0.0')
    initial_z = DeclareLaunchArgument('initial_z', default_value='0.0')
    initial_yaw = DeclareLaunchArgument('initial_yaw', default_value='0.0')
    world = DeclareLaunchArgument('world', default_value='simple_trees')
    entity_name = DeclareLaunchArgument('entity_name', default_value='husky')

    return LaunchDescription([
        initial_x,
        initial_y,
        initial_z,
        initial_yaw,
        world,
        entity_name,
        Node(
            package='rs1_gui',
            executable='gui_node',
            name='rs1_gui',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'initial_x': LaunchConfiguration('initial_x'),
                'initial_y': LaunchConfiguration('initial_y'),
                'initial_z': LaunchConfiguration('initial_z'),
                'initial_yaw': LaunchConfiguration('initial_yaw'),
                'world': LaunchConfiguration('world'),
                'entity_name': LaunchConfiguration('entity_name')
            }]
        )
    ])
