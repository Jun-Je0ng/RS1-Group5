from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    # Paths
    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    # Args
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='True')
    rviz_arg         = DeclareLaunchArgument('rviz', default_value='False')
    nav2_arg         = DeclareLaunchArgument('nav2', default_value='True')
    world_arg        = DeclareLaunchArgument('world', default_value='simple_trees',
                          description='Which world to load', choices=['simple_trees','large_demo'])

    # Drone spawn pose
    ld.add_action(DeclareLaunchArgument('robot_x',   default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('robot_y',   default_value='62.0'))
    ld.add_action(DeclareLaunchArgument('robot_z',   default_value='2.0'))
    ld.add_action(DeclareLaunchArgument('robot_yaw', default_value='0.0'))

    ld.add_action(use_sim_time_arg)
    ld.add_action(rviz_arg)
    ld.add_action(nav2_arg)
    ld.add_action(world_arg)

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_x   = LaunchConfiguration('robot_x')
    robot_y   = LaunchConfiguration('robot_y')
    robot_z   = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')

    # URDF → /robot_description
    robot_description_content = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_path, 'urdf_drone', 'parrot.urdf.xacro'])]),
        value_type=str)
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': use_sim_time}],
        output='screen'
    )
    ld.add_action(robot_state_pub)

    # Robot localization (odom→base_link)
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([config_path, 'robot_localization.yaml']),
                    {'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_localization_node)

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([pkg_path, 'worlds',
                                               [LaunchConfiguration('world'), '.sdf']]), ' -r']
        }.items()
    )
    ld.add_action(gazebo)

    # Spawn drone
    spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic','/robot_description','-x',robot_x,'-y',robot_y,'-z',robot_z,'-Y',robot_yaw]
    )
    ld.add_action(spawner)

    # Bridge
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge.yaml']),
                     'use_sim_time': use_sim_time}]
    )
    ld.add_action(gazebo_bridge)

    # -------- MOVEMENT STACK (new) --------
    # twist_mux (external pkg) using our config/twist_mux.yaml
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[{'yaml_cfg_file':
                     PathJoinSubstitution([pkg_path, 'config', 'twist_mux.yaml'])}]
    )
    ld.add_action(twist_mux)

    # mode manager (manual/auto switch)
    mode_manager = Node(
        package='41068_ignition_bringup',
        executable='mode_manager',
        name='mode_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    ld.add_action(mode_manager)

    # waypoint follower (3D)
    follower_3d = Node(
        package='41068_ignition_bringup',
        executable='waypoint_follower_3d',
        name='waypoint_follower_3d',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'kp_lin': 0.6, 'kp_z': 0.6, 'kp_yaw': 0.8,
                     'max_v': 1.0, 'max_vz': 0.8, 'max_wz': 0.8}]
    )
    ld.add_action(follower_3d)

    # adapter → MAVROS velocity setpoint (change topic if you don’t use MAVROS)
    adapter = Node(
        package='41068_ignition_bringup',
        executable='command_adapter_mavros',
        name='command_adapter_mavros',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'mavros_twist_topic': '/mavros/setpoint_velocity/cmd_vel_unstamped'}]
    )
    ld.add_action(adapter)
    # --------------------------------------

    # RViz (optional)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path, '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz)

    # Nav2 (optional)
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path, 'launch', '41068_navigation.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    return ld
