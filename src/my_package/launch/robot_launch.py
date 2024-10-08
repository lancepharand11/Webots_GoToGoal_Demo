# MODIFIED FROM webots_ros2_epuck/launch/robot_launch.py

import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    package_dir1 = get_package_share_directory('my_package')
    package_dir2 = get_package_share_directory('webots_ros2_epuck')
    # robot_description_path = os.path.join(package_dir1, 'resource', 'my_robot.urdf')
    robot_description_path = os.path.join(package_dir2, 'resource', 'epuck_webots.urdf')
    fill_map = LaunchConfiguration('fill_map', default=True)
    world = LaunchConfiguration('world')
    map_filename = LaunchConfiguration('map', default=os.path.join(package_dir2, 'resource', 'epuck_world_map.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    webots = WebotsLauncher(
        # world=os.path.join(package_dir1, 'worlds', 'my_world.wbt'),
        world=PathJoinSubstitution([package_dir2, 'worlds', world]),
        ros2_supervisor=True
    )

    # my_robot_driver = WebotsController(
    #     robot_name='my_robot',
    #     parameters=[
    #         {'robot_description': robot_description_path},
    #     ]
    # )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    # ROS control spawners
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]


    ros2_control_params = os.path.join(package_dir2, 'resource', 'ros2_control.yml')
    use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy'])
    if use_twist_stamped:
        mappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    else:
        mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    epuck_driver = WebotsController(
        robot_name='e-puck',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )

    epuck_process = Node(
        package='webots_ros2_epuck',
        executable='epuck_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    robot_control_node = Node(
        package="my_package",
        executable="my_robot_control"
    )

    # Wait for the simulation to be ready to start the tools and spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=epuck_driver,
        nodes_to_start= ros_control_spawners
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='epuck_world.wbt',
            description='Choose one of the world files from `/webots_ros2_epuck/world` directory'
        ),
        webots,
        webots._supervisor,

        robot_state_publisher,
        footprint_publisher,

        epuck_driver,
        epuck_process,
        robot_control_node,
        waiting_nodes,
        

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])
