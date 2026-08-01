#!/usr/bin/env -S ros2 launch
"""Launch the OpenMANIPULATOR-X tabletop pick/place practice."""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    start_rviz = LaunchConfiguration('start_rviz')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_only = LaunchConfiguration('setup_only')
    log_level = LaunchConfiguration('log_level')
    wait_timeout_sec = LaunchConfiguration('wait_timeout_sec')

    tabletop_world = PathJoinSubstitution([
        FindPackageShare('open_manipulator_demos'),
        'worlds',
        'tabletop_practice',
    ])

    gazebo = GroupAction(actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_bringup'),
                'launch',
                'open_manipulator_x_gazebo.launch.py',
            ])
        ]),
        launch_arguments=[
            ('world', tabletop_world),
            ('start_rviz', 'false'),
            ('headless', headless),
            ('use_sim_time', use_sim_time),
        ],
    )])

    moveit = GroupAction(actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_moveit_config'),
                'launch',
                'open_manipulator_x_moveit.launch.py',
            ])
        ]),
        launch_arguments=[
            ('start_rviz', start_rviz),
            ('use_sim', use_sim_time),
        ],
    )])

    demo = Node(
        package='open_manipulator_demos',
        executable='demo_pick_and_place',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'setup_only': setup_only},
            {'wait_timeout_sec': wait_timeout_sec},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Whether to start MoveIt RViz',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo simulation clock',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Run Gazebo server without the GUI client',
        ),
        DeclareLaunchArgument(
            'setup_only',
            default_value='true',
            description='Create the planning scene without commanding motion',
        ),
        DeclareLaunchArgument(
            'wait_timeout_sec',
            default_value='30.0',
            description='Seconds to wait for MoveIt services and action servers',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS log level for the demo node',
        ),
        gazebo,
        moveit,
        demo,
    ])
