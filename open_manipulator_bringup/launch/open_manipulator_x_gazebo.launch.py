#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Wonho Yun, Sungho Woo, Woojin Wie

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Launch Arguments
    open_manipulator_description_path = os.path.join(
        get_package_share_directory('open_manipulator_description')
    )

    open_manipulator_bringup_path = os.path.join(
        get_package_share_directory('open_manipulator_bringup')
    )

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(open_manipulator_bringup_path, 'worlds'),
            ':' + str(Path(open_manipulator_description_path).parent.resolve()),
        ],
    )

    arguments = LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='empty_world', description='Gz sim World'
        ),
        DeclareLaunchArgument(
            'start_rviz', default_value='false', description='Whether to execute rviz2'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Whether launched ROS nodes use the Gazebo simulation clock',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo server without the GUI client',
        ),
    ])
    start_rviz = LaunchConfiguration('start_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py',
        ]),
        launch_arguments=[
            (
                'gz_args',
                [
                    LaunchConfiguration('world'),
                    '.sdf',
                    ' -v 1',
                    ' -r',
                    PythonExpression(["' -s' if '", headless, "' == 'true' else ''"]),
                ],
            )
        ],
    )

    xacro_file = os.path.join(
        open_manipulator_description_path,
        'urdf',
        'open_manipulator_x',
        'open_manipulator_x.urdf.xacro',
    )

    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})

    robot_desc = doc.toprettyxml(indent='  ')

    params = {'robot_description': robot_desc}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_desc,
            '-x',   '0.0',
            '-y',   '0.0',
            '-z',   '0.07',  # OMX sits on top of the platform
            '-R',   '0.0',
            '-P',   '0.0',
            '-Y',   '0.0',
            '-name',            'open_manipulator_x',
            '-allow_renaming',  'true',
            '-use_sim',         'true',
        ],
    )

    # Define a cylinder SDF to spawn directly with Gazebo
    # cylinder_sdf = f'''
    # <?xml version="1.0"?>
    # <sdf version="1.6">
    #   <model name="cylinder_0">
    #     <pose> -0.10 -0.21 0.10 0 0 0</pose>
    #     <link name="link">
    #       <collision name="collision">
    #         <geometry>
    #           <cylinder>
    #             <radius>0.01</radius>
    #             <length>0.05</length>
    #           </cylinder>
    #         </geometry>
    #       </collision>
    #       <visual name="visual">
    #         <geometry>
    #           <cylinder>
    #             <radius>0.012</radius>
    #             <length>0.05</length>
    #           </cylinder>
    #         </geometry>
    #         <material>
    #           <ambient>0.8 0.3 0.3 1</ambient>
    #           <diffuse>0.8 0.3 0.3 1</diffuse>
    #           <specular>0.5 0.5 0.5 1</specular>
    #         </material>
    #       </visual>
    #     </link>
    #   </model>
    # </sdf>
    # '''

    # Controller spawner nodes
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # gz_spawn_cylinder_direct = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     output='screen',
    #     arguments=[
    #         '-string', cylinder_sdf,
    #         '-x', '-0.10',
    #         '-y', '-0.21',
    #         '-z', '0.10',
    #         '-R', '0.0',
    #         '-P', '0.0',
    #         '-Y', '0.0',
    #         '-name', 'student_cylinder_001',
    #         '-allow_renaming', 'false',
    #     ],
    # )

    rviz_config_file = os.path.join(
        open_manipulator_description_path, 'rviz', 'open_manipulator.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(start_rviz),
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner, gripper_controller_spawner],
            )
        ),
        bridge,
        gazebo_resource_path,
        arguments,
        gazebo,
        node_robot_state_publisher,
        rviz,
        gz_spawn_entity,
        # gz_spawn_cylinder_direct
    ])
