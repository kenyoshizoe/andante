#!/usr/bin/env python3
import os
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
# from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    path_to_urdf = get_package_share_directory(
        'andante_description') + "/urdf/andante.urdf.xacro"
    create_2_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='create_2_robot_state_publisher',
        # namespace='andante',
        output='screen',
        parameters=[
            {
                'robot_description': ParameterValue(
                    Command(['xacro ', str(path_to_urdf)]), value_type=str
                )
            }
        ]
    )

    world = os.path.join(get_package_share_directory('andante_gazebo'),
                         'worlds', 'aruco_calibration.world')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    )

    urdf_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        # namespace='andante',
        output='screen',
        arguments=['-entity', 'andante',
                   '-topic', 'robot_description'],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        create_2_robot_state_publisher,
        gzserver_launch,
        gzclient_launch,
        urdf_spawner
    ])
