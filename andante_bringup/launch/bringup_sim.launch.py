import os
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.actions.set_remap import SetRemap
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('andante_bringup'),
        'config',
        'petitcon22.yaml'
    )
    push_ns = PushRosNamespace('andante')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value="petitcon22.world"
    )

    # robot_state_publisher
    path_to_urdf = get_package_share_directory(
        'andante_description') + "/urdf/andante.urdf.xacro"
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='andante_robot_state_publisher',
        output='screen',
        parameters=[
                {
                    'robot_description': ParameterValue(
                        Command(['xacro ', str(path_to_urdf)]), value_type=str
                    )
                },
            config
        ]
    )

    # smoother
    cmd_vel_smoother = Node(
        package='andante_cmd_vel_smoother',
        executable='andante_cmd_vel_smoother',
        name='andante_cmd_vel_smoother',
        output='screen',
        parameters=[config]
    )

    # gazebo
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    world = PathJoinSubstitution([get_package_share_directory('andante_gazebo'),
                                 'worlds', LaunchConfiguration('world')])
    # gzserver = GroupAction(
    #     actions=[
    #         SetRemap(src='cmd_vel', dst='cmd_vel_filtered'),
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(pkg_gazebo_ros, 'launch',
    #                              'gzserver.launch.py')
    #             ),
    #             launch_arguments={'world': world}.items(),
    #         )
    #     ]
    # )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch',
                         'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    )
    urdf_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', 'andante',
                   '-topic', 'robot_description',
                   '-robot_namespace', 'andante',
                   '-x', '1.5025', '-y', '-1.4', '-z', '0.3',
                   '-Y', str(math.pi)
                   ],
        parameters=[{'use_sim_time': True}, config],
    )

    return LaunchDescription([
        push_ns,
        world_arg,
        robot_state_publisher,
        cmd_vel_smoother,
        gzserver,
        gzclient,
        urdf_spawner
    ])
