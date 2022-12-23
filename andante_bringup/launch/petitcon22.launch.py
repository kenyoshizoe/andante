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

    camera_linetracer = Node(
        package='andante_camera_linetrace',
        executable='andante_camera_linetrace',
        name='andante_camera_linetrace',
        output='screen',
        parameters=[
            config
        ],
        remappings=[
            ("camera/image_raw", "elp_camera/image_raw"),
            ("camera/camera_info", "elp_camera/camera_info"),
            ("cmd_vel", "cmd_vel_unfiltered")
        ]
    )

    return LaunchDescription([
        push_ns,
        camera_linetracer
    ])
