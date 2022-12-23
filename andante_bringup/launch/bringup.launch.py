import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('andante_bringup'),
        'config',
        'petitcon22.yaml'
    )
    push_ns = PushRosNamespace('andante')

    create_driver = Node(
        package='create_driver',
        executable='create_driver',
        name='create_driver',
        parameters=[
            config
        ]
    )
    path_to_urdf = get_package_share_directory(
        'andante_description') + "/urdf/andante.urdf.xacro"
    create_2_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='create_2_robot_state_publisher',
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

    cmd_vel_smoother = Node(
        package='andante_cmd_vel_smoother',
        executable='andante_cmd_vel_smoother',
        name='andante_cmd_vel_smoother',
        output='screen',
        parameters=[config]
    )

    # camera
    camera_v4l2 = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        namespace='elp_camera',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription(
        push_ns,
        create_driver,
        create_2_robot_state_publisher,
        cmd_vel_smoother,
        camera_v4l2
    )
