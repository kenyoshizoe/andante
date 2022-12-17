import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    path_to_urdf = get_package_share_directory(
        'andante_description') + "/urdf/andante.urdf.xacro"
    create_2_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='create_2_robot_state_publisher',
        namespace='andante',
        output='screen',
        parameters=[
            {
                'robot_description': ParameterValue(
                    Command(['xacro ', str(path_to_urdf)]), value_type=str
                )
            }
        ]
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(
            get_package_share_directory("andante_description"),
            'rviz', 'display.rviz'
        )]
    )

    return LaunchDescription([
        create_2_robot_state_publisher,
        rviz
    ])
