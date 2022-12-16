import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments='-d ' + os.path.join(
            get_package_share_directory("andante_bringup"),
            'rviz', 'default.rviz'
        )
    )

    return LaunchDescription([rviz])
