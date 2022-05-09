import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    slam_params_file = DeclareLaunchArgument(
        name='slam_params_file',
        default_value=os.path.join(get_package_share_directory("andante_launch"),
                                    'config', 'slam_toolbox.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )
    start_async_slam_toolbox_node = Node(
        name='slam_toolbox',
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        output='screen',
        parameters=[LaunchConfiguration('slam_params_file')]
    )
    return LaunchDescription([
        slam_params_file,
        start_async_slam_toolbox_node,
        laser_link_tf_publisher
    ])
