import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    #create2(andante)
    create_driver = Node(
        package='create_driver',
        executable='create_driver',
        name='create_driver',
        output='screen',
        parameters=[
            {'config': os.path.join(get_package_share_directory("andante_bringup"), "config/create_driver.yaml")}
        ]
    )
    path_to_urdf= get_package_share_directory('andante_description') + "/urdf/andante.urdf.xacro"
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
            }
        ]
    )
    # ld06
    ld06_serial_port = DeclareLaunchArgument(
        name='ld06_serial_port',
        default_value='/dev/ttyUSB_LD06',
        description='LD06 Serial Port'
    )
    ld06_topic = DeclareLaunchArgument(
        name='ld06_topic',
        default_value='scan',
        description='LD06 Topic Name'
    )
    ld06_frame = DeclareLaunchArgument(
        name='ld06_frame',
        default_value='laser_link',
        description='Lidar Frame ID'
    )
    ld06_range_threshold = DeclareLaunchArgument(
        name='ld06_range_threshold', 
        default_value='0.2',
        description='Range Threshold'
    )
    ldlidar = Node(
        package='ldlidar',
        executable='ldlidar',
        name='ldlidar',
        output='screen',
        parameters=[
            {'serial_port': LaunchConfiguration("ld06_serial_port")},
            {'topic_name': LaunchConfiguration("ld06_topic")},
            {'lidar_frame': LaunchConfiguration("ld06_frame")},
            {'range_threshold': LaunchConfiguration("ld06_range_threshold")}
        ]
    )
    return LaunchDescription([
        create_driver,
        create_2_robot_state_publisher,
        ld06_serial_port,
        ld06_topic,
        ld06_frame,
        ld06_range_threshold,
        ldlidar
    ])
