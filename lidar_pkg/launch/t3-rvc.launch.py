# launch/t3-rvc.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution
import os
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    lidar_config = PathJoinSubstitution([
        FindPackageShare('lidar_pkg'),
        'config',
        'waffle_params.yaml'
    ])

    yolo_config = PathJoinSubstitution([
        FindPackageShare('lidar_pkg'),
        'config',
        'yolo_params.yaml'
    ])

    tb3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_custom_maze1.launch.py'
            )
        )
    )

    lidar_node = Node(
        package='lidar_pkg',
        executable='lidar_analysis',
        name='waffle_rvc',
        namespace='/yang',
        parameters=[lidar_config],
        output='screen',
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('scan', '/scan')
        ]
    )

    image_node = Node(
        package='lidar_pkg',
        executable='yolo_detector',
        name='waffle_v4l2',
        namespace='/yang',
        parameters=[yolo_config],
        output='screen',
        remappings=[
            ('image_raw', '/camera/image_raw')
        ]
    )

    return LaunchDescription([
        tb3_gazebo,
        lidar_node,
        image_node
    ])