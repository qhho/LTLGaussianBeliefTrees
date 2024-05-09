from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_dir = get_package_share_directory('agent_bringup')
    rviz_config_path = os.path.join(pkg_dir, 'config', 'rviz_config.rviz')

    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'map'],
        name='static_transform_publisher',
        output='screen'
    )

    agent_gbt = Node(
        package='agent_gbt',
        executable='agent_gbt',
        output='screen',
        shell=True,
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        static_transform_publisher,
        agent_gbt,
        rviz2
    ])
