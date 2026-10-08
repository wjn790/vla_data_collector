
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('arm_preset_manager')
    config_file = os.path.join(pkg_share, 'config', 'arm_motion_player.yaml')

    return LaunchDescription([
        Node(
            package='arm_preset_manager',
            executable='motion_player_node',
            name='arm_motion_player',
            output='screen',
            parameters=[config_file],
        )
    ])
