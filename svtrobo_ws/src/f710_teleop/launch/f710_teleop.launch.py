import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    启动 F710 手柄遥控节点。

    优先使用源码目录下的 config/f710_teleop.yaml（改完即生效，无需重新编译），
    不存在时再用安装目录的 config。
    """
    # 优先用源码里的 config（由 install 路径反推 src），改 yaml 后不打包也能生效
    pkg_share = get_package_share_directory('f710_teleop')
    config_file = os.path.join(pkg_share, 'config', 'f710_teleop.yaml')
    parts = os.path.normpath(pkg_share).split(os.sep)
    if 'install' in parts:
        idx = parts.index('install')
        src_base = os.sep.join(parts[:idx] + ['src', 'f710_teleop'])
        src_config = os.path.join(src_base, 'config', 'f710_teleop.yaml')
        if os.path.isfile(src_config):
            config_file = src_config

    f710_node = Node(
        package='f710_teleop',
        executable='my_controller_node',
        name='my_controller_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([f710_node])

