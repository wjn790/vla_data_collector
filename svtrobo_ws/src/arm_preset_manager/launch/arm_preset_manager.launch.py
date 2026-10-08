import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def spawn_arm_nodes(context, *args, **kwargs):
    """生成 robot_state_publisher + ros2_control_node（bimanual 双臂）"""
    right_can = context.perform_substitution(LaunchConfiguration('right_can_interface'))
    left_can = context.perform_substitution(LaunchConfiguration('left_can_interface'))
    hand = context.perform_substitution(LaunchConfiguration('hand'))

    # 处理 xacro — bimanual 模式
    xacro_path = os.path.join(
        get_package_share_directory('openarm_description'),
        'urdf', 'robot', 'v10.urdf.xacro'
    )
    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            'arm_type': 'v10',
            'bimanual': 'true',
            'use_fake_hardware': 'false',
            'ros2_control': 'true',
            'right_can_interface': right_can,
            'left_can_interface': left_can,
            'hand': hand,
            'can_fd': 'true',
        }
    ).toprettyxml(indent='  ')

    # bimanual controller 配置
    controllers_file = os.path.join(
        get_package_share_directory('openarm_bringup'),
        'config', 'v10_controllers', 'openarm_v10_bimanual_controllers.yaml'
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=[{'robot_description': robot_description}, controllers_file],
    )

    return [robot_state_pub, ros2_control_node]


def generate_launch_description():
    pkg_dir = get_package_share_directory('arm_preset_manager')
    config_file = os.path.join(pkg_dir, 'config', 'arm_presets.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('joy_topic', default_value='/f710/joy'),
        DeclareLaunchArgument('config_file', default_value=config_file),
        DeclareLaunchArgument('right_can_interface', default_value='can0'),
        DeclareLaunchArgument('left_can_interface', default_value='can1'),
        DeclareLaunchArgument('hand', default_value='true'),

        # 启动双臂硬件
        OpaqueFunction(function=spawn_arm_nodes),

        # joint_state_broadcaster（延迟1秒）
        TimerAction(period=1.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen',
            ),
        ]),

        # left_forward_position_controller（延迟1.5秒）
        TimerAction(period=1.5, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['left_forward_position_controller', '-c', '/controller_manager'],
                output='screen',
            ),
        ]),

        # right_forward_position_controller（延迟1.5秒）
        TimerAction(period=1.5, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['right_forward_position_controller', '-c', '/controller_manager'],
                output='screen',
            ),
        ]),

        # left_gripper_controller（延迟2秒）
        TimerAction(period=2.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['left_gripper_controller', '-c', '/controller_manager'],
                output='screen',
            ),
        ]),

        # right_gripper_controller（延迟2秒）
        TimerAction(period=2.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['right_gripper_controller', '-c', '/controller_manager'],
                output='screen',
            ),
        ]),

        # 预设管理器（延迟5秒）
        TimerAction(period=5.0, actions=[
            Node(
                package='arm_preset_manager',
                executable='preset_manager_node',
                name='arm_preset_manager',
                output='screen',
                parameters=[{
                    'config_file': LaunchConfiguration('config_file'),
                    'joy_topic': LaunchConfiguration('joy_topic'),
                }],
            ),
        ]),
    ])
