# Lab 8 仿真一键启动
# 启动：仿真环境 + SLAM + Nav2 + 状态机 + 探索 + ArUco 检测
# 运行: ros2 launch lab8 lab8_launch.py
# 无显示器时: ros2 launch lab8 lab8_launch.py headless:=true

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_lab8 = get_package_share_directory('lab8')
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_nav = LaunchConfiguration('use_nav', default='false')
    headless = LaunchConfiguration('headless', default='false')  # 默认有 GUI，需显示器

    # turtlebot4_ignition（有 GUI）
    turtlebot4_ignition_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_ignition.launch.py']
    )
    sim_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot4_ignition_launch]),
        launch_arguments=[
            ('world', 'maze'),
            ('slam', 'true'),
            ('nav2', use_nav),
        ],
        condition=UnlessCondition(headless)
    )

    # headless：无 GUI，仅服务端，适合 Dev Container
    ignition_headless_launch = PathJoinSubstitution(
        [pkg_lab8, 'launch', 'ignition_headless.launch.py'])
    turtlebot4_spawn_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_spawn.launch.py'])

    ignition_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ignition_headless_launch]),
        condition=IfCondition(headless)
    )
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot4_spawn_launch]),
        launch_arguments=[
            ('world', 'maze'),
            ('slam', 'true'),
            ('nav2', use_nav),
        ],
        condition=IfCondition(headless)
    )

    # Lab 8 节点
    aruco_params = PathJoinSubstitution([pkg_lab8, 'config', 'aruco_params.yaml'])

    explore_server = Node(
        package='lab8',
        executable='explore_action_server',
        name='explore_action_server',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    aruco_detector = Node(
        package='lab8',
        executable='aruco_detector',
        name='aruco_detector',
        parameters=[aruco_params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    orchestrator = Node(
        package='lab8',
        executable='orchestrator',
        name='orchestrator',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'use_nav': use_nav},
            {'explore_max_duration': 180.0},
            {'approach_offset_m': 0.12},
            {'skip_undock': False},
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_nav', default_value='false'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true=无 GUI，容器内无显示时用；false=有 Gazebo 窗口'),
        sim_with_gui,
        ignition_headless,
        spawn,
        explore_server,
        aruco_detector,
        orchestrator,
    ])
