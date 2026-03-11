# Lab 8 无 GUI 仿真启动（Dev Container 无显示器时使用）
# 使用 -s --headless-rendering 在无 X 环境下运行
# 用法: ros2 launch lab8 ignition_headless.launch.py

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')
    pkg_turtlebot4_description = get_package_share_directory(
        'turtlebot4_description')
    pkg_irobot_create_ignition_bringup = get_package_share_directory(
        'irobot_create_ignition_bringup')
    pkg_irobot_create_description = get_package_share_directory(
        'irobot_create_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Resource path（与 turtlebot4 ignition 一致）
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_turtlebot4_ignition_bringup, 'worlds'), ':' +
            os.path.join(pkg_irobot_create_ignition_bringup, 'worlds'), ':' +
            str(Path(pkg_turtlebot4_description).parent.resolve()), ':' +
            str(Path(pkg_irobot_create_description).parent.resolve())])

    # Headless: -s 仅服务端, --headless-rendering 无 X 渲染, -r 自动运行
    gz_sim_launch = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', 'maze.sdf -s -r -v 4 --headless-rendering'),
        ]
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock' + '@rosgraph_msgs/msg/Clock' + '[ignition.msgs.Clock'
        ])

    ld = LaunchDescription()
    ld.add_action(ign_resource_path)
    ld.add_action(ignition)
    ld.add_action(clock_bridge)
    return ld
