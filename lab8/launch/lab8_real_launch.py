#!/usr/bin/env python3
# Lab 8 真机启动：SLAM + Nav2 + orchestrator + explore + aruco_detector

import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _get_script_path(script_name):
    """Get path to script; fallback to source when package/install broken."""
    # Try ament index first
    try:
        pkg_prefix = get_package_prefix('lab8')
        path = os.path.join(pkg_prefix, 'lib', 'lab8', script_name)
        if os.path.exists(path):
            return os.path.abspath(path)
    except Exception:
        pass
    # Fallback: this launch is at .../share/lab8/launch/ or .../src/lab8/launch/
    _dir = os.path.dirname(os.path.abspath(__file__))
    # If in install: .../install/lab8/share/lab8/launch -> go up to install, then src
    # If in source: .../src/lab8/launch -> scripts is ../scripts/
    for _ in range(5):
        scripts_path = os.path.join(_dir, 'scripts', script_name)
        if os.path.isfile(scripts_path):
            return scripts_path
        if os.path.basename(_dir) == 'lab8' and os.path.isdir(os.path.join(_dir, 'scripts')):
            return os.path.join(_dir, 'scripts', script_name)
        _dir = os.path.dirname(_dir)
        if not _dir or _dir == os.path.dirname(_dir):
            break
    raise FileNotFoundError(f"Cannot find {script_name}")


def generate_launch_description():
    pkg_turtlebot4_navigation = get_package_share_directory('turtlebot4_navigation')
    try:
        pkg_lab8 = get_package_share_directory('lab8')
    except Exception:
        pkg_lab8 = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
    aruco_params = os.path.join(pkg_lab8, 'config', 'aruco_params.yaml')
    nav2_params = os.path.join(pkg_lab8, 'config', 'nav2.yaml')
    slam_params = os.path.join(pkg_lab8, 'config', 'slam.yaml')

    slam_launch = PathJoinSubstitution([
        pkg_turtlebot4_navigation, 'launch', 'slam.launch.py'])
    nav2_launch = PathJoinSubstitution([
        pkg_turtlebot4_navigation, 'launch', 'nav2.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument('skip_undock', default_value='false',
                              description='Skip undock (set true if robot already off dock)'),
        DeclareLaunchArgument('image_topic', default_value='/oakd/rgb/preview/image_raw',
                              description='RGB image topic for ArUco'),
        DeclareLaunchArgument('camera_info_topic', default_value='/oakd/rgb/preview/camera_info',
                              description='Camera info topic'),

        # SLAM with custom params (base_frame=base_footprint)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([slam_launch]),
            launch_arguments=[
                ('use_sim_time', 'false'),
                ('params', slam_params),
            ]
        ),

        # Nav2 with custom params (bond_timeout=20 to allow DWB critics to load)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch]),
            launch_arguments=[
                ('use_sim_time', 'false'),
                ('params_file', nav2_params),
            ]
        ),

        # odom -> base_link TF bridge (Create3 publishes /odom topic but not TF)
        ExecuteProcess(
            cmd=['python3', _get_script_path('odom_to_tf.py'),
                 '--ros-args', '-p', 'use_sim_time:=false'],
            output='screen',
            shell=False,
        ),

        # ArUco detector (ExecuteProcess to avoid symlink path issues)
        ExecuteProcess(
            cmd=['python3', _get_script_path('aruco_detector.py'),
                 '--ros-args', '--params-file', aruco_params,
                 '-p', 'use_sim_time:=false'],
            output='screen',
            shell=False,
        ),

        # Explore action server
        ExecuteProcess(
            cmd=['python3', _get_script_path('explore_action_server.py'),
                 '--ros-args', '-p', 'use_sim_time:=false'],
            output='screen',
            shell=False,
        ),

        # Orchestrator
        ExecuteProcess(
            cmd=['python3', _get_script_path('orchestrator.py'),
                 '--ros-args', '-p', 'use_sim_time:=false',
                 '-p', ['skip_undock:=', LaunchConfiguration('skip_undock')]],
            output='screen',
            shell=False,
        ),
    ])
