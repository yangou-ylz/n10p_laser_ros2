#!/usr/bin/python3
"""N10P 一键导航 — 自动读取 FC yaw, 与建图时 slam_yaw 比较, 算初始朝向

原理:
  SLAM 建图时自动保存 FC yaw → maps/slam_yaw.txt
  导航时读取 slam_yaw + 当前 FC yaw → initial_yaw = nav_yaw - slam_yaw
  → AMCL 从正确朝向出发, 秒收敛

用法:
  ros2 launch n10p_nav nav_ekf_launch.py map:=/path/to/map.yaml
"""

import math, sys, os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sys.path.insert(0, os.path.join(get_package_share_directory('n10p_nav'), 'launch'))

    bringup_dir = get_package_share_directory('n10p_bringup')
    nav_dir = get_package_share_directory('n10p_nav')

    map_yaml = LaunchConfiguration('map', default=os.path.join(
        nav_dir, '..', '..', '..', '..', '..', 'maps', 'n10p_map.yaml'))

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'n10p_bringup_launch.py')),
        launch_arguments={'scan_source': 'wired', 'use_ekf': 'true'}.items(),
    )

    def launch_nav_with_auto_yaw(context):
        """bringup 启动 8 秒后: 读 slam_yaw + FC yaw → 算偏移 → 启导航"""
        from yaw_util import get_initial_yaw
        yaw = get_initial_yaw()

        nav_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_dir, 'launch', 'nav_only_launch.py')),
            launch_arguments={
                'map': map_yaml,
                'initial_yaw': str(yaw),
            }.items(),
        )
        return [nav_launch]

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=map_yaml,
                              description='地图 yaml 路径'),
        bringup_launch,
        TimerAction(period=8.0, actions=[
            OpaqueFunction(function=launch_nav_with_auto_yaw),
        ]),
    ])
