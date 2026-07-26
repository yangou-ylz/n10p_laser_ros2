#!/usr/bin/python3
"""N10P 一键建图 (带 EKF 融合)

用法:
  ros2 launch n10p_slam slam_ekf_launch.py

注意: 不再依赖 FC yaw。建图时地图的坐标系由 slam_toolbox 自己维护，
  导航时用户在 RViz 手动标一次初始位姿即可。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = get_package_share_directory('n10p_bringup')
    slam_dir = get_package_share_directory('n10p_slam')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'n10p_bringup_launch.py')),
        launch_arguments={'scan_source': 'wired', 'use_ekf': 'true'}.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'slam_only_launch.py')),
        launch_arguments={'launch_rviz': 'false'}.items(),
    )

    return LaunchDescription([
        bringup_launch,
        TimerAction(period=6.0, actions=[slam_launch]),
    ])
