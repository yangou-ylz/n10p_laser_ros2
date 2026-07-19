#!/usr/bin/python3
"""N10P 一键建图 (带 EKF 融合 + 自动保存初始 yaw)

原理:
  bringup 启动后 6 秒, FC 数据稳定 → 自动读取 FC yaw
  → 保存到 maps/slam_yaw.txt → 启动 SLAM

用法:
  ros2 launch n10p_slam slam_ekf_launch.py
"""

import sys, os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # yaw_util 在 n10p_nav 的 launch 目录
    sys.path.insert(0, os.path.join(get_package_share_directory('n10p_nav'), 'launch'))

    bringup_dir = get_package_share_directory('n10p_bringup')
    slam_dir = get_package_share_directory('n10p_slam')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'n10p_bringup_launch.py')),
        launch_arguments={'scan_source': 'wired', 'use_ekf': 'true'}.items(),
    )

    def save_yaw_then_launch_slam(context):
        """bringup 6 秒后: 读 FC yaw → 保存 → 启 SLAM"""
        from yaw_util import read_fc_yaw, save_slam_yaw
        yaw = read_fc_yaw(timeout=5.0, samples=30)
        if yaw is not None:
            save_slam_yaw(yaw)
        else:
            print('[SLAM初始化] ⚠ 无法读取 FC yaw, 使用 yaw=0')

        slam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_dir, 'launch', 'slam_only_launch.py')),
            launch_arguments={'launch_rviz': 'false'}.items(),
        )
        return [slam_launch]

    return LaunchDescription([
        bringup_launch,
        TimerAction(period=6.0, actions=[
            OpaqueFunction(function=save_yaw_then_launch_slam),
        ]),
    ])
