#!/usr/bin/python3
# ============================================================================
# ekf_odom_launch.py — 独立 EKF 里程计融合启动文件
# ============================================================================
# 不干涉 bringup、SLAM、Nav2 任何节点。
# 只启动一个 ekf_filter_node，订阅 /imu + /odom，发布过滤后的 odom→base_link TF。
#
# 用法:
#   ros2 launch n10p_fusion ekf_odom_launch.py
#
# 切换方案:
#   use_ekf:=false — 关闭 EKF, 使用原始 ano_bridge TF (向后兼容)
#   use_ekf:=true  — 启用 EKF, ano_bridge 的 TF 发布被禁用
# ============================================================================
"""独立 EKF 里程计融合节点 — robot_localization ekf_node 包装"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():

    use_ekf = LaunchConfiguration('use_ekf', default='false')

    ekf_config = os.path.join(
        get_package_share_directory('n10p_fusion'),
        'config', 'ekf.yaml')

    ekf_node = Node(
        package='n10p_fusion',
        executable='imu_filter_node',
        name='imu_filter_node',
        output='screen',
        parameters=[ekf_config],
        condition=IfCondition(LaunchConfiguration('use_ekf')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_ekf', default_value='false',
                              description='启用 EKF 滤波 (true=融合IMU, false=原始里程计)'),
        ekf_node,
    ])
