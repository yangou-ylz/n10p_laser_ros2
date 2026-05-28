#!/usr/bin/python3
"""启动匿名凌霄飞控桥接节点 + N10P 驱动"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():

    # ── 匿名桥接节点 ──────────────────────────────
    bridge_params = os.path.join(
        get_package_share_directory('n10p_bringup'),
        'params', 'ano_bridge.yaml')

    bridge_node = Node(
        package='n10p_bringup',
        executable='ano_bridge_node',
        name='ano_bridge_node',
        output='screen',
        parameters=[bridge_params],
    )

    # ── N10P 激光雷达驱动 ──────────────────────────
    driver_params = os.path.join(
        get_package_share_directory('lslidar_driver'),
        'params', 'lsx10.yaml')

    driver_node = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        parameters=[driver_params],
    )

    # ── 静态 TF: base_link → laser_frame ──────────
    # N10P 安装在无人机下方，坐标系: X前 Y左 Z上
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0', '0', '-0.1', '0', '0', '0', 'base_link', 'laser_frame'],
    )

    return LaunchDescription([
        bridge_node,
        driver_node,
        static_tf_node,
    ])
