#!/usr/bin/python3
"""N10P SLAM 建图启动文件 — 手持建图模式（不需要飞控）
启动: 占位里程计 + N10P 雷达 + slam-toolbox + (可选 RViz2)

参数:
  scan_source:=wired    有线模式 (默认, lslidar_driver)
  scan_source:=wireless 无线模式 (ESP32 WiFi → n10p_wifi_bridge_node)
  launch_rviz:=false    是否启动 RViz2 (树莓派默认不启动, 开发机传 true)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os


def generate_launch_description():

    scan_source = LaunchConfiguration('scan_source', default='wired')
    is_wireless = PythonExpression(["'", scan_source, "' == 'wireless'"])

    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='false')

    # ── 1. 占位里程计 (飞控不在线时提供 odom → base_link TF) ──
    dummy_odom_node = Node(
        package='n10p_bringup',
        executable='dummy_odom_node',
        name='dummy_odom_node',
        output='screen',
    )

    # ── 2. N10P 激光雷达 (有线/无线可选) ─────────────
    driver_params = os.path.join(
        get_package_share_directory('lslidar_driver'),
        'params', 'lsx10.yaml')

    driver_node = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        parameters=[driver_params],
        condition=UnlessCondition(is_wireless),
    )

    wifi_bridge_node = Node(
        package='n10p_bringup',
        executable='n10p_wifi_bridge_node',
        name='n10p_wifi_bridge_node',
        output='screen',
        parameters=[{'host': '192.168.0.184', 'port': 8888}],
        condition=IfCondition(is_wireless),
    )

    # ── 3. 静态 TF: base_link → laser_frame ──────────
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0', '0', '-0.1', '0', '0', '0', 'base_link', 'laser_frame'],
    )

    # ── 4. SLAM: slam-toolbox online async ──────────
    slam_config = os.path.join(
        get_package_share_directory('n10p_slam'),
        'config', 'mapper_params_online_async.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config],
    )

    # ── 5. RViz2 (树莓派默认不启动, 电脑端独自运行) ──
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('n10p_slam'),
            'config', 'n10p_slam.rviz')],
        output='screen',
    )

    def launch_setup(context):
        nodes = [
            dummy_odom_node,
            driver_node,
            wifi_bridge_node,
            static_tf_node,
            TimerAction(period=3.0, actions=[slam_node]),
        ]
        if LaunchConfiguration('launch_rviz').perform(context).lower() == 'true':
            nodes.append(TimerAction(period=6.0, actions=[rviz_node]))
        return nodes

    return LaunchDescription([
        DeclareLaunchArgument('scan_source', default_value='wired',
                              description='雷达数据源: wired (有线) | wireless (ESP32 WiFi)'),
        launch_rviz_arg,
        OpaqueFunction(function=launch_setup),
    ])
