#!/usr/bin/python3
"""N10P 桌面测试模式 — 用真实 N10P 雷达 + 键盘模拟里程计 + Nav2 导航
使用方法:
  终端1: ros2 run n10p_bringup keyboard_odom_node   (键盘控制虚拟里程计)
  终端2: ros2 launch n10p_nav desktop_test_launch.py (所有其他节点)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():

    nav_dir = get_package_share_directory('n10p_nav')
    driver_dir = get_package_share_directory('lslidar_driver')

    # ── 参数 ──────────────────────────────────────────
    map_yaml = LaunchConfiguration('map', default=os.path.join(
        nav_dir, '..', '..', '..', '..', '..', 'maps', 'n10p_map.yaml'))
    params_file = os.path.join(nav_dir, 'config', 'nav2_params_n10p.yaml')
    rviz_config = os.path.join(nav_dir, 'config', 'n10p_nav.rviz')

    # ══════════════════════════════════════════════════
    # 1. 真实 N10P 激光雷达驱动
    # ══════════════════════════════════════════════════
    driver_node = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        parameters=[os.path.join(driver_dir, 'params', 'lsx10.yaml')],
    )

    # 静态 TF: base_link → laser_frame
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0', '0', '-0.1', '0', '0', '0', 'base_link', 'laser_frame'],
    )

    # ══════════════════════════════════════════════════
    # 2. 定位层: 地图 + AMCL
    # ══════════════════════════════════════════════════
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_yaml}],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
    )

    lifecycle_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
            'bond_timeout': 5.0,
        }],
    )

    # ══════════════════════════════════════════════════
    # 3. 导航栈: 规划器 + 控制器 + 行为树
    # ══════════════════════════════════════════════════
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
    )

    lifecycle_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['planner_server', 'controller_server', 'bt_navigator'],
            'bond_timeout': 5.0,
        }],
    )

    # ══════════════════════════════════════════════════
    # 4. RViz2
    # ══════════════════════════════════════════════════
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=map_yaml, description='地图 yaml 路径'),

        # 传感器立即启动
        driver_node,
        static_tf_node,

        # 定位: 地图 → AMCL → 生命周期
        TimerAction(period=2.0, actions=[map_server_node]),
        TimerAction(period=3.0, actions=[amcl_node]),
        TimerAction(period=4.0, actions=[lifecycle_localization]),

        # 导航栈
        TimerAction(period=5.0, actions=[planner_node]),
        TimerAction(period=5.0, actions=[controller_node]),
        TimerAction(period=5.0, actions=[bt_navigator_node]),
        TimerAction(period=6.0, actions=[lifecycle_navigation]),

        # RViz2 最后启动
        TimerAction(period=8.0, actions=[rviz_node]),
    ])
