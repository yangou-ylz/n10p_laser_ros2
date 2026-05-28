#!/usr/bin/python3
"""N10P Nav2 导航启动文件
启动: 传感器 + 地图服务器 + AMCL 定位 + 规划器 + 控制器 + BT + RViz2
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():

    nav_dir = get_package_share_directory('n10p_nav')
    bringup_dir = get_package_share_directory('n10p_bringup')
    driver_dir = get_package_share_directory('lslidar_driver')

    # ── 参数 ──────────────────────────────────────────
    map_yaml = LaunchConfiguration('map', default=os.path.join(
        nav_dir, '..', '..', '..', '..', '..', 'maps', 'n10p_map.yaml'))
    params_file = os.path.join(nav_dir, 'config', 'nav2_params_n10p.yaml')
    rviz_config = os.path.join(nav_dir, 'config', 'n10p_nav.rviz')

    # ══════════════════════════════════════════════════
    # 1. 传感器层
    # ══════════════════════════════════════════════════

    # 占位里程计 (飞控不在线时提供全零 odom + 飞控姿态)
    dummy_odom_node = Node(
        package='n10p_bringup',
        executable='dummy_odom_node',
        name='dummy_odom_node',
        output='screen',
    )

    # N10P 激光雷达驱动
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

    # 定位生命周期管理 (map_server + amcl)
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
    # 3. 导航层: 规划 + 控制 + BT
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

    # 导航生命周期管理 (planner + controller + bt_navigator)
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
    # 4. RViz2 (导航视图)
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
        dummy_odom_node,
        driver_node,
        static_tf_node,

        # 地图加载 → AMCL → 生命周期 (等传感器就绪)
        TimerAction(period=2.0, actions=[map_server_node]),
        TimerAction(period=3.0, actions=[amcl_node]),
        TimerAction(period=4.0, actions=[lifecycle_localization]),

        # 导航栈 (等定位就绪后启动)
        TimerAction(period=5.0, actions=[planner_node]),
        TimerAction(period=5.0, actions=[controller_node]),
        TimerAction(period=5.0, actions=[bt_navigator_node]),
        TimerAction(period=6.0, actions=[lifecycle_navigation]),

        # RViz2 最后启动
        TimerAction(period=8.0, actions=[rviz_node]),
    ])
