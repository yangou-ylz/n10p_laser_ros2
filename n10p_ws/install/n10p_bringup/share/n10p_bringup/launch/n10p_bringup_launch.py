#!/usr/bin/python3
# ============================================================================
# n10p_bringup_launch.py — 传感器全开启动文件
# ============================================================================
# 同时启动三个节点：飞控桥接 + 雷达驱动 + 静态 TF
# 适合：需要飞控里程计 + 雷达数据的场景（如 SLAM 建图配合飞控、Nav2 导航）
#
# 重要：这个文件启动了雷达驱动，所以不能和 slam_launch.py（也自带驱动）同时运行！
#       串口冲突 → 驱动崩溃(double free)
# ============================================================================
"""启动匿名凌霄飞控桥接节点 + N10P 驱动 (+ 可选 EKF 融合)"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node        # 普通节点（非生命周期管理）
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
import os


def generate_launch_description():

    scan_source = LaunchConfiguration('scan_source', default='wired')
    is_wireless = PythonExpression(["'", scan_source, "' == 'wireless'"])
    use_ekf = LaunchConfiguration('use_ekf', default='true')

    # ══════════════════════════════════════════════════
    # 启动前强制清理 (先跑完, 节点延迟2秒再启动, 避免竞态)
    # ══════════════════════════════════════════════════
    cleanup_all = ExecuteProcess(
        cmd=['bash', '-c',
             'for pid in $(lsof -t /dev/ttyUSB0 2>/dev/null); do kill -9 $pid 2>/dev/null; done;'
             'for pid in $(lsof -t /dev/ttyACM0 2>/dev/null); do kill -9 $pid 2>/dev/null; done;'
             'rm -f /dev/shm/fastrtps_* 2>/dev/null;'
             'exit 0'],
        name='cleanup_pre_launch',
    )

    # ══════════════════════════════════════════════════
    # 所有节点 (延迟2秒, 确保 cleanup 已完成)
    # ══════════════════════════════════════════════════
    bridge_params = os.path.join(
        get_package_share_directory('n10p_bringup'), 'params', 'ano_bridge.yaml')

    # 自动识别 FC 数据口: 扫描 /dev/ttyUSB* @500k, 找发 0xAA 帧的那个
    def _detect_fc_port():
        import subprocess
        script = os.path.expanduser('~/n10p_leishen/n10p_ws/scripts/auto_detect_serial.py')
        try:
            r = subprocess.run(['python3', script], capture_output=True, text=True, timeout=10)
            for line in r.stdout.split('\n'):
                if 'N10P_FC_DATA=' in line:
                    port = line.split('=')[1].strip()
                    print(f'\n[bringup] 自动识别 FC 数据口: {port}\n')
                    return port
        except Exception as e:
            print(f'\n[bringup] 串口识别失败: {e}, 回退到 /dev/ttyUSB1\n')
        return '/dev/ttyUSB1'

    fc_port = _detect_fc_port()

    # use_ekf=true 时 ano_bridge 不发 TF (由 EKF 节点发)，避免两个 TF 源冲突
    bridge_node = TimerAction(period=2.0, actions=[Node(
        package='n10p_bringup', executable='ano_bridge_node',
        name='ano_bridge_node', output='screen',
        parameters=[bridge_params, {'serial_port': fc_port,
                    'publish_tf': PythonExpression(["'", use_ekf, "' != 'true'"])}])])

    driver_params = os.path.join(
        get_package_share_directory('lslidar_driver'), 'params', 'lsx10.yaml')
    driver_node = TimerAction(period=2.0, actions=[Node(
        package='lslidar_driver', executable='lslidar_driver_node',
        name='lslidar_driver_node', output='screen', parameters=[driver_params],
        condition=UnlessCondition(is_wireless))])

    wifi_bridge_node = TimerAction(period=2.0, actions=[Node(
        package='n10p_bringup', executable='n10p_wifi_bridge_node',
        name='n10p_wifi_bridge_node', output='screen',
        parameters=[{'host': '192.168.0.184', 'port': 8888}],
        condition=IfCondition(is_wireless))])

    static_tf_node = TimerAction(period=2.0, actions=[Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0','0','+0.1','0','0','0','base_link','laser_frame'])])

    ekf_config = os.path.join(get_package_share_directory('n10p_fusion'), 'config', 'ekf.yaml')
    ekf_node = TimerAction(period=2.0, actions=[Node(
        package='n10p_fusion', executable='imu_filter_node',
        name='imu_filter_node', output='screen', parameters=[ekf_config],
        condition=IfCondition(use_ekf))])

    return LaunchDescription([
        cleanup_all,
        DeclareLaunchArgument('scan_source', default_value='wired',
                              description='雷达数据源: wired | wireless'),
        DeclareLaunchArgument('use_ekf', default_value='true',
                              description='启用 EKF 融合 (默认=true)'),
        bridge_node, driver_node, wifi_bridge_node, static_tf_node, ekf_node,
    ])
