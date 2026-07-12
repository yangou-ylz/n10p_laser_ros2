#!/usr/bin/env python3
"""
ano_bridge_node.py — 凌霄飞控 → ROS2 桥接节点
================================================

使用分层架构：
  ano_protocol.py  — 纯协议层（帧描述符、校验、编解码）
  ano_transport.py — 传输层（串口管理、后台线程、帧同步、回调分发）

本节点职责：将飞控串口数据转换为 ROS2 标准消息并发布。

发布话题:
  /odom       — nav_msgs/Odometry    (位置+速度+姿态, 20Hz)
  /imu        — sensor_msgs/Imu       (加速度+角速度+姿态, ~100Hz)
  /battery    — sensor_msgs/BatteryState (电压+电流, ~1Hz)
  /fc_status  — n10p_bringup/FCStatus (飞行模式+解锁状态, ~20Hz)

发布 TF:
  odom → base_link (动态, 20Hz)

参数:
  serial_port   — 串口路径 (默认 /dev/ttyAMA0)
  baud_rate     — 波特率 (默认 500000)
  frame_id      — 机器人本体坐标系 (默认 base_link)
  odom_frame_id — 里程计坐标系 (默认 odom)
  publish_tf    — 是否发布 odom→base_link TF (默认 true)
  acc_scale     — 加速度量纲转换系数 (默认 0.004788 m/s²/LSB)
  gyr_scale     — 角速度量纲转换系数 (默认 0.001065 rad/s/LSB)
  gyr_offset_x/y/z — 角速度零偏补偿 (默认 0.0)
  pub_rate      — 里程计发布频率 (默认 20.0 Hz)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from .ano_transport import SerialTransport


class AnoBridgeNode(Node):
    """凌霄飞控 → ROS2 桥接节点"""

    def __init__(self):
        super().__init__('ano_bridge_node')

        # ── 参数声明 ──────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 500000)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('acc_scale', 0.004788)     # m/s² per LSB (±16g)
        self.declare_parameter('gyr_scale', 0.001065)     # rad/s per LSB (±2000dps)
        self.declare_parameter('gyr_offset_x', 0.0)
        self.declare_parameter('gyr_offset_y', 0.0)
        self.declare_parameter('gyr_offset_z', 0.0)
        self.declare_parameter('pub_rate', 20.0)           # 里程计发布频率 Hz

        # ── 参数读取 ──────────────────────────────────────
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.acc_scale = self.get_parameter('acc_scale').value
        self.gyr_scale = self.get_parameter('gyr_scale').value
        self.gyr_offset = [
            self.get_parameter('gyr_offset_x').value,
            self.get_parameter('gyr_offset_y').value,
            self.get_parameter('gyr_offset_z').value,
        ]
        pub_rate = self.get_parameter('pub_rate').value

        # ── 数据缓存（由传输层回调写入，ROS2 定时器读取） ──
        self.pos_x = 0.0       # m
        self.pos_y = 0.0       # m
        self.pos_z = 0.0       # m
        self.vel_x = 0.0       # m/s
        self.vel_y = 0.0       # m/s
        self.vel_z = 0.0       # m/s
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0  # w,x,y,z
        self.gyr = [0.0, 0.0, 0.0]   # rad/s
        self.acc = [0.0, 0.0, 0.0]   # m/s²
        self.fusion_sta = 0           # 融合状态

        # 电池状态
        self.voltage = 0.0    # V
        self.current = 0.0    # A

        # 飞控状态
        self.fc_mode = 0
        self.fc_mode_str = '未知'
        self.fc_unlocked = False
        self.fc_cmd_cid = 0
        self.fc_cmd_0 = 0
        self.fc_cmd_1 = 0

        # ── QoS 配置 ──────────────────────────────────────
        # Best Effort: 里程计数据每秒 20 帧，丢一帧马上有新的。
        # 用 Reliable 会导致队列堆积、延迟增大。
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # /battery 低频帧用 Reliable，保证不丢
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ── 发布者 ────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/odom', sensor_qos)
        self.imu_pub = self.create_publisher(Imu, '/imu', sensor_qos)
        self.battery_pub = self.create_publisher(BatteryState, '/battery', reliable_qos)
        # /fc_status 使用标准消息中已有的类型 — 这里用简单的日志发布
        # TODO: 如需结构化飞控状态消息，可定义自定义 msg
        self.get_logger().info('飞控状态通过日志输出，/fc_status 待自定义消息定义后发布')

        # ── TF 广播器 ─────────────────────────────────────
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── 传输层 ────────────────────────────────────────
        self._transport = SerialTransport(port, baud)

        # 注册帧回调（回调在传输层的后台线程中执行）
        self._transport.register_callback(0x01, self._on_imu_raw)
        self._transport.register_callback(0x02, self._on_baro_mag)
        self._transport.register_callback(0x03, self._on_euler)
        self._transport.register_callback(0x04, self._on_quaternion)
        self._transport.register_callback(0x05, self._on_altitude)
        self._transport.register_callback(0x06, self._on_fc_status)
        self._transport.register_callback(0x07, self._on_velocity)
        self._transport.register_callback(0x08, self._on_position)
        self._transport.register_callback(0x0D, self._on_battery)
        self._transport.register_callback(0x0E, self._on_module_status)

        # 启动传输层（后台线程开始读串口）
        if not self._transport.start():
            self.get_logger().fatal(f'无法打开串口 {port}，节点将继续运行但无数据')

        # ── 定时器：固定频率发布里程计 ────────────────────
        pub_period = 1.0 / pub_rate
        self._pub_timer = self.create_timer(pub_period, self._publish_odometry)

        # ── 定时器：每秒打印统计 ──────────────────────────
        self._stats_timer = self.create_timer(10.0, self._print_stats)

        self.get_logger().info(f'凌霄飞控桥接节点已启动 ({port} @ {baud} bps)')

    # ═══════════════════════════════════════════════════════════════
    # 帧回调（在传输层后台线程中执行，只做轻量数据更新）
    # ═══════════════════════════════════════════════════════════════

    def _on_imu_raw(self, d: dict) -> None:
        """0x01 惯性传感器原始数据 → 缓存加速度+角速度"""
        if 'error' in d:
            return
        self.acc = [
            d['acc_x'] * self.acc_scale,
            d['acc_y'] * self.acc_scale,
            d['acc_z'] * self.acc_scale,
        ]
        self.gyr = [
            d['gyr_x'] * self.gyr_scale + self.gyr_offset[0],
            d['gyr_y'] * self.gyr_scale + self.gyr_offset[1],
            d['gyr_z'] * self.gyr_scale + self.gyr_offset[2],
        ]
        self._publish_imu()

    def _on_baro_mag(self, d: dict) -> None:
        """0x02 气压计+磁力计 → 缓存气压高度"""
        if 'error' in d:
            return
        # 气压高度以米为单位缓存
        self.baro_alt = d['baro_alt_cm'] * 0.01

    def _on_euler(self, d: dict) -> None:
        """0x03 欧拉角（低频，仅作参考）"""
        if 'error' in d:
            return
        self.fusion_sta = d['fusion_sta']

    def _on_quaternion(self, d: dict) -> None:
        """0x04 四元数 → 缓存姿态（主姿态来源，~67Hz）"""
        if 'error' in d:
            return
        self.q0 = d['w']
        self.q1 = d['x']
        self.q2 = d['y']
        self.q3 = d['z']
        self.fusion_sta = d['fusion_sta']

    def _on_altitude(self, d: dict) -> None:
        """0x05 融合高度 → 缓存 Z 位置"""
        if 'error' in d:
            return
        self.pos_z = d['alt_fused_cm'] * 0.01

    def _on_fc_status(self, d: dict) -> None:
        """0x06 飞控状态 → 缓存模式和解锁状态"""
        if 'error' in d:
            return
        self.fc_mode = d['mode']
        self.fc_mode_str = d.get('mode_str', f'未知({d["mode"]})')
        self.fc_unlocked = bool(d['unlocked'])
        self.fc_cmd_cid = d['cmd_cid']
        self.fc_cmd_0 = d['cmd_0']
        self.fc_cmd_1 = d['cmd_1']

    def _on_velocity(self, d: dict) -> None:
        """0x07 飞行速度 → 缓存线速度"""
        if 'error' in d:
            return
        self.vel_x = d['vel_x_cms'] * 0.01
        self.vel_y = d['vel_y_cms'] * 0.01
        self.vel_z = d['vel_z_cms'] * 0.01

    def _on_position(self, d: dict) -> None:
        """0x08 XY 位移 → 缓存水平位置"""
        if 'error' in d:
            return
        self.pos_x = d['pos_x_cm'] * 0.01
        self.pos_y = d['pos_y_cm'] * 0.01

    def _on_battery(self, d: dict) -> None:
        """0x0D 电池信息 → 缓存电压/电流并发布"""
        if 'error' in d:
            return
        self.voltage = d['voltage_v']
        self.current = d['current_a']
        self._publish_battery()

    def _on_module_status(self, d: dict) -> None:
        """0x0E 外接模块状态 → 记录日志"""
        if 'error' in d:
            return
        # 状态变化时打印日志
        gps_str = d.get('sta_gps_str', '?')
        if gps_str != '无数据':
            self.get_logger().debug(
                f'模块状态: 速度={d.get("sta_gvel_str","?")} '
                f'位置={d.get("sta_gpos_str","?")} '
                f'GPS={gps_str} '
                f'高度辅助={d.get("sta_alt_str","?")}'
            )

    # ═══════════════════════════════════════════════════════════════
    # ROS2 消息发布（由定时器或帧回调触发）
    # ═══════════════════════════════════════════════════════════════

    def _publish_odometry(self) -> None:
        """发布 /odom 消息 + odom→base_link TF (固定频率)"""
        now = self.get_clock().now()

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.frame_id

        # 位置 — 2D SLAM 不需要高度，飞控 XY/高度无外部定位时不可靠
        # 姿态仍由四元数提供（用于旋转先验）
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0

        # 姿态（四元数 w,x,y,z）
        msg.pose.pose.orientation.w = self.q0
        msg.pose.pose.orientation.x = self.q1
        msg.pose.pose.orientation.y = self.q2
        msg.pose.pose.orientation.z = self.q3

        # 速度
        msg.twist.twist.linear.x = self.vel_x
        msg.twist.twist.linear.y = self.vel_y
        msg.twist.twist.linear.z = self.vel_z
        msg.twist.twist.angular.x = self.gyr[0]
        msg.twist.twist.angular.y = self.gyr[1]
        msg.twist.twist.angular.z = self.gyr[2]

        # 协方差（经验值）
        # 位置协方差拉满（飞控0x08位置不可靠，完全交给扫描匹配）
        # 姿态协方差设为1.0 rad²（±57°），飞控四元数仅作初始化参考，
        # 实际旋转由 slam_toolbox 扫描匹配主导纠正
        cov_pose = [
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ]
        cov_twist = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01,
        ]
        msg.pose.covariance = cov_pose
        msg.twist.covariance = cov_twist

        self.odom_pub.publish(msg)

        # 发布 TF
        if self.publish_tf:
            self._publish_odom_tf(now)

    def _publish_odom_tf(self, now) -> None:
        """发布 odom → base_link 动态 TF"""
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.frame_id
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = self.q0
        t.transform.rotation.x = self.q1
        t.transform.rotation.y = self.q2
        t.transform.rotation.z = self.q3
        self.tf_broadcaster.sendTransform(t)

    def _publish_imu(self) -> None:
        """发布 /imu 消息（在收到 IMU 原始数据帧时触发，~100Hz）"""
        now = self.get_clock().now()

        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id

        # 姿态
        msg.orientation.w = self.q0
        msg.orientation.x = self.q1
        msg.orientation.y = self.q2
        msg.orientation.z = self.q3

        # 角速度
        msg.angular_velocity.x = self.gyr[0]
        msg.angular_velocity.y = self.gyr[1]
        msg.angular_velocity.z = self.gyr[2]

        # 线加速度
        msg.linear_acceleration.x = self.acc[0]
        msg.linear_acceleration.y = self.acc[1]
        msg.linear_acceleration.z = self.acc[2]

        # 协方差
        msg.orientation_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001,
        ]
        msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01,
        ]
        msg.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1,
        ]

        self.imu_pub.publish(msg)

    def _publish_battery(self) -> None:
        """发布 /battery 消息（在收到电池帧时触发，~1Hz）"""
        now = self.get_clock().now()

        msg = BatteryState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.voltage = self.voltage
        msg.current = self.current
        # 简单估算：假设 3S 电池，满电 12.6V，截止 10.5V
        if self.voltage > 0:
            msg.percentage = max(0.0, min(1.0, (self.voltage - 10.5) / (12.6 - 10.5)))
        else:
            msg.percentage = float('nan')
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present = True

        self.battery_pub.publish(msg)

    def _print_stats(self) -> None:
        """定期打印帧率统计"""
        stats, errors = self._transport.stats()
        total = sum(stats.values())
        if total == 0:
            self.get_logger().info('统计: 尚未收到任何帧')
            return

        lines = []
        for cmd in sorted(stats.keys()):
            count = stats[cmd]
            from .ano_protocol import FRAME_NAME
            name = FRAME_NAME.get(cmd, f'0x{cmd:02X}')
            lines.append(f'{name}={count}')
        self.get_logger().info(f'帧统计 (校验错误={errors}): ' + ', '.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = AnoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, '_transport'):
            node._transport.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
