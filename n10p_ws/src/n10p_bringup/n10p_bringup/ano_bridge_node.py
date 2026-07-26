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
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from geometry_msgs.msg import TransformStamped, TwistStamped
from tf2_ros import TransformBroadcaster

from .ano_transport import SerialTransport
from .rpi_pos_frame import (
    build_f5_frame, build_invalid_frame, build_hover_frame,
    FLAG_SLAM_VALID, FLAG_TARGET_VALID, FLAG_VISUAL_MODE)


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
        self.declare_parameter('vx_sign', -1.0)             # vx 符号: +1.0或-1.0, 匹配FC坐标系
        self.declare_parameter('vy_sign', -1.0)             # vy 符号: +1.0或-1.0, 匹配FC坐标系
        # 位置下行参数 (0xF5 帧)
        self.declare_parameter('pos_downlink_enable', False) # 启用位置下行
        self.declare_parameter('pos_downlink_rate', 50.0)    # 下行频率 Hz
        self.declare_parameter('pos_downlink_mode', 'waypoint')  # 'waypoint' | 'visual'
        self.declare_parameter('wp_x_cm', 100.0)             # 默认航点X cm
        self.declare_parameter('wp_y_cm', 0.0)               # 默认航点Y cm
        self.declare_parameter('wp_z_cm', 80.0)              # 默认航点Z cm

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
        self.vx_sign = self.get_parameter('vx_sign').value
        self.vy_sign = self.get_parameter('vy_sign').value

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
        self.vel_raw_pub = self.create_publisher(TwistStamped, '/fc_vel_raw', sensor_qos)  # 死区前原始速度 (诊断用)
        self.imu_pub = self.create_publisher(Imu, '/imu', sensor_qos)
        self._last_imu_pub_ts = 0.0               # IMU 限速: 上次发布时间
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

        # ── 位置下行：AMCL 定位 → 0xAA 帧 → 飞控 ──────────
        pos_downlink_enable = self.get_parameter('pos_downlink_enable').value
        pos_downlink_rate = self.get_parameter('pos_downlink_rate').value
        self._pos_downlink_enabled = pos_downlink_enable
        # AMCL 位姿缓存 (None = 未收到有效定位)
        self._amcl_x = None
        self._amcl_y = None
        self._amcl_z = None
        self._amcl_age = 0.0       # 距上次收到 AMCL 数据的秒数
        self._amcl_last_ts = None  # 上次收到 AMCL 的时间 (monotonic)

        # 订阅 AMCL 位姿
        from geometry_msgs.msg import PoseWithCovarianceStamped
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self._on_amcl_pose, 10)
        self.get_logger().info('已订阅 /amcl_pose，等待 AMCL 定位数据...')

        # 位置下行定时器 (50Hz)
        if pos_downlink_enable:
            pos_period = 1.0 / pos_downlink_rate
            self._pos_timer = self.create_timer(
                pos_period, self._send_position_downlink)
            self.get_logger().info(
                f'位置下行: 已启用, {pos_downlink_rate}Hz, '
                f'目标 STM32 UART2 (PD6 RX)')

        self.get_logger().info(f'凌霄飞控桥接节点已启动 ({port} @ {baud} bps)')
        self.get_logger().info(f'速度方向: vx_sign={self.vx_sign}, vy_sign={self.vy_sign}')

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
        # 限制 /imu 发布频率 ≤100Hz (IMU帧源501Hz, 跳过冗余帧省CPU)
        now_mono = time.monotonic()
        if now_mono - self._last_imu_pub_ts >= 0.01:
            self._last_imu_pub_ts = now_mono
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
        self.vel_x = self.vx_sign * d['vel_x_cms'] * 0.01
        self.vel_y = self.vy_sign * d['vel_y_cms'] * 0.01
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

        # 诊断: 发布死区前的原始 FC 速度到 /fc_vel_raw
        raw_twist = TwistStamped()
        raw_twist.header.stamp = now.to_msg()
        raw_twist.header.frame_id = self.frame_id
        raw_twist.twist.linear.x = self.vel_x
        raw_twist.twist.linear.y = self.vel_y
        raw_twist.twist.linear.z = self.vel_z
        self.vel_raw_pub.publish(raw_twist)

        # 速度 — 加死区过滤 FC 静止噪声 (<0.02m/s 视为零)
        FC_VEL_DEAD_ZONE = 0.02
        vx = self.vel_x
        vy = self.vel_y
        vz = self.vel_z
        if abs(vx) < FC_VEL_DEAD_ZONE: vx = 0.0
        if abs(vy) < FC_VEL_DEAD_ZONE: vy = 0.0
        if abs(vz) < FC_VEL_DEAD_ZONE: vz = 0.0
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.linear.z = vz
        msg.twist.twist.angular.x = self.gyr[0]
        msg.twist.twist.angular.y = self.gyr[1]
        msg.twist.twist.angular.z = self.gyr[2]

        # 协方差 (2026-07-12 实测: Yaw偏差0.85°@90°, σ<0.03° → 四元数A级可信)
        # 位置: 1.0 — 飞控 0x08 XY_Pos 无外部定位时不可靠, 交给 AMCL 扫描匹配
        # 姿态: 0.001 — 飞控四元数高度可信 (±1.8°), AMCL 可直接信任旋转分量
        cov_pose = [
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.001,
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

    # ═══════════════════════════════════════════════════════════════
    # 位置下行：AMCL 定位 → 0xAA 位置帧 → 串口 → 飞控
    # ═══════════════════════════════════════════════════════════════

    def _on_amcl_pose(self, msg) -> None:
        """
        AMCL 定位回调：缓存最新位姿用于位置下行。

        AMCL 发布的 /amcl_pose 是机器人在地图坐标系(map)中的位姿。
        我们将它作为"相对起飞点的绝对位移"发送给飞控。

        坐标系对齐:
          ROS map: x=前, y=左, z=上 (REP-105)
          飞控:    x=前, y=左, z=上
          → 直接对应，无需旋转

        单位转换: 米(m) → 厘米(cm)
        """
        self._amcl_x = msg.pose.pose.position.x * 100.0
        self._amcl_y = msg.pose.pose.position.y * 100.0
        self._amcl_z = msg.pose.pose.position.z * 100.0
        self._amcl_last_ts = time.monotonic()

    def _send_position_downlink(self) -> None:
        """
        定时器回调：根据当前模式构造 0xF5 帧并发送 (50Hz)。

        双模式:
          waypoint 模式: tar=预设航点, flags=0x03
          visual 模式:   tar=cur+K230偏移, flags=0x07 (视觉丢失时 tar=cur 悬停)

        AMCL 超时 (>200ms) → 自动发送全无效帧 (flags=0x00)
        """
        now = time.monotonic()

        # ── 检查 AMCL 新鲜度 ──────────────────────────
        if self._amcl_last_ts is not None:
            self._amcl_age = now - self._amcl_last_ts

        if (self._amcl_last_ts is None or self._amcl_age > 0.2
                or self._amcl_x is None):
            # SLAM 无效: 发全无效帧, 飞控暂停 PID 悬停
            frame = build_invalid_frame()
            self._transport.send_raw(frame)
            if self._amcl_last_ts is None or int(now) % 5 == 0:
                self.get_logger().warn(
                    f'0xF5↓: SLAM 无数据 (age={self._amcl_age:.1f}s) → 无效帧',
                    throttle_duration_sec=5.0)
            return

        # ── 模式选择 ──────────────────────────────────
        mode = self.get_parameter('pos_downlink_mode').value

        if mode == 'visual':
            # 视觉模式: 用 K230 偏移计算目标
            frame = self._build_visual_frame()
        else:
            # 航点模式: 用预设航点
            wp_x = self.get_parameter('wp_x_cm').value
            wp_y = self.get_parameter('wp_y_cm').value
            wp_z = self.get_parameter('wp_z_cm').value
            frame = build_f5_frame(
                self._amcl_x, self._amcl_y, self._amcl_z,
                wp_x, wp_y, wp_z,
                FLAG_SLAM_VALID | FLAG_TARGET_VALID)

        ok = self._transport.send_raw(frame)
        if not ok:
            self.get_logger().error('0xF5↓: 串口发送失败', throttle_duration_sec=2.0)

    def _build_visual_frame(self) -> bytes:
        """
        构造视觉模式帧 (0xF5 flags=0x07)。

        K230 占位: 当前 K230 未接入, 默认 tar=cur (悬停)。
        后续接入 K230 后, 读取 /k230_detection 话题的 dx/dy/dz 叠加到 cur。
        """
        # ── K230 占位 (后续从 /k230_detection 话题读取) ──
        dx, dy, dz = 0.0, 0.0, 0.0       # 默认无偏移 = 悬停
        target_valid = False               # 默认无视觉目标

        # TODO: 从 self._k230_dx 等缓存读取实际值 (订阅 /k230_detection)

        if target_valid:
            tar_x = self._amcl_x + dx
            tar_y = self._amcl_y + dy
            tar_z = self._amcl_z + dz
            flags = FLAG_SLAM_VALID | FLAG_TARGET_VALID | FLAG_VISUAL_MODE
        else:
            # 视觉丢失 → 悬停
            tar_x, tar_y, tar_z = self._amcl_x, self._amcl_y, self._amcl_z
            flags = FLAG_SLAM_VALID | FLAG_TARGET_VALID

        return build_f5_frame(
            self._amcl_x, self._amcl_y, self._amcl_z,
            tar_x, tar_y, tar_z, flags)


def main(args=None):
    rclpy.init(args=args)
    node = AnoBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # 先停后台串口线程, 再销毁 ROS2 资源 (避免回调在 context 销毁后 publish)
        if hasattr(node, '_transport'):
            node._transport.stop()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
