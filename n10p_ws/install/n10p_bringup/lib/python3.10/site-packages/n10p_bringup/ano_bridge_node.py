#!/usr/bin/env python3
"""
匿名凌霄飞控 串口解析节点 (ANO Protocol V7)
============================================
解析匿名数传接收到的飞控数据帧，发布 ROS2 Odometry 和 IMU 话题。

协议帧格式:
  [0xAA] [D_ADDR] [ID] [LEN] [DATA...] [SC] [AC]
    1B      1B      1B    1B    n B       1B   1B

校验:
  SC  = sum(HEAD .. DATA_end) & 0xFF
  AC  = cumulative_sum(SC_during_sum) & 0xFF
"""

import struct       # 解析二进制帧（将 bytes 解包为 int16/int32 等）
import time

import rclpy              # ROS2 Python 客户端库
from rclpy.node import Node  # 所有 ROS2 Python 节点的基类
# QoS（服务质量）相关的枚举类型
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry              # 里程计消息（/odom 的类型）
from sensor_msgs.msg import Imu                # IMU 消息（/imu 的类型）
# 坐标变换和几何消息
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, Twist, TwistWithCovariance, PoseWithCovariance, Pose, Point
from tf2_ros import TransformBroadcaster       # 动态 TF 广播器（发布 odom→base_link 变换）
import serial           # pyserial：Python 串口通信库


# ── 协议常量 ──────────────────────────────────────────────
FRAME_HEAD = 0xAA
BROADCAST_ADDR = 0xFF
HOST_ADDR = 0xAF

# 飞控输出帧 ID（我们关心的）
ID_IMU_RAW     = 0x01  # 惯性传感器: ACC[3] GYR[3] SHOCK_STA
ID_MAG_BARO    = 0x02  # 罗盘/气压/温度
ID_EULER       = 0x03  # 姿态: 欧拉角 ROL PIT YAW
ID_QUAT        = 0x04  # 姿态: 四元数 V0 V1 V2 V3
ID_ALTITUDE    = 0x05  # 高度: ALT_FU ALT_ADD
ID_SPEED       = 0x07  # 速度: SPEED_X Y Z (cm/s)
ID_POSITION    = 0x08  # 位置: POS_X POS_Y (cm)
ID_MODULE_STA  = 0x0E  # 外接模块状态


class AnoBridgeNode(Node):
    """匿名凌霄飞控 → ROS2 桥接节点"""

    def __init__(self):
        super().__init__('ano_bridge_node')

        # ── 参数 ──────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('acc_scale', 0.004788)    # m/s² per LSB (±16g)
        self.declare_parameter('gyr_scale', 0.001065)    # rad/s per LSB (±2000dps)
        self.declare_parameter('gyr_offset_x', 0.0)
        self.declare_parameter('gyr_offset_y', 0.0)
        self.declare_parameter('gyr_offset_z', 0.0)
        self.declare_parameter('pub_rate', 20.0)         # Hz, 里程计发布频率

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

        # ── QoS（服务质量）配置 ──────────────────────────
        # 为什么 Best Effort？里程计数据每秒 20 帧，丢一帧马上有新的。
        # 如果用 Reliable（保证送达），下游处理慢 → 队列堆积 → 延迟越来越大。
        # Best Effort 保证下游拿到的永远是最新的数据。
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 尽力而为，丢了不重传
            durability=DurabilityPolicy.VOLATILE,        # 不持久化，新订阅者收不到旧数据
            history=HistoryPolicy.KEEP_LAST,             # 只保留最近的 N 条消息
            depth=10,                                    # N=10，队列最多缓存 10 条
        )

        # ── 发布者（Publisher）─ 向外发布话题 ──────────
        # /odom 话题：发布机器人的里程计（位置+速度+姿态），20Hz
        # Odometry = nav_msgs.msg.Odometry，是 ROS2 标准消息类型
        self.odom_pub = self.create_publisher(Odometry, '/odom', sensor_qos)
        # /imu 话题：发布 IMU 数据（加速度+角速度+姿态），有 IMU 数据时即发
        self.imu_pub = self.create_publisher(Imu, '/imu', sensor_qos)

        # ── TF 广播器 ─ 动态发布 odom → base_link 的坐标变换 ──
        # 这是 TF 树中关键的一段：里程计坐标系 → 机器人本体坐标系
        # 机器人一直在移动，所以这段变换是动态的（20Hz 更新）
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── 数据缓存 ──────────────────────────────────
        self.pos_x = 0.0          # m
        self.pos_y = 0.0          # m
        self.pos_z = 0.0          # m
        self.vel_x = 0.0          # m/s
        self.vel_y = 0.0          # m/s
        self.vel_z = 0.0          # m/s
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0  # w,x,y,z
        self.gyr = [0.0, 0.0, 0.0]  # rad/s
        self.acc = [0.0, 0.0, 0.0]  # m/s²
        self.last_odom_time = self.get_clock().now()

        # ── 串口 ──────────────────────────────────────
        self.get_logger().info(f'打开串口 {port}，波特率 {baud}')
        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
        except serial.SerialException as e:
            self.get_logger().fatal(f'无法打开串口: {e}')
            raise

        self.buf = bytearray()

        # ── 定时器：定期发布里程计（pub_rate = 20Hz） ──
        # 不是每收到一帧飞控数据就发一次 /odom，而是固定 20Hz 发布
        # 这样下游 SLAM/AMCL 收到的里程计频率稳定、可预测
        pub_period = 1.0 / self.get_parameter('pub_rate').value  # 1/20 = 0.05 秒
        self.timer = self.create_timer(pub_period, self.publish_odometry)

        # ── 串口读取定时器：以 1kHz 频率轮询串口 ──────────
        # 飞控以 921600bps 高速发送数据，必须高频轮询才能及时收全
        self.read_timer = self.create_timer(0.001, self.read_serial)  # 每 1ms 检查一次串口缓冲区

        self.get_logger().info('匿名凌霄飞控桥接节点已启动')

    # ── 串口读取 ──────────────────────────────────────

    def read_serial(self):
        """非阻塞读取串口字节，送入缓冲区解析"""
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                self.buf.extend(data)
                self.parse_buffer()
        except serial.SerialException as e:
            self.get_logger().error(f'串口读取错误: {e}')

    def parse_buffer(self):
        """从缓冲区中提取并解析完整帧"""
        while len(self.buf) >= 6:  # 最小帧: HEAD + D_ADDR + ID + LEN(0) + SC + AC
            # 寻找帧头
            head_idx = self.buf.find(FRAME_HEAD)
            if head_idx < 0:
                self.buf.clear()
                return
            if head_idx > 0:
                del self.buf[:head_idx]

            if len(self.buf) < 6:
                return

            d_addr = self.buf[1]
            frame_id = self.buf[2]
            data_len = self.buf[3]

            frame_total = 4 + data_len + 2  # HEAD D_ADDR ID LEN + DATA + SC AC
            if len(self.buf) < frame_total:
                return  # 数据不完整，等下一次

            # 提取完整帧
            frame = self.buf[:frame_total]

            # 校验
            if self.verify_checksum(frame):
                self.dispatch_frame(d_addr, frame_id, frame[4:4 + data_len])

            # 移除已处理帧
            del self.buf[:frame_total]

    def verify_checksum(self, frame):
        """验证匿名协议 V7 双重校验"""
        sc = 0
        ac = 0
        data_end = 4 + frame[3]  # 包含 HEAD 到 DATA 结束
        for i in range(data_end):
            sc = (sc + frame[i]) & 0xFF
            ac = (ac + sc) & 0xFF
        return sc == frame[-2] and ac == frame[-1]

    # ── 帧分发 ────────────────────────────────────────

    def dispatch_frame(self, d_addr, frame_id, data):
        """根据帧 ID 分发到对应解析函数"""
        if frame_id == ID_IMU_RAW:
            self.parse_imu_raw(data)
        elif frame_id == ID_MAG_BARO:
            self.parse_mag_baro(data)
        elif frame_id == ID_EULER:
            self.parse_euler(data)
        elif frame_id == ID_QUAT:
            self.parse_quat(data)
        elif frame_id == ID_ALTITUDE:
            self.parse_altitude(data)
        elif frame_id == ID_SPEED:
            self.parse_speed(data)
        elif frame_id == ID_POSITION:
            self.parse_position(data)
        elif frame_id == ID_MODULE_STA:
            self.parse_module_sta(data)

    # ── 各帧解析 ──────────────────────────────────────

    def parse_imu_raw(self, data):
        """ID 0x01: ACC_X ACC_Y ACC_Z (int16×3) + GYR_X GYR_Y GYR_Z (int16×3) + SHOCK_STA (uint8)"""
        if len(data) < 13:
            return
        acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, _ = struct.unpack('<hhhhhhB', data[:13])
        self.acc = [
            acc_x * self.acc_scale,
            acc_y * self.acc_scale,
            acc_z * self.acc_scale,
        ]
        self.gyr = [
            gyr_x * self.gyr_scale + self.gyr_offset[0],
            gyr_y * self.gyr_scale + self.gyr_offset[1],
            gyr_z * self.gyr_scale + self.gyr_offset[2],
        ]
        self.publish_imu()

    def parse_mag_baro(self, data):
        """ID 0x02: MAG_X MAG_Y MAG_Z (int16×3) + ALT_BAR (int32 cm) + TMP (int16 ×0.1°C) + BAR_STA MAG_STA (uint8×2)"""
        if len(data) < 14:
            return
        mag_x, mag_y, mag_z, alt_bar, tmp, bar_sta, mag_sta = struct.unpack('<hhhihBB', data[:14])
        self.alt_baro = alt_bar * 0.01  # cm → m

    def parse_euler(self, data):
        """ID 0x03: ROL×100 PIT×100 YAW×100 (int16×3) + FUSION_STA (uint8)"""
        if len(data) < 7:
            return
        rol, pit, yaw, fusion_sta = struct.unpack('<hhhB', data[:7])
        self.roll = rol * 0.01    # °
        self.pitch = pit * 0.01   # °
        self.yaw = yaw * 0.01     # °
        self.fusion_sta = fusion_sta

    def parse_quat(self, data):
        """ID 0x04: V0 V1 V2 V3×10000 (int16×4) + FUSION_STA (uint8)"""
        if len(data) < 9:
            return
        v0, v1, v2, v3, fusion_sta = struct.unpack('<hhhhB', data[:9])
        # 协议中 V0~V3 是四元数 (w, x, y, z)，传输时 ×10000
        self.q0 = v0 / 10000.0
        self.q1 = v1 / 10000.0
        self.q2 = v2 / 10000.0
        self.q3 = v3 / 10000.0
        self.fusion_sta = fusion_sta

    def parse_altitude(self, data):
        """ID 0x05: ALT_FU (int32 cm) + ALT_ADD (int32 cm) + ALT_STA (uint8)"""
        if len(data) < 9:
            return
        alt_fu, alt_add, alt_sta = struct.unpack('<iiB', data[:9])
        self.pos_z = alt_fu * 0.01       # cm → m
        self.alt_laser = alt_add * 0.01  # 激光测距高度

    def parse_speed(self, data):
        """ID 0x07: SPEED_X SPEED_Y SPEED_Z (int16×3 cm/s)"""
        if len(data) < 6:
            return
        vx, vy, vz = struct.unpack('<hhh', data[:6])
        self.vel_x = vx * 0.01  # cm/s → m/s
        self.vel_y = vy * 0.01
        self.vel_z = vz * 0.01

    def parse_position(self, data):
        """ID 0x08: POS_X POS_Y (int32×2 cm)"""
        if len(data) < 8:
            return
        px, py = struct.unpack('<ii', data[:8])
        self.pos_x = px * 0.01  # cm → m
        self.pos_y = py * 0.01

    def parse_module_sta(self, data):
        """ID 0x0E: STA_G_VEL STA_G_POS STA_GPS STA_ALT_ADD (uint8×4)"""
        if len(data) < 4:
            return
        self.sta_vel = data[0]
        self.sta_pos = data[1]
        self.sta_gps = data[2]
        self.sta_alt = data[3]

    # ── 消息发布 ──────────────────────────────────────

    def publish_odometry(self):
        """发布 nav_msgs/Odometry 消息"""
        now = self.get_clock().now()

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.frame_id

        # 位置
        msg.pose.pose.position.x = self.pos_x
        msg.pose.pose.position.y = self.pos_y
        msg.pose.pose.position.z = self.pos_z

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

        # 协方差（合理默认值）
        # pose: 位置 0.01m², 姿态 0.001rad²
        cov_pose = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.001]
        # twist: 线速度 0.01(m/s)², 角速度 0.01(rad/s)²
        cov_twist = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        msg.pose.covariance = cov_pose
        msg.twist.covariance = cov_twist

        self.odom_pub.publish(msg)

        # 发布 TF
        if self.publish_tf:
            self.publish_odom_tf(now)

    def publish_odom_tf(self, now):
        """发布 odom → base_link 动态 TF"""
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.frame_id
        t.transform.translation.x = self.pos_x
        t.transform.translation.y = self.pos_y
        t.transform.translation.z = self.pos_z
        t.transform.rotation.w = self.q0
        t.transform.rotation.x = self.q1
        t.transform.rotation.y = self.q2
        t.transform.rotation.z = self.q3
        self.tf_broadcaster.sendTransform(t)

    def publish_imu(self):
        """发布 sensor_msgs/Imu 消息"""
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
        # 姿态: 0.001 rad² (融合后)
        msg.orientation_covariance = [0.001, 0.0, 0.0,
                                       0.0, 0.001, 0.0,
                                       0.0, 0.0, 0.001]
        # 角速度: 0.01 (rad/s)² (raw gyro)
        msg.angular_velocity_covariance = [0.01, 0.0, 0.0,
                                            0.0, 0.01, 0.0,
                                            0.0, 0.0, 0.01]
        # 加速度: 0.1 (m/s²)² (raw accel)
        msg.linear_acceleration_covariance = [0.1, 0.0, 0.0,
                                               0.0, 0.1, 0.0,
                                               0.0, 0.0, 0.1]

        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AnoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
