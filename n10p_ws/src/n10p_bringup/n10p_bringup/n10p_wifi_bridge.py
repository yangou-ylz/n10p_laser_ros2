#!/usr/bin/env python3
"""
N10P WiFi 桥接节点 — 通过 TCP 接收 ESP32 转发的 N10P 原始帧，发布 /scan

用法:
  # 独立运行 (CLI 参数)
  python3 n10p_wifi_bridge.py [--host 192.168.0.184] [--port 8888]

  # ROS2 launch (ROS 参数)
  ros2 run n10p_bringup n10p_wifi_bridge_node --ros-args -p host:=192.168.0.184

  # launch 文件 (推荐)
  Node(package='n10p_bringup', executable='n10p_wifi_bridge_node',
       parameters=[{'host': '192.168.0.184', 'port': 8888}])
"""

import sys
import socket
import struct
import time
import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# ======== N10P 帧常量 (与 lslidar_driver 一致) ========
FRAME_SIZE       = 108
FRAME_HEADER0    = 0xA5
FRAME_HEADER1    = 0x5A
POINTS_PER_FRAME = 16
DATA_START       = 7
DEGREE_START     = 5
DEGREE_END       = 105
POINT_LEN        = 6
FRAME_ID         = "laser_frame"
SCAN_TOPIC       = "/scan"
PUBLISH_HZ       = 10
MIN_RANGE        = 0.02
MAX_RANGE        = 12.0


def crc8(data: bytes) -> int:
    """累加和取低8位, 与 lslidar_driver N10_CalCRC8 一致"""
    return sum(data) & 0xFF


class ScanAccumulator:
    """积累多帧点数据, 匹配 lslidar_driver 的角度映射逻辑"""
    def __init__(self):
        self.raw_points = []  # [(angle_deg, range_m, intensity), ...]

    def add_point(self, angle_deg: float, range_m: float, intensity: float):
        """存储原始点 (不做过滤, 匹配驱动行为)"""
        self.raw_points.append((angle_deg, range_m, intensity))

    def build_scan(self, stamp, count_num=None):
        """
        构建 LaserScan, 角度映射与 lslidar_driver 完全一致:
          point_idx = round((360 - degree) * count_num / 360)
          scan_num = 2 * count_num
        """
        points = self.raw_points
        self.raw_points = []

        if not points:
            return None

        # 动态 count_num: 匹配有线驱动的行为
        if count_num is None:
            count_num = len(points)
        count_num = max(count_num, 50)  # 最少 50 点

        scan_num = 2 * count_num
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME_ID
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = 2.0 * math.pi / scan_num
        msg.range_min = float(MIN_RANGE)
        msg.range_max = float(MAX_RANGE)
        msg.ranges = [float('inf')] * scan_num
        msg.intensities = [0.0] * scan_num

        for angle_deg, range_m, intensity in points:
            # 直接映射: N10P 角度 → 扫描索引 (每个物理方向只出现一次, 不镜像)
            a = angle_deg % 360.0
            if a < 0:
                a += 360.0
            point_idx = int(round(a * scan_num / 360.0)) % scan_num

            if range_m <= MIN_RANGE or range_m >= MAX_RANGE:
                continue

            msg.ranges[point_idx] = float(range_m)
            msg.intensities[point_idx] = float(intensity)

        return msg


class N10PWifiBridge(Node):
    def __init__(self):
        super().__init__('n10p_wifi_bridge')

        # ROS2 参数 (优先级: CLI 参数 > ROS 参数 > 默认值)
        self.declare_parameter('host', self._cli_host_or('192.168.0.184'))
        self.declare_parameter('port', self._cli_port_or(8888))

        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value

        self.pub = self.create_publisher(LaserScan, SCAN_TOPIC, 10)
        self.acc = ScanAccumulator()
        self.sock = None
        self.running = True

        self.total_frames = 0
        self.valid_frames = 0
        self.last_report = time.time()

        # 延迟发布: 等 slam-toolbox 就绪 (map→odom TF) 后才开始发 /scan
        # 否则 scan 先于 TF 到达 → MessageFilter 队列爆满 → 死锁
        self._publish_ready = False
        self._delay_timer = self.create_timer(5.0, self._enable_publishing)
        # 固定 count_num: 匹配有线驱动典型值, 保证每帧 scan 大小一致
        self._count_num = 529  # N10P 典型, 对应 scan_num=1058
        # 10Hz 定时发布 (但 publish_scan 会检查 _publish_ready)
        self.timer = self.create_timer(0.1, self.publish_scan)

        # TCP 接收线程
        self.tcp_thread = threading.Thread(target=self._spin_tcp, daemon=True)
        self.tcp_thread.start()

        self.get_logger().info(f"WiFi桥接节点已启动, 目标 {self.host}:{self.port}, 5秒后开始发布 /scan")

    # ── CLI 参数回退 (兼容独立运行) ──────────────
    @staticmethod
    def _cli_host_or(default):
        for i, a in enumerate(sys.argv):
            if a == '--host' and i + 1 < len(sys.argv):
                return sys.argv[i + 1]
        return default

    @staticmethod
    def _cli_port_or(default):
        for i, a in enumerate(sys.argv):
            if a == '--port' and i + 1 < len(sys.argv):
                return int(sys.argv[i + 1])
        return default

    def _enable_publishing(self):
        """延迟回调 (仅执行一次): SLAM 就绪后开始发布 /scan"""
        self._delay_timer.cancel()
        self.acc.raw_points.clear()  # 丢弃延迟期间积累的旧数据
        self._publish_ready = True
        self.get_logger().info('/scan 发布已启用 (延迟结束)')

    def publish_scan(self):
        if not self._publish_ready:
            return
        msg = self.acc.build_scan(self.get_clock().now().to_msg(), count_num=self._count_num)
        if msg is None:
            return
        n_valid = sum(1 for r in msg.ranges if r < float('inf'))
        if n_valid > 10:
            self.pub.publish(msg)

    def _spin_tcp(self):
        while self.running:
            try:
                self._connect()
                self._run()
            except Exception as e:
                self.get_logger().error(f"TCP错误: {e}, 3秒后重连...")
                time.sleep(3)

    def _connect(self):
        while self.running:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(5.0)
                self.sock.connect((self.host, self.port))
                self.sock.settimeout(1.0)
                self.get_logger().info(f"已连接 ESP32 {self.host}:{self.port}")
                return
            except (ConnectionRefusedError, socket.timeout, OSError) as e:
                self.get_logger().warn(f"连接失败 ({e}), 3秒后重试...")
                time.sleep(3)

    def _parse_frame(self, frame: bytes):
        if len(frame) != FRAME_SIZE:
            return False
        if crc8(frame[:FRAME_SIZE - 1]) != frame[FRAME_SIZE - 1]:
            return False

        start_angle = struct.unpack('>H', frame[DEGREE_START:DEGREE_START + 2])[0] * 0.01
        end_angle = struct.unpack('>H', frame[DEGREE_END:DEGREE_END + 2])[0] * 0.01
        if end_angle < start_angle:
            end_angle += 360.0
        angle_step = (end_angle - start_angle) / (POINTS_PER_FRAME - 1)

        for i in range(POINTS_PER_FRAME):
            offset = DATA_START + i * POINT_LEN
            if offset + 5 >= FRAME_SIZE:
                break
            dist_raw = struct.unpack('<H', frame[offset:offset + 2])[0]
            conf_raw = struct.unpack('<H', frame[offset + 2:offset + 4])[0]
            # 匹配有线驱动: dist_raw==0 或 0xFFFF 的点也保留(角度信息有价值),
            # range_m=0 会在 build_scan 中被转换为 inf
            if dist_raw == 0xFFFF:
                continue
            range_m = dist_raw / 1000.0
            self.acc.add_point(start_angle + angle_step * i, range_m, float(conf_raw))
        return True

    def _run(self):
        buf = b''
        frame = bytearray()
        state = 0  # 0=WAIT_H0, 1=WAIT_H1, 2=COLLECT

        while self.running:
            try:
                data = self.sock.recv(8192)
                if not data:
                    raise ConnectionError("TCP断开")
            except (socket.timeout, BlockingIOError):
                data = b''
            except Exception as e:
                raise

            buf += data
            # 首次收到数据时打印 hex dump, 诊断帧格式
            if data and not hasattr(self, '_dumped'):
                self._dumped = True
                self.get_logger().info(f"首次收到 {len(data)} 字节: {data[:64].hex()}")
            while len(buf) > 0:
                b = buf[0]
                buf = buf[1:]

                if state == 0:
                    if b == FRAME_HEADER0:
                        frame = bytearray([b])
                        state = 1
                elif state == 1:
                    if b == FRAME_HEADER1:
                        frame.append(b)
                        state = 2
                    elif b == FRAME_HEADER0:
                        frame = bytearray([b])
                    else:
                        state = 0
                elif state == 2:
                    frame.append(b)
                    if len(frame) >= FRAME_SIZE:
                        self.total_frames += 1
                        if self._parse_frame(bytes(frame)):
                            self.valid_frames += 1
                        state = 0

            # 统计 (5秒间隔)
            now = time.time()
            if now - self.last_report > 5.0:
                fps = self.valid_frames / (now - self.last_report)
                self.get_logger().info(
                    f"收到={len(data)}B 帧: valid={self.valid_frames} 总={self.total_frames} 缓点={len(self.acc.raw_points)}",
                    throttle_duration_sec=5.0)
                self.total_frames = 0
                self.valid_frames = 0
                self.last_report = now

        if self.sock:
            self.sock.close()


def main():
    rclpy.init(args=sys.argv)
    node = N10PWifiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
