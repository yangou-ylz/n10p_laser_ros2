#!/usr/bin/env python3
"""
N10P WiFi 桥接节点 — 独立 ROS2 节点, 不修改任何现有代码

功能: TCP 连接 ESP32 → 解析 N10P 108字节帧 → 发布 /scan (LaserScan)

用法:
  ros2env
  python3 n10p_wifi_bridge.py [--host 192.168.0.184] [--port 8888]

可与 lslidar_driver 切换使用:
  - 有线模式: ros2 launch lslidar_driver lslidar_launch.py
  - 无线模式: python3 n10p_wifi_bridge.py
"""

import sys, socket, struct, time, math, threading
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# ======== N10P 帧常量 (与 lslidar_driver 一致) ========
FRAME_SIZE       = 108
FRAME_HEADER0    = 0xA5
FRAME_HEADER1    = 0x5A
POINTS_PER_FRAME = 16
DATA_START       = 7                # 数据从字节7开始
DEGREE_START     = 5                # 起始角度在字节5-6 (uint16 BE, 0.01°)
DEGREE_END       = 105              # 结束角度在字节105-106
POINT_LEN        = 6                # 每个点6字节
MAX_POINTS       = 6000             # scan_points 数组大小

# ======== 默认参数 ========
DEFAULT_HOST     = "192.168.0.184"
DEFAULT_PORT     = 8888
FRAME_ID         = "laser_frame"
SCAN_TOPIC       = "/scan"
PUBLISH_HZ       = 10               # 发布频率
MIN_RANGE        = 0.02             # N10P 最小量程 (m)
MAX_RANGE        = 12.0             # N10P 最大量程 (m)


def crc8(data: bytes) -> int:
    """累加和取低8位, 与 lslidar_driver N10_CalCRC8 一致"""
    return sum(data) & 0xFF


class ScanAccumulator:
    """积累多帧点数据, 发布完整 LaserScan"""
    def __init__(self):
        self.points = [None] * MAX_POINTS  # (range_m, intensity) or None
        self.count = 0
        self.last_angle = -1
        self.full_rotation = False

    def add_point(self, angle_deg: float, range_m: float, intensity: float):
        """添加一个点到缓冲区"""
        if range_m <= MIN_RANGE or range_m >= MAX_RANGE:
            return
        # 角度归一化到 0-360, 映射到 points 数组索引
        a = angle_deg % 360.0
        if a < 0:
            a += 360.0
        idx = int(round(a * MAX_POINTS / 360.0)) % MAX_POINTS
        self.points[idx] = (range_m, intensity)
        self.count += 1
        # 检测角度回绕 → 一圈完成
        if self.last_angle > 300 and a < 60:
            self.full_rotation = True
        self.last_angle = a

    def build_scan(self, stamp, count_num=2000):
        """构建 LaserScan 消息 (count_num = 半圈点数, scan_num = 2*count_num)"""
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

        for i in range(MAX_POINTS):
            if i < scan_num and self.points[i] is not None:
                r, intensity = self.points[i]
                msg.ranges[i] = max(MIN_RANGE, min(MAX_RANGE, r))
                msg.intensities[i] = intensity

        self.points = [None] * MAX_POINTS
        self.count = 0
        self.full_rotation = False
        return msg


class N10PWifiBridge(Node):
    def __init__(self, host, port):
        super().__init__('n10p_wifi_bridge')
        self.host = host
        self.port = port
        self.pub = self.create_publisher(LaserScan, SCAN_TOPIC, 10)
        self.acc = ScanAccumulator()
        self.sock = None
        self.running = True
        self.count_num = 2000  # N10P 半圈点数

        # 统计
        self.total_frames = 0
        self.valid_frames = 0
        self.last_report = time.time()

        # 定时发布 /scan (10Hz), 无论缓冲多少点都发
        self.timer = self.create_timer(0.1, self.publish_scan)

    def publish_scan(self):
        """Timer 回调: 10Hz 发布 /scan"""
        msg = self.acc.build_scan(self.get_clock().now().to_msg(), self.count_num)
        n_valid = sum(1 for r in msg.ranges if r < float('inf'))
        if n_valid > 10:  # 至少有10个有效点才发布
            self.pub.publish(msg)
            self.acc.points = [None] * MAX_POINTS
            self.acc.count = 0
            self.acc.full_rotation = False

    def connect(self):
        """连接 ESP32 TCP Server, 带重试"""
        while self.running:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(5.0)
                self.sock.connect((self.host, self.port))
                self.sock.settimeout(1.0)
                self.get_logger().info(f"已连接 ESP32 {self.host}:{self.port}")
                return True
            except (ConnectionRefusedError, socket.timeout, OSError) as e:
                self.get_logger().warn(f"连接失败 ({e}), 3秒后重试...")
                time.sleep(3)

    def parse_frame(self, frame: bytes):
        """解析一个108字节N10P帧, 提取16个点"""
        if len(frame) != FRAME_SIZE:
            return

        # 校验 CRC
        if crc8(frame[:FRAME_SIZE-1]) != frame[FRAME_SIZE-1]:
            return

        # 起始角度 (uint16 BE, 0.01度)
        start_angle = struct.unpack('>H', frame[DEGREE_START:DEGREE_START+2])[0] * 0.01
        end_angle   = struct.unpack('>H', frame[DEGREE_END:DEGREE_END+2])[0] * 0.01

        if end_angle < start_angle:
            end_angle += 360.0

        angle_step = (end_angle - start_angle) / (POINTS_PER_FRAME - 1) if POINTS_PER_FRAME > 1 else 0

        # 提取16个点
        for i in range(POINTS_PER_FRAME):
            offset = DATA_START + i * POINT_LEN
            if offset + 5 >= FRAME_SIZE:
                break
            dist_raw = struct.unpack('<H', frame[offset:offset+2])[0]
            conf_raw = struct.unpack('<H', frame[offset+2:offset+4])[0]

            if dist_raw == 0xFFFF or dist_raw == 0:
                continue

            range_m = dist_raw / 1000.0  # mm → m
            angle_deg = start_angle + angle_step * i
            self.acc.add_point(angle_deg, range_m, float(conf_raw))

        return True

    def run(self):
        """主循环: 读TCP → 同步帧 → 解析 → 发布"""
        buf = b''
        frame = bytearray()
        state = 0  # 0=wait H0, 1=wait H1, 2=collect

        while self.running:
            # 收发数据
            try:
                data = self.sock.recv(8192)
                if not data:
                    raise ConnectionError("TCP断开")
            except (socket.timeout, BlockingIOError):
                data = b''
            except Exception as e:
                self.get_logger().error(f"TCP错误: {e}")
                break

            buf += data
            # 帧同步状态机
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
                        if self.parse_frame(bytes(frame)):
                            self.valid_frames += 1
                        state = 0

            # 定时发布
            now = time.time()
            if now - self.last_report > 1.0:
                fps = self.valid_frames / (now - self.last_report)
                self.get_logger().info(
                    f"帧: total={self.total_frames} valid={self.valid_frames} "
                    f"fps={fps:.0f} 缓点={self.acc.count}",
                    throttle_duration_sec=5.0)
                self.total_frames = 0
                self.valid_frames = 0
                self.last_report = now

        self.get_logger().info("TCP 连接断开")
        if self.sock:
            self.sock.close()

    def spin(self):
        """在独立线程中运行"""
        while self.running:
            try:
                self.connect()
                self.run()
            except Exception as e:
                self.get_logger().error(f"运行错误: {e}")

        if self.sock:
            self.sock.close()


def main():
    rclpy.init(args=sys.argv)

    # 解析命令行参数
    host = DEFAULT_HOST
    port = DEFAULT_PORT
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] == '--host' and i+1 < len(args):
            host = args[i+1]; i += 2
        elif args[i] == '--port' and i+1 < len(args):
            port = int(args[i+1]); i += 2
        else:
            i += 1

    bridge = N10PWifiBridge(host, port)
    thread = threading.Thread(target=bridge.spin, daemon=True)
    thread.start()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.running = False
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
