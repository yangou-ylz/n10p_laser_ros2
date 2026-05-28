#!/usr/bin/env python3
"""
混合里程计节点 — 飞控无位置时也能用
位置: 全零，由 SLAM scan matching 自行估计
姿态: 从飞控串口读 0x04 四元数帧，保证激光平面不歪
"""

import struct
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# ── 协议常量 ──────────────────────────────────────────────
FRAME_HEAD = 0xAA
ID_QUAT = 0x04  # 四元数 V0 V1 V2 V3 ×10000 (int16×4)


class DummyOdomNode(Node):
    """位置用零里程计 + 飞控四元数姿态"""

    def __init__(self):
        super().__init__('dummy_odom_node')

        self.declare_parameter('serial_port', '/dev/serial/by-id/usb-ANO_TC_ANO_RadioLink-if00')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value

        # ── 姿态缓存（初始为单位四元数） ──────────────────
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0  # w,x,y,z

        # ── 发布者 ────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── 串口（可选） ──────────────────────────────────
        self.ser = None
        self.buf = bytearray()
        try:
            import serial
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info(f'飞控串口已打开: {port}，读取姿态四元数')
        except Exception as e:
            self.get_logger().warn(f'飞控串口不可用 ({e})，姿态使用单位四元数')

        # ── 定时器 ────────────────────────────────────────
        self.timer = self.create_timer(0.05, self.publish)          # 20Hz 发布里程计
        if self.ser:
            self.read_timer = self.create_timer(0.001, self.read_serial)  # 1kHz 读串口

        self.get_logger().info('混合里程计节点已启动 (位置=全零, 姿态=飞控四元数)')

    # ── 串口读取（与 ano_bridge_node 逻辑一致） ────────────

    def read_serial(self):
        try:
            if self.ser.in_waiting > 0:
                self.buf.extend(self.ser.read(self.ser.in_waiting))
                self.parse_buffer()
        except Exception:
            pass

    def parse_buffer(self):
        while len(self.buf) >= 6:
            head_idx = self.buf.find(FRAME_HEAD)
            if head_idx < 0:
                self.buf.clear()
                return
            if head_idx > 0:
                del self.buf[:head_idx]
            if len(self.buf) < 6:
                return

            data_len = self.buf[3]
            frame_total = 4 + data_len + 2
            if len(self.buf) < frame_total:
                return

            frame = self.buf[:frame_total]
            frame_id = self.buf[2]

            # 只处理四元数帧 (0x04)
            if frame_id == ID_QUAT and self.verify_checksum(frame):
                self.parse_quat(frame[4:4 + data_len])

            del self.buf[:frame_total]

    def verify_checksum(self, frame):
        sc = 0
        ac = 0
        data_end = 4 + frame[3]
        for i in range(data_end):
            sc = (sc + frame[i]) & 0xFF
            ac = (ac + sc) & 0xFF
        return sc == frame[-2] and ac == frame[-1]

    def parse_quat(self, data):
        """ID 0x04: V0 V1 V2 V3×10000 (int16×4) + FUSION_STA (uint8)"""
        if len(data) < 9:
            return
        v0, v1, v2, v3, _ = struct.unpack('<hhhhB', data[:9])
        self.q0 = v0 / 10000.0
        self.q1 = v1 / 10000.0
        self.q2 = v2 / 10000.0
        self.q3 = v3 / 10000.0

    # ── 发布 ────────────────────────────────────────────

    def publish(self):
        now = self.get_clock().now()

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.frame_id
        # 位置保持 (0,0,0) — scan matching 自己算
        odom.pose.pose.orientation.w = self.q0
        odom.pose.pose.orientation.x = self.q1
        odom.pose.pose.orientation.y = self.q2
        odom.pose.pose.orientation.z = self.q3
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = self.odom_frame_id
        tf.child_frame_id = self.frame_id
        tf.transform.rotation.w = self.q0
        tf.transform.rotation.x = self.q1
        tf.transform.rotation.y = self.q2
        tf.transform.rotation.z = self.q3
        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = DummyOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
