#!/usr/bin/env python3
"""
dummy_odom_node.py — 混合里程计节点（飞控无位置时使用）
========================================================

位置: 全零 (0,0,0)，由 SLAM scan matching 自行估计真实位移。
姿态: 从飞控串口读取 0x04 四元数帧，保证激光平面不倾斜。
TF:   发布 odom → base_link（位置全零 + 飞控姿态）。

适用场景: 手持建图，飞控不在线或不想用飞控位置数据。

使用分层架构:
  ano_transport.py — 传输层（串口管理、后台线程、帧同步）
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from .ano_transport import SerialTransport


class DummyOdomNode(Node):
    """
    混合里程计节点。

    - 位置: 始终为 (0,0,0)。SLAM 的 scan matching 自己估算位移，
            通过 map→odom TF 纠正这里的"零位移"。
    - 姿态: 从飞控串口读取 0x04 四元数帧。保证激光平面不倾斜。
    - 发布: /odom (20Hz) + odom→base_link TF (20Hz)
    """

    def __init__(self):
        super().__init__('dummy_odom_node')

        # ── 参数 ──────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 500000)
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

        # ── 传输层 ────────────────────────────────────────
        self._transport = SerialTransport(port, baud)
        self._transport.register_callback(0x04, self._on_quaternion)

        if self._transport.start():
            self.get_logger().info(f'飞控串口已打开: {port}，读取姿态四元数')
        else:
            self.get_logger().warn(f'飞控串口不可用 ({port})，姿态使用单位四元数')

        # ── 定时器：20Hz 发布里程计 ───────────────────────
        self._pub_timer = self.create_timer(0.05, self._publish)

        self.get_logger().info('混合里程计节点已启动 (位置=全零, 姿态=飞控四元数)')

    # ── 帧回调 ────────────────────────────────────────────

    def _on_quaternion(self, d: dict) -> None:
        """0x04 四元数帧回调 — 只更新姿态缓存"""
        if 'error' in d:
            return
        self.q0 = d['w']
        self.q1 = d['x']
        self.q2 = d['y']
        self.q3 = d['z']

    # ── 发布 ──────────────────────────────────────────────

    def _publish(self) -> None:
        """
        发布 /odom 消息 + odom→base_link TF (20Hz)。

        位置始终为 (0,0,0) — 这是故意设计的。
        SLAM 的 scan matching 会自己算出机器人的真实位移，
        然后通过 map→odom TF 来纠正"零位移"。
        最终地图仍然是正确的。
        """
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

        # 发布 TF
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
        if hasattr(node, '_transport'):
            node._transport.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
