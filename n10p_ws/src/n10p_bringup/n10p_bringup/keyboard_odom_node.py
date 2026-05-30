#!/usr/bin/env python3
"""
键盘里程计节点 — 用 WASD 模拟无人机全向运动
纯 Python 标准库实现，不依赖任何外部包。

发布 /odom (Odometry) + odom→base_link TF (20Hz)
运动模型: 全向 (linear.x + linear.y + angular.z 独立控制)
"""

import sys
import tty
import termios
import select
import threading
from math import cos, sin

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class KeyboardOdomNode(Node):
    """键盘 → 全向里程计"""

    def __init__(self):
        super().__init__('keyboard_odom_node')

        self.declare_parameter('max_linear_vel', 0.3)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # ── 运动状态 ──────────────────────────────────
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0   # 键盘线程写, timer 线程读-积分-清零
        self.vy = 0.0
        self.vth = 0.0
        self.running = True

        # ── 发布者 ────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── 20Hz 里程计积分 + 发布 ─────────────────────
        self.timer = self.create_timer(0.05, self.update_odom)

        # ── 键盘读取线程 ──────────────────────────────
        self.key_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self.key_thread.start()

        self._print_help()

    # ══════════════════════════════════════════════════
    # 键盘读取 (独立线程)
    # ══════════════════════════════════════════════════

    def _print_help(self):
        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════════════╗\n'
            '║     N10P 桌面测试 — 键盘里程计              ║\n'
            '╠══════════════════════════════════════════════╣\n'
            '║  W/X  : +x/−x  前进/后退                    ║\n'
            '║  A/D  : +y/−y  左移/右移                    ║\n'
            '║  Q/E  : 左转 / 右转                         ║\n'
            '║  S    : 停止                                ║\n'
            '║  R    : 重置位置到 (0,0,0)                  ║\n'
            '║  Ctrl+C: 退出                               ║\n'
            '╚══════════════════════════════════════════════╝'
        )

    def _keyboard_loop(self):
        """非阻塞读取键盘输入，设置速度向量。"""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while self.running:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    c = sys.stdin.read(1)
                    self._handle_key(c)
        except Exception:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def _handle_key(self, c):
        v = self.max_linear_vel
        w = self.max_angular_vel

        if c == 'w':
            self.vx, self.vy, self.vth = v, 0.0, 0.0
        elif c == 'x':
            self.vx, self.vy, self.vth = -v, 0.0, 0.0
        elif c == 'a':
            self.vx, self.vy, self.vth = 0.0, v, 0.0
        elif c == 'd':
            self.vx, self.vy, self.vth = 0.0, -v, 0.0
        elif c == 'q':
            self.vx, self.vy, self.vth = 0.0, 0.0, w
        elif c == 'e':
            self.vx, self.vy, self.vth = 0.0, 0.0, -w
        elif c == 's':
            self.vx, self.vy, self.vth = 0.0, 0.0, 0.0
        elif c == 'r':
            self.x, self.y, self.yaw = 0.0, 0.0, 0.0
            self.vx, self.vy, self.vth = 0.0, 0.0, 0.0
            self.get_logger().info('位置已重置到原点')
        elif ord(c) == 3:  # Ctrl+C
            self.running = False

    # ══════════════════════════════════════════════════
    # 里程计积分 + 发布 (ROS2 Timer 回调, 20Hz)
    # ══════════════════════════════════════════════════

    def update_odom(self):
        dt = 0.05

        # 体坐标系速度 → 世界坐标系积分
        self.x += (self.vx * cos(self.yaw) - self.vy * sin(self.yaw)) * dt
        self.y += (self.vx * sin(self.yaw) + self.vy * cos(self.yaw)) * dt
        self.yaw += self.vth * dt

        now = self.get_clock().now()

        # ── Odometry 消息 ─────────────────────────
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = cos(self.yaw / 2.0)
        # AMCL 需要非零 twist 才能触发运动更新
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        self.odom_pub.publish(odom)

        # ── TF 广播 ──────────────────────────────
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = sin(self.yaw / 2.0)
        tf.transform.rotation.w = cos(self.yaw / 2.0)
        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = KeyboardOdomNode()
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
