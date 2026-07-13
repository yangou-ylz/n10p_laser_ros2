#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
n10p_imu_filter.py — 轻量 IMU+里程计互补滤波器
================================================
替代 robot_localization EKF (ARM64 有 NaN bug)。

功能: 订阅 /imu (角速度+加速度+姿态) + /odom (速度+姿态),
      输出更平滑的 odom→base_link TF + /odometry/filtered。

算法: 互补滤波 — 高频用 IMU 陀螺仪积分, 低频用飞控四元数修正。
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class IMUFilterNode(Node):
    """互补滤波器: IMU角速度积分 + 飞控四元数修正 → 平滑 odom→base_link TF"""

    def __init__(self):
        super().__init__('n10p_imu_filter')

        # ── 参数 ──────────────────────────────────────
        self.declare_parameter('alpha_orientation', 0.02)
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('publish_tf', True)

        self.alpha = self.get_parameter('alpha_orientation').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # ── 姿态缓存 ───────────────────────────────
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0
        self.q0_raw, self.q1_raw, self.q2_raw, self.q3_raw = 1.0, 0.0, 0.0, 0.0
        self.gyr = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.last_imu_ts = None
        self.imu_timeout = 3.0       # 3秒无IMU→回退透传模式

        # ── QoS: 匹配 ano_bridge 的 Best Effort ─────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── 发布者 (100Hz → Best Effort, 高频数据丢一帧无妨) ──
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', sensor_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── 订阅者 (QoS 必须匹配发布者!) ──────────────
        self.imu_sub = self.create_subscription(Imu, '/imu', self._on_imu, sensor_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._on_odom, sensor_qos)

        # ── 定时器: 固定频率发布 ─────────────────────
        period = 1.0 / self.publish_rate
        self._pub_timer = self.create_timer(period, self._publish)

        self.get_logger().info(f'IMU互补滤波器已启动 (alpha={self.alpha}, rate={self.publish_rate}Hz)')

    def _on_imu(self, msg: Imu):
        """IMU 回调: 缓存角速度 + 四元数 + 做互补滤波"""
        # 飞控四元数 (绝对姿态, 67Hz, 低频修正用)
        self.q0_raw = msg.orientation.w
        self.q1_raw = msg.orientation.x
        self.q2_raw = msg.orientation.y
        self.q3_raw = msg.orientation.z

        # 陀螺仪角速度 (高频, 用于帧间积分)
        self.gyr = [msg.angular_velocity.x,
                     msg.angular_velocity.y,
                     msg.angular_velocity.z]

        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_imu_ts is not None:
            dt = now_sec - self.last_imu_ts
            if 0.0 < dt < 0.1:  # 10ms 以内的合理间隔
                # ── 步骤1: 用陀螺仪积分姿态 (高频预测) ──
                gx, gy, gz = self.gyr
                # 小角度近似: 四元数增量 = [1, gx*dt/2, gy*dt/2, gz*dt/2]
                half_dt = dt * 0.5
                dq_w = 1.0
                dq_x = gx * half_dt
                dq_y = gy * half_dt
                dq_z = gz * half_dt
                # 归一化增量
                dq_norm = math.sqrt(dq_w*dq_w + dq_x*dq_x + dq_y*dq_y + dq_z*dq_z)
                dq_w /= dq_norm
                dq_x /= dq_norm
                dq_y /= dq_norm
                dq_z /= dq_norm

                # 四元数乘法: q_pred = q_filtered ⊗ dq
                pw, px, py, pz = self.q0, self.q1, self.q2, self.q3
                q_pred_w = pw*dq_w - px*dq_x - py*dq_y - pz*dq_z
                q_pred_x = pw*dq_x + px*dq_w + py*dq_z - pz*dq_y
                q_pred_y = pw*dq_y - px*dq_z + py*dq_w + pz*dq_x
                q_pred_z = pw*dq_z + px*dq_y - py*dq_x + pz*dq_w

                # ── 步骤2: 用飞控四元数修正 (低频, 互补滤波) ──
                # q_filtered = (1-α) * q_pred + α * q_raw   (球面线性插值近似)
                alpha = self.alpha
                q0_new = (1.0 - alpha) * q_pred_w + alpha * self.q0_raw
                q1_new = (1.0 - alpha) * q_pred_x + alpha * self.q1_raw
                q2_new = (1.0 - alpha) * q_pred_y + alpha * self.q2_raw
                q3_new = (1.0 - alpha) * q_pred_z + alpha * self.q3_raw

                # 归一化
                norm = math.sqrt(q0_new*q0_new + q1_new*q1_new + q2_new*q2_new + q3_new*q3_new)
                if norm > 1e-9:
                    self.q0, self.q1, self.q2, self.q3 = q0_new/norm, q1_new/norm, q2_new/norm, q3_new/norm

        self.last_imu_ts = now_sec

    def _on_odom(self, msg: Odometry):
        """/odom 回调: 缓存飞控线速度"""
        self.vel = [msg.twist.twist.linear.x,
                     msg.twist.twist.linear.y,
                     msg.twist.twist.linear.z]

    def _publish(self):
        """定时发布: /odometry/filtered + odom→base_link TF"""
        now = self.get_clock().now()

        # ── IMU 超时检测: 3秒无数据→回退透传模式 ──
        import time as _time
        imu_alive = (self.last_imu_ts is not None and
                     _time.monotonic() - self.last_imu_ts < self.imu_timeout)
        if not imu_alive:
            # 透传: 直接用飞控原始四元数 (来自 /odom 回调中的 /imu orientation)
            self.q0, self.q1, self.q2, self.q3 = self.q0_raw, self.q1_raw, self.q2_raw, self.q3_raw

        # ── Odometry 消息 ──────────────────────────
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        # 位置: 保持为0 (飞控位置不可靠, 由 AMCL 修正)
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        # 姿态: 用滤波后的四元数
        odom.pose.pose.orientation.w = self.q0
        odom.pose.pose.orientation.x = self.q1
        odom.pose.pose.orientation.y = self.q2
        odom.pose.pose.orientation.z = self.q3
        # 速度: 来自飞控
        odom.twist.twist.linear.x = self.vel[0]
        odom.twist.twist.linear.y = self.vel[1]
        odom.twist.twist.linear.z = self.vel[2]
        odom.twist.twist.angular.x = self.gyr[0]
        odom.twist.twist.angular.y = self.gyr[1]
        odom.twist.twist.angular.z = self.gyr[2]
        # 协方差 (位置不信任, 姿态较信任)
        odom.pose.covariance[0] = 1.0   # x
        odom.pose.covariance[7] = 1.0   # y
        odom.pose.covariance[14] = 1.0  # z
        odom.pose.covariance[21] = 0.001  # roll
        odom.pose.covariance[28] = 0.001  # pitch
        odom.pose.covariance[35] = 0.01   # yaw (偏航不确定性略高)
        self.odom_pub.publish(odom)

        # ── TF 广播 ────────────────────────────────
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = now.to_msg()
            tf.header.frame_id = 'odom'
            tf.child_frame_id = 'base_link'
            tf.transform.rotation.w = self.q0
            tf.transform.rotation.x = self.q1
            tf.transform.rotation.y = self.q2
            tf.transform.rotation.z = self.q3
            self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = IMUFilterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # 安全清理: 先销毁节点, 再 shutdown context (只调一次)
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
