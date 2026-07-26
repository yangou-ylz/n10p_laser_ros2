#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
n10p_imu_filter.py — IMU+里程计互补滤波器
============================================
订阅: /imu (角速度+加速度+姿态) + /odom (速度)
输出: /odometry/filtered + odom→base_link TF + /ekf_status

算法:
  姿态 — 互补滤波: 高频 IMU 陀螺仪积分 + 低频飞控四元数修正 + 自适应 alpha
  速度 — 互补滤波: 高频 IMU 加速度积分 + 低频飞控速度修正
"""

import math, time as _time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class IMUFilterNode(Node):
    """互补滤波器: IMU角速度+加速度积分 + 飞控修正 → 平滑里程计 TF"""

    def __init__(self):
        super().__init__('n10p_imu_filter')

        # ── 参数 ──────────────────────────────────────
        self.declare_parameter('alpha_orientation', 0.02)
        self.declare_parameter('alpha_velocity', 0.05)
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('publish_tf', True)

        self.alpha_ori = self.get_parameter('alpha_orientation').value
        self.alpha_vel = self.get_parameter('alpha_velocity').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # ── 状态缓存 ───────────────────────────────
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0       # 滤波后姿态
        self.q0_raw, self.q1_raw, self.q2_raw, self.q3_raw = 1.0,0.0,0.0,0.0  # FC 原始姿态
        self.gyr = [0.0, 0.0, 0.0]              # 最新角速度 (rad/s)
        self.acc = [0.0, 0.0, 0.0]              # 最新线加速度 (m/s²)
        self.vel_fc = [0.0, 0.0, 0.0]           # FC 速度
        self.vel_filt = [0.0, 0.0, 0.0]         # 滤波后速度
        self.pos_x, self.pos_y, self.pos_z = 0.0, 0.0, 0.0  # 积分位置 (供 AMCL 平移先验)
        self.last_imu_ts = None                  # 上一帧 IMU 时间
        self.last_process_ts = 0.0               # 上次处理 IMU 的时间
        self.process_min_dt = 0.01               # 最小处理间隔 (100Hz上限)
        self.imu_timeout = 3.0                   # IMU 超时 (秒)
        self.status = 'initializing'             # EKF 状态
        self.status_updated = False

        # ── Slew Rate Limiter 状态 ─────────────────
        self.last_fc_vx = None                   # 上一帧 FC vx (经 slew 钳位后)
        self.last_fc_vy = None                   # 上一帧 FC vy
        self.MAX_SLEW = 3.0                       # 最大速度变化率 m/s² (只拦>0.3g极端异常)
        self.INNOVATION_THRESH = 1.0              # FC跳变>1m/s → 降权

        # ── QoS ────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        # ── 发布者 ─────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', sensor_qos)
        self.status_pub = self.create_publisher(String, '/ekf_status', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── 订阅者 ─────────────────────────────────
        self.imu_sub = self.create_subscription(Imu, '/imu', self._on_imu, sensor_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._on_odom, sensor_qos)

        # ── 定时器 ─────────────────────────────────
        self._pub_timer = self.create_timer(1.0 / self.publish_rate, self._publish)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'IMU互补滤波器已启动 (alpha_ori={self.alpha_ori}, alpha_vel={self.alpha_vel}, rate={self.publish_rate}Hz)')

    # ══════════════════════════════════════════════════
    # 3.2 自适应 alpha: 旋转越快越信 IMU 陀螺仪, 静止时更信飞控
    # ══════════════════════════════════════════════════
    def _adaptive_alpha(self) -> float:
        gyr_mag = math.sqrt(self.gyr[0]**2 + self.gyr[1]**2 + self.gyr[2]**2)
        # 静止/慢转 (<0.1 rad/s≈6°/s): alpha=0.10 (加倍信任FC绝对四元数, 抑制陀螺零偏)
        # 快速旋转 (>2.0 rad/s≈115°/s): alpha=0.005 (更信陀螺仪)
        if gyr_mag < 0.1:
            return 0.10
        elif gyr_mag > 2.0:
            return 0.005
        else:
            return 0.10 - 0.095 * (gyr_mag - 0.1) / 1.9

    # ══════════════════════════════════════════════════
    # IMU 回调: 姿态互补滤波 + 加速度积分
    # ══════════════════════════════════════════════════
    def _on_imu(self, msg: Imu):
        # 速率限制: IMU 帧 500Hz, 只处理 100Hz, 跳过冗余帧
        now_mono = _time.monotonic()
        if now_mono - self.last_process_ts < self.process_min_dt:
            return  # 跳过, 保留最新缓存
        self.last_process_ts = now_mono

        self.q0_raw = msg.orientation.w
        self.q1_raw = msg.orientation.x
        self.q2_raw = msg.orientation.y
        self.q3_raw = msg.orientation.z

        self.gyr = [msg.angular_velocity.x,
                     msg.angular_velocity.y,
                     msg.angular_velocity.z]

        self.acc = [msg.linear_acceleration.x,
                     msg.linear_acceleration.y,
                     msg.linear_acceleration.z]

        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_imu_ts is not None:
            dt = now_sec - self.last_imu_ts
            if 0.0 < dt < 0.1:
                # ── 姿态互补滤波 ──────────────────────
                gx, gy, gz = self.gyr
                half_dt = dt * 0.5
                dq_w, dq_x, dq_y, dq_z = 1.0, gx*half_dt, gy*half_dt, gz*half_dt
                dq_norm = math.sqrt(dq_w*dq_w + dq_x*dq_x + dq_y*dq_y + dq_z*dq_z)
                dq_w, dq_x, dq_y, dq_z = dq_w/dq_norm, dq_x/dq_norm, dq_y/dq_norm, dq_z/dq_norm

                pw, px, py, pz = self.q0, self.q1, self.q2, self.q3
                qp_w = pw*dq_w - px*dq_x - py*dq_y - pz*dq_z
                qp_x = pw*dq_x + px*dq_w + py*dq_z - pz*dq_y
                qp_y = pw*dq_y - px*dq_z + py*dq_w + pz*dq_x
                qp_z = pw*dq_z + px*dq_y - py*dq_x + pz*dq_w

                a = self._adaptive_alpha()
                q0_n = (1.0-a)*qp_w + a*self.q0_raw
                q1_n = (1.0-a)*qp_x + a*self.q1_raw
                q2_n = (1.0-a)*qp_y + a*self.q2_raw
                q3_n = (1.0-a)*qp_z + a*self.q3_raw
                norm = math.sqrt(q0_n*q0_n + q1_n*q1_n + q2_n*q2_n + q3_n*q3_n)
                if norm > 1e-9:
                    self.q0,self.q1,self.q2,self.q3 = q0_n/norm, q1_n/norm, q2_n/norm, q3_n/norm

                # ── 3.3 速度互补滤波 ──────────────────
                # 1. 用当前姿态移除加速度计中的重力分量
                #    四元数 q=(w,x,y,z) 代表 body→world 旋转
                #    world 重力 (0,0,G) 在 body 系中的投影:
                G = 9.80
                qw, qx, qy, qz = self.q0, self.q1, self.q2, self.q3
                gx = 2.0 * (qx*qz - qw*qy) * G
                gy = 2.0 * (qw*qx + qy*qz) * G
                gz = (qw*qw - qx*qx - qy*qy + qz*qz) * G
                # 2. 移除重力后的纯运动加速度, 加死区过滤静止噪声
                ax = self.acc[0] - gx
                ay = self.acc[1] - gy
                az = self.acc[2] - gz
                DEAD_ZONE = 0.10   # m/s², 提高阈值消除IMU噪声导致的零偏漂移
                if abs(ax) < DEAD_ZONE: ax = 0.0
                if abs(ay) < DEAD_ZONE: ay = 0.0
                # IMU加速度不用于平移速度 — 倾斜时重力泄漏无法完全消除 (PX4同理)
                # 速度估计仅用FC做指数平滑, IMU只管姿态
                dv_x = 0.0
                dv_y = 0.0
                # 3. 指数平滑: (1-αv)*旧速度 + αv*FC速度
                #    静止时 (FC速度≈0) 用高 αv 快速归零, 防止速度残余持续积分
                #    交叉轴抑制: 单轴主导时清零副轴的FC参考, 消除FC传感器轴间耦合
                fc_vx = self.vel_fc[0]
                fc_vy = self.vel_fc[1]
                X_DOMINANT = 3.0  # 主轴/副轴比值阈值
                if abs(fc_vx) > X_DOMINANT * abs(fc_vy):
                    fc_vy = 0.0       # X主导 → 压制Y轴FC耦合
                elif abs(fc_vy) > X_DOMINANT * abs(fc_vx):
                    fc_vx = 0.0       # Y主导 → 压制X轴FC耦合

                # ── Slew Rate Limiter: 限制FC速度变化率, 消除急停反向过冲 ──
                if self.last_fc_vx is not None and dt > 0:
                    max_delta = self.MAX_SLEW * dt
                    dvx_fc = fc_vx - self.last_fc_vx
                    if abs(dvx_fc) > max_delta:
                        fc_vx = self.last_fc_vx + math.copysign(max_delta, dvx_fc)
                    dvy_fc = fc_vy - self.last_fc_vy
                    if abs(dvy_fc) > max_delta:
                        fc_vy = self.last_fc_vy + math.copysign(max_delta, dvy_fc)
                self.last_fc_vx = fc_vx
                self.last_fc_vy = fc_vy

                # ── 连续b值: FC速度越低b越大, 平滑过渡, 避免硬切换跳变 ──
                fc_speed = math.sqrt(fc_vx**2 + fc_vy**2)
                B_HIGH = 0.9
                B_LOW = 0.30             # 运动时指数平滑系数 (纯粹FC平滑, 无IMU参与)
                SPD_HIGH = 0.10          # >0.10m/s → b=B_LOW (运动态)
                SPD_LOW = 0.02           # <0.02m/s → b=B_HIGH (静止态)
                if fc_speed > SPD_HIGH:
                    b = B_LOW
                elif fc_speed < SPD_LOW:
                    b = B_HIGH
                else:
                    # 线性过渡: 0.02→0.10 对应 b=0.9→0.05
                    ratio = (fc_speed - SPD_LOW) / (SPD_HIGH - SPD_LOW)
                    b = B_HIGH + (B_LOW - B_HIGH) * ratio

                # ── Innovation Gate: FC跳变过大时降权, 信任当前滤波状态 ──
                b_x = b
                b_y = b
                innovation_x = abs(fc_vx - self.vel_filt[0])
                innovation_y = abs(fc_vy - self.vel_filt[1])
                if innovation_x > self.INNOVATION_THRESH:
                    b_x = 0.9  # 大跳变 → 紧锁当前状态
                if innovation_y > self.INNOVATION_THRESH:
                    b_y = 0.9

                self.vel_filt[0] = (1.0-b_x)*self.vel_filt[0] + b_x*fc_vx
                self.vel_filt[1] = (1.0-b_y)*self.vel_filt[1] + b_y*fc_vy
                # vz 写 FC 速度 (飞控0x07帧), 不积分IMU加速度
                self.vel_filt[2] = self.vel_fc[2]

                # 积分XY位置 — 提供高频运动预测, AMCL 负责低频绝对修正
                self.pos_x += self.vel_filt[0] * dt
                self.pos_y += self.vel_filt[1] * dt
                self.pos_z = 0.0   # 2D SLAM 不需要Z

        self.last_imu_ts = now_sec

    # ══════════════════════════════════════════════════
    # /odom 回调: 缓存 FC 速度
    # ══════════════════════════════════════════════════
    def _on_odom(self, msg: Odometry):
        self.vel_fc = [msg.twist.twist.linear.x,
                        msg.twist.twist.linear.y,
                        msg.twist.twist.linear.z]

    # ══════════════════════════════════════════════════
    # 3.1 状态发布 (1Hz)
    # ══════════════════════════════════════════════════
    def _publish_status(self):
        if self.last_imu_ts is None:
            self.status = 'initializing'
        elif _time.monotonic() - self.last_imu_ts > self.imu_timeout:
            self.status = 'no_imu'
        else:
            gyr_mag = math.sqrt(self.gyr[0]**2 + self.gyr[1]**2 + self.gyr[2]**2)
            alpha = self._adaptive_alpha()
            self.status = f'running | alpha={alpha:.4f} | gyr_mag={gyr_mag:.3f} rad/s'

        msg = String()
        msg.data = self.status
        self.status_pub.publish(msg)

        if not self.status_updated:
            self.get_logger().info(f'EKF 状态: {self.status}')
            self.status_updated = (self.status.startswith('running'))

    # ══════════════════════════════════════════════════
    # 定时发布: /odometry/filtered + TF
    # ══════════════════════════════════════════════════
    def _publish(self):
        now = self.get_clock().now()

        # IMU 超时回退
        imu_alive = (self.last_imu_ts is not None and
                     _time.monotonic() - self.last_imu_ts < self.imu_timeout)
        if not imu_alive:
            self.q0,self.q1,self.q2,self.q3 = self.q0_raw,self.q1_raw,self.q2_raw,self.q3_raw
            self.vel_filt = list(self.vel_fc)
            self.pos_x = self.pos_y = self.pos_z = 0.0

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.pos_x
        odom.pose.pose.position.y = self.pos_y
        odom.pose.pose.position.z = self.pos_z
        odom.pose.pose.orientation.w = self.q0
        odom.pose.pose.orientation.x = self.q1
        odom.pose.pose.orientation.y = self.q2
        odom.pose.pose.orientation.z = self.q3
        odom.twist.twist.linear.x = self.vel_filt[0]
        odom.twist.twist.linear.y = self.vel_filt[1]
        odom.twist.twist.linear.z = self.vel_filt[2]
        odom.twist.twist.angular.x = self.gyr[0]
        odom.twist.twist.angular.y = self.gyr[1]
        odom.twist.twist.angular.z = self.gyr[2]
        odom.pose.covariance[0] = 1.0
        odom.pose.covariance[7] = 1.0
        odom.pose.covariance[14] = 1.0
        odom.pose.covariance[21] = 0.001
        odom.pose.covariance[28] = 0.001
        odom.pose.covariance[35] = 0.01
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = now.to_msg()
            tf.header.frame_id = 'odom'
            tf.child_frame_id = 'base_link'
            tf.transform.translation.x = self.pos_x
            tf.transform.translation.y = self.pos_y
            tf.transform.translation.z = self.pos_z
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
        try: node.destroy_node()
        except Exception: pass
        try:
            if rclpy.ok(): rclpy.shutdown()
        except Exception: pass


if __name__ == '__main__':
    main()
