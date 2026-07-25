#!/usr/bin/env python3
"""飞行轴间漂诊断: 抓取 FC 原始速度(死区前) vs 滤波后，定位轴间耦合来源。

用法: python3 /home/ylz/n10p_leishen/scripts/diag_cross_axis.py
要求: 导航已启动, 飞行中进行单轴移动测试
时长: 180秒, 每秒2次采样
"""
import rclpy, math, time, csv
from pathlib import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

LOG_DIR = Path("/home/ylz/n10p_leishen/logs")

rclpy.init(args=[])
node = rclpy.create_node("_diag_cross_axis")

# ── 缓存 ────────────────────────────────────
fc_vel_raw = None       # (vx, vy, vz) FC 原始速度 (死区前)
odom_vel = None         # (vx, vy) /odom 速度 (死区后)
filt_vel = None         # (vx, vy) 滤波后速度
filt_pos = None         # (x, y) 滤波位置
amcl_pose = None        # (x, y, yaw_deg) AMCL 位姿
odom_base_tf = None     # (x, y) odom→base_link TF

best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)

def cb_fc_raw(msg: TwistStamped):
    global fc_vel_raw
    fc_vel_raw = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)

def cb_odom(msg: Odometry):
    global odom_vel
    odom_vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)

def cb_filt(msg: Odometry):
    global filt_vel, filt_pos
    filt_vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)
    filt_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

def cb_amcl(msg: PoseWithCovarianceStamped):
    global amcl_pose
    q = msg.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    amcl_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, math.degrees(yaw))

def cb_tf(msg: TFMessage):
    global odom_base_tf
    for t in msg.transforms:
        if t.header.frame_id == 'odom' and t.child_frame_id == 'base_link':
            odom_base_tf = (t.transform.translation.x, t.transform.translation.y)

node.create_subscription(TwistStamped, '/fc_vel_raw', cb_fc_raw, best_effort)
node.create_subscription(Odometry, '/odom', cb_odom, best_effort)
node.create_subscription(Odometry, '/odometry/filtered', cb_filt, best_effort)
node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', cb_amcl, best_effort)
node.create_subscription(TFMessage, '/tf', cb_tf, 10)

LOG_DIR.mkdir(parents=True, exist_ok=True)
ts = time.strftime("%Y%m%d_%H%M%S")
log_path = LOG_DIR / f"diag_cross_axis_{ts}.csv"

with open(log_path, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['elapsed_s',
                     # FC 原始速度 (死区前)
                     'fc_raw_vx', 'fc_raw_vy',
                     # /odom 速度 (死区后)
                     'odom_vx', 'odom_vy',
                     # 滤波后速度
                     'filt_vx', 'filt_vy',
                     # 滤波位置
                     'filt_px', 'filt_py',
                     # odom→base TF
                     'tf_px', 'tf_py',
                     # AMCL 位姿
                     'amcl_x', 'amcl_y', 'amcl_yaw',
                     # 轴间耦合指标: 前飞时 vy/vx 比值
                     'coupling_ratio'])

    print("=" * 95)
    print("  飞行轴间耦合诊断 — 单轴飞行！")
    print(f"  日志: {log_path}")
    print("=" * 95)
    print(f"{'t':>5s} | {'FC原始(m/s)':>14s} | {'/odom(m/s)':>14s} | {'滤波(m/s)':>14s} | {'耦合比':>7s} | {'AMCL(°)':>7s}")
    print("-" * 95)

    t0 = time.monotonic()
    last_sample = t0

    while time.monotonic() - t0 < 180:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()

        if now - last_sample < 0.5:
            continue

        elapsed = now - t0
        frv = fc_vel_raw if fc_vel_raw else (0, 0, 0)
        odv = odom_vel if odom_vel else (0, 0)
        flv = filt_vel if filt_vel else (0, 0)
        flp = filt_pos if filt_pos else (0, 0)
        tfp = odom_base_tf if odom_base_tf else (0, 0)
        acl = amcl_pose if amcl_pose else (0, 0, 0)

        # 轴间耦合比: 主飞X时 vy/vx 的绝对值比例
        if abs(frv[0]) > 0.05:
            ratio = abs(frv[1]) / abs(frv[0])
        elif abs(frv[1]) > 0.05:
            ratio = abs(frv[0]) / abs(frv[1])
        else:
            ratio = 0.0

        # 耦合告警
        warn = ""
        if ratio > 0.05:
            warn = " ⚠ 轴间耦合!"
        elif ratio > 0.02:
            warn = " ⚡ 轻微耦合"

        print(f"{elapsed:5.1f}s | ({frv[0]:6.3f},{frv[1]:6.3f}) | ({odv[0]:6.3f},{odv[1]:6.3f}) | "
              f"({flv[0]:6.3f},{flv[1]:6.3f}) | {ratio:6.4f}{warn} | {acl[2]:6.1f}")

        writer.writerow([f"{elapsed:.2f}",
                         f"{frv[0]:.4f}", f"{frv[1]:.4f}",
                         f"{odv[0]:.4f}", f"{odv[1]:.4f}",
                         f"{flv[0]:.4f}", f"{flv[1]:.4f}",
                         f"{flp[0]:.4f}", f"{flp[1]:.4f}",
                         f"{tfp[0]:.4f}", f"{tfp[1]:.4f}",
                         f"{acl[0]:.4f}", f"{acl[1]:.4f}", f"{acl[2]:.2f}",
                         f"{ratio:.6f}"])

        last_sample = now

print("-" * 95)
print(f"日志: {log_path}")
node.destroy_node()
rclpy.shutdown()
