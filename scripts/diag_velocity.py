#!/usr/bin/env python3
"""速断: 检查 FC 速度是否到达 imu_filter_node, 是否被正确积分成位置。

用法: python3 /home/ylz/n10p_leishen/scripts/diag_velocity.py
要求: 导航已启动, 飞行中运行
"""
import rclpy, math, time, csv
from pathlib import Path
from collections import defaultdict
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

LOG_DIR = Path("/home/ylz/n10p_leishen/logs")

rclpy.init(args=[])
node = rclpy.create_node("_diag_vel")

# 存储
odom_vel = None          # /odom 速度
odom_filt_vel = None     # /odometry/filtered 速度
odom_filt_pos = None     # /odometry/filtered 位置
odom_tf_pos = None       # TF odom→base 位置
fc_vel_raw = []          # FC 速度历史

best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)

def cb_odom(msg):
    global odom_vel
    odom_vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)

def cb_filt(msg):
    global odom_filt_vel, odom_filt_pos
    odom_filt_vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)
    odom_filt_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

def cb_tf(msg):
    global odom_tf_pos
    for t in msg.transforms:
        if t.header.frame_id == 'odom' and t.child_frame_id == 'base_link':
            odom_tf_pos = (t.transform.translation.x, t.transform.translation.y)

# 用 best_effort 订阅 /odom (避免 QoS 不匹配)
node.create_subscription(Odometry, '/odom', cb_odom, best_effort)
node.create_subscription(Odometry, '/odometry/filtered', cb_filt, 10)
node.create_subscription(TFMessage, '/tf', cb_tf, 10)

LOG_DIR.mkdir(parents=True, exist_ok=True)
ts = time.strftime("%Y%m%d_%H%M%S")
log_path = LOG_DIR / f"diag_velocity_{ts}.csv"

with open(log_path, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['elapsed_s', 'odom_vel_x', 'odom_vel_y',
                     'filt_vel_x', 'filt_vel_y', 'filt_pos_x', 'filt_pos_y',
                     'tf_pos_x', 'tf_pos_y'])

    print("=" * 78)
    print("  速度/位置数据流诊断 — 移动无人机！")
    print(f"  日志: {log_path}")
    print("=" * 78)
    print(f"{'t(s)':>5s} | {'/odom速度':>12s} | {'滤波速度':>12s} | {'滤波位置':>12s} | {'TF位置':>12s}")
    print("-" * 78)

    t0 = time.monotonic()
    last_print = t0

    while time.monotonic() - t0 < 120:
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.monotonic()
        elapsed = now - t0

        if now - last_print < 1.0:
            continue

        ov_x = odom_vel[0] if odom_vel else 0
        ov_y = odom_vel[1] if odom_vel else 0
        fv_x = odom_filt_vel[0] if odom_filt_vel else 0
        fv_y = odom_filt_vel[1] if odom_filt_vel else 0
        fp_x = odom_filt_pos[0] if odom_filt_pos else 0
        fp_y = odom_filt_pos[1] if odom_filt_pos else 0
        tp_x = odom_tf_pos[0] if odom_tf_pos else 0
        tp_y = odom_tf_pos[1] if odom_tf_pos else 0

        print(f"{elapsed:5.1f}s | ({ov_x:5.2f},{ov_y:5.2f})m/s | ({fv_x:5.2f},{fv_y:5.2f})m/s | "
              f"({fp_x:5.2f},{fp_y:5.2f})m | ({tp_x:5.2f},{tp_y:5.2f})m")

        writer.writerow([f"{elapsed:.2f}", f"{ov_x:.3f}", f"{ov_y:.3f}",
                         f"{fv_x:.3f}", f"{fv_y:.3f}", f"{fp_x:.3f}", f"{fp_y:.3f}",
                         f"{tp_x:.3f}", f"{tp_y:.3f}"])
        last_print = now

print("-" * 78)
print(f"日志: {log_path}")
node.destroy_node()
rclpy.shutdown()
