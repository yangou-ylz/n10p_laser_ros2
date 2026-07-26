#!/usr/bin/env python3
"""一键诊断: 启动后你只管飞(后→前→左→右), 脚本自动抓数据。
用法: python3 /home/ylz/n10p_leishen/scripts/diag_direction.py
"""
import rclpy, math, time, csv
from pathlib import Path
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

LOG_DIR = Path("/home/ylz/n10p_leishen/logs")

rclpy.init(args=[])
node = rclpy.create_node("_diag_dir")

fc_raw = None      # /fc_vel_raw: FC原始数据, 死区前
odom_vel = None    # /odom: ano_bridge 处理后 (应用了YAML符号)
filt_vel = None    # /odometry/filtered: imu_filter 处理后
odom_base_tf = None

best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)

def cb_fc(msg):
    global fc_raw; fc_raw = (msg.twist.linear.x, msg.twist.linear.y)

def cb_odom(msg):
    global odom_vel; odom_vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)

def cb_filt(msg):
    global filt_vel; filt_vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)

def cb_tf(msg):
    global odom_base_tf
    for t in msg.transforms:
        if t.header.frame_id == 'odom' and t.child_frame_id == 'base_link':
            odom_base_tf = (t.transform.translation.x, t.transform.translation.y)

node.create_subscription(TwistStamped, '/fc_vel_raw', cb_fc, best_effort)
node.create_subscription(Odometry, '/odom', cb_odom, best_effort)
node.create_subscription(Odometry, '/odometry/filtered', cb_filt, best_effort)
node.create_subscription(TFMessage, '/tf', cb_tf, 10)

LOG_DIR.mkdir(parents=True, exist_ok=True)
ts = time.strftime("%Y%m%d_%H%M%S")
log_path = LOG_DIR / f"diag_direction_{ts}.csv"

with open(log_path, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['elapsed_s', 'fc_vx', 'fc_vy', 'odom_vx', 'odom_vy',
                     'filt_vx', 'filt_vy', 'tf_px', 'tf_py',
                     'dir_flag'])

    print("=" * 90)
    print("  方向诊断 — 你只需飞: 后→前→左→右 (每方向~3秒, 间停2秒)")
    print(f"  日志: {log_path}")
    print("=" * 90)
    print(f"{'t':>5s} | {'FC原始':>14s} | {'/odom':>14s} | {'滤波后':>14s} | 方向判断")
    print("-" * 90)

    t0 = time.monotonic()
    last_sample = t0

    while time.monotonic() - t0 < 30:
        rclpy.spin_once(node, timeout_sec=0.01)
        now = time.monotonic()
        if now - last_sample < 0.2:  # 5Hz
            continue

        elapsed = now - t0
        fv = fc_raw if fc_raw else (0.0, 0.0)
        ov = odom_vel if odom_vel else (0.0, 0.0)
        flv = filt_vel if filt_vel else (0.0, 0.0)
        tp = odom_base_tf if odom_base_tf else (0.0, 0.0)

        # 方向判断: FC原始 和 /odom 是否同号?
        dir_flag = ""
        if abs(fv[0]) > 0.03:
            if fv[0] * ov[0] < 0:
                dir_flag += " VX反!"
            if fv[0] * flv[0] < 0:
                dir_flag += " FVX反!"
        if abs(fv[1]) > 0.03:
            if fv[1] * ov[1] < 0:
                dir_flag += " VY反!"
            if fv[1] * flv[1] < 0:
                dir_flag += " FVY反!"

        marker = " ←←← 方向反了!" if dir_flag else ""

        print(f"{elapsed:5.1f}s | ({fv[0]:+6.3f},{fv[1]:+6.3f}) | ({ov[0]:+6.3f},{ov[1]:+6.3f}) | "
              f"({flv[0]:+6.3f},{flv[1]:+6.3f}) |{dir_flag}{marker}")

        writer.writerow([f"{elapsed:.2f}",
                         f"{fv[0]:.4f}", f"{fv[1]:.4f}",
                         f"{ov[0]:.4f}", f"{ov[1]:.4f}",
                         f"{flv[0]:.4f}", f"{flv[1]:.4f}",
                         f"{tp[0]:.4f}", f"{tp[1]:.4f}",
                         f"{1 if dir_flag else 0}"])

        last_sample = now

print("-" * 90)
print(f"日志: {log_path}")
print("如果看到 'VX反!' 或 'VY反!' → FC原始和/odom符号不一致 → ano_bridge YAML问题")
print("如果看到 'FVX反!' 或 'FVY反!' → /odom和滤波后符号不一致 → imu_filter问题")
print("如果都没反但RViz方向错 → TF或AMCL问题")
node.destroy_node()
rclpy.shutdown()
