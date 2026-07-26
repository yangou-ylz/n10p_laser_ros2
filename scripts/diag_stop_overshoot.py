#!/usr/bin/env python3
"""急停过冲诊断: 10Hz采样捕捉速度归零时的反向瞬态。

用法: python3 /home/ylz/n10p_leishen/scripts/diag_stop_overshoot.py
要求: 导航已启动, 飞行中进行急停测试
时长: 90秒, 每秒10次采样 (聚焦stop transient)
"""
import rclpy, math, time, csv
from pathlib import Path
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

LOG_DIR = Path("/home/ylz/n10p_leishen/logs")

rclpy.init(args=[])
node = rclpy.create_node("_diag_stop_overshoot")

fc_raw = None    # (vx, vy) FC原始
filt_vel = None  # (vx, vy) 滤波后

best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)

def cb_fc(msg: TwistStamped):
    global fc_raw
    fc_raw = (msg.twist.linear.x, msg.twist.linear.y)

def cb_filt(msg: Odometry):
    global filt_vel
    filt_vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)

node.create_subscription(TwistStamped, '/fc_vel_raw', cb_fc, best_effort)
node.create_subscription(Odometry, '/odometry/filtered', cb_filt, best_effort)

LOG_DIR.mkdir(parents=True, exist_ok=True)
ts = time.strftime("%Y%m%d_%H%M%S")
log_path = LOG_DIR / f"diag_stop_overshoot_{ts}.csv"

with open(log_path, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['elapsed_s', 'fc_vx', 'fc_vy', 'filt_vx', 'filt_vy',
                     'dvx', 'dvy', 'overshoot_flag'])

    print("=" * 80)
    print("  急停过冲诊断 (10Hz) — 移动→急停→保持, 重复4方向")
    print(f"  日志: {log_path}")
    print("=" * 80)
    print(f"{'t':>6s} | {'FC原始':>16s} | {'滤波':>16s} | {'Δv':>16s} | 过冲")
    print("-" * 80)

    t0 = time.monotonic()
    last_sample = t0
    prev_fc = None
    SAMPLE_DT = 0.1  # 10Hz

    while time.monotonic() - t0 < 90:
        rclpy.spin_once(node, timeout_sec=0.01)
        now = time.monotonic()

        if now - last_sample < SAMPLE_DT:
            continue

        elapsed = now - t0
        fv = fc_raw if fc_raw else (0, 0)
        flv = filt_vel if filt_vel else (0, 0)

        # 计算速度变化率 (dv/dt, 近似加速度)
        dvx, dvy = 0.0, 0.0
        if prev_fc is not None:
            dt = now - last_sample
            if dt > 0:
                dvx = (fv[0] - prev_fc[0]) / dt
                dvy = (fv[1] - prev_fc[1]) / dt
        prev_fc = fv

        # 过冲检测: 速度反向 (v_now 与 Δv 符号相反 = 急减速)
        overshoot = ""
        if abs(dvx) > 0.3:  # >0.3m/s² 的减速度
            if fv[0] * dvx < 0:  # 速度和加速度反向
                overshoot += " X过冲!"
            if fv[1] * dvy < 0:
                overshoot += " Y过冲!"

        print(f"{elapsed:6.2f}s | ({fv[0]:7.3f},{fv[1]:7.3f}) | ({flv[0]:7.3f},{flv[1]:7.3f}) | "
              f"({dvx:7.3f},{dvy:7.3f}) |{overshoot}")

        writer.writerow([f"{elapsed:.3f}",
                         f"{fv[0]:.4f}", f"{fv[1]:.4f}",
                         f"{flv[0]:.4f}", f"{flv[1]:.4f}",
                         f"{dvx:.4f}", f"{dvy:.4f}",
                         f"{1 if overshoot else 0}"])

        last_sample = now

print("-" * 80)
print(f"日志: {log_path}")
node.destroy_node()
rclpy.shutdown()
