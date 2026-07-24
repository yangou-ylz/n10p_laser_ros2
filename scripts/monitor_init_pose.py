#!/usr/bin/env python3
"""
导航初始位姿监控脚本 — 运行 40 秒，记录 AMCL yaw / TF 链变化，检测突变。

用法:
  先启动导航:
    ros2 launch n10p_nav nav_ekf_launch.py map:=/home/ylz/n10p_leishen/maps/n10p_map.yaml

  另开终端运行:
    python3 /home/ylz/n10p_leishen/scripts/monitor_init_pose.py

输出:
  1. 终端实时打印每秒的 yaw 数据
  2. 保存完整日志到 logs/monitor_init_pose_<时间戳>.csv
"""
import rclpy, math, time, csv, os
from pathlib import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

DURATION = 40  # 监控秒数
LOG_DIR = Path("/home/ylz/n10p_leishen/logs")


class InitPoseMonitor:
    def __init__(self):
        self.node = rclpy.create_node("_init_pose_monitor")
        self.amcl_pose = None
        self.map_odom_yaw = None       # map→odom 的 yaw (rad)
        self.odom_base_yaw = None      # odom→base_link 的 yaw (rad)
        self.odom_msg_yaw = None       # /odom 话题直接报的 yaw (rad)
        self.records = []               # [(t, amcl_yaw, map_odom_yaw, odom_base_yaw, tf_chain_yaw, odom_msg_yaw, yaw_sigma)]

        self.node.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._cb_amcl, 10)
        self.node.create_subscription(TFMessage, "/tf", self._cb_tf, 10)
        self.node.create_subscription(Odometry, "/odom", self._cb_odom, 10)

    def _yaw_from_quat(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.z * q.z + q.y * q.y)
        return math.atan2(siny, cosy)

    def _cb_amcl(self, msg: PoseWithCovarianceStamped):
        self.amcl_pose = msg

    def _cb_tf(self, msg: TFMessage):
        for t in msg.transforms:
            y = self._yaw_from_quat(t.transform.rotation)
            if t.header.frame_id == "map" and t.child_frame_id == "odom":
                self.map_odom_yaw = y
            elif t.header.frame_id == "odom" and t.child_frame_id == "base_link":
                self.odom_base_yaw = y

    def _cb_odom(self, msg: Odometry):
        self.odom_msg_yaw = self._yaw_from_quat(msg.pose.pose.orientation)

    def run(self):
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        log_path = LOG_DIR / f"monitor_init_pose_{ts}.csv"

        with open(log_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "elapsed_s",
                "amcl_yaw_deg",
                "map_to_odom_yaw_deg",
                "odom_to_base_yaw_deg",
                "tf_chain_yaw_deg",
                "odom_msg_yaw_deg",
                "amcl_yaw_sigma_deg",
                "note",
            ])

            print("=" * 78)
            print("  导航初始位姿监控 — 运行 40 秒")
            print(f"  日志: {log_path}")
            print("=" * 78)
            header = f"{'t(s)':>5s} | {'AMCLyaw':>7s} | {'m→o':>7s} | {'o→b':>7s} | {'TF链':>7s} | {'/odom':>7s} | {'σ':>5s} | 备注"
            print(header)
            print("-" * 78)

            t0 = time.monotonic()
            last_print = t0
            prev_amcl_yaw = None

            while time.monotonic() - t0 < DURATION:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                now = time.monotonic()

                if self.amcl_pose is None or self.map_odom_yaw is None or self.odom_base_yaw is None:
                    # 数据还没就绪
                    if now - last_print > 5.0:
                        missing = []
                        if self.amcl_pose is None:
                            missing.append("/amcl_pose")
                        if self.map_odom_yaw is None:
                            missing.append("map→odom TF")
                        if self.odom_base_yaw is None:
                            missing.append("odom→base TF")
                        print(f"  ⏳ 等待数据: {', '.join(missing)} ...")
                        last_print = now
                    continue

                # 计算各 yaw
                q = self.amcl_pose.pose.pose.orientation
                amcl_yaw = math.degrees(self._yaw_from_quat(q))
                m2o = math.degrees(self.map_odom_yaw)
                o2b = math.degrees(self.odom_base_yaw)
                tf_chain = math.degrees(self.map_odom_yaw + self.odom_base_yaw)
                odom_yaw = math.degrees(self.odom_msg_yaw) if self.odom_msg_yaw is not None else 0
                cov = self.amcl_pose.pose.covariance
                sigma = math.degrees(math.sqrt(max(0.0, cov[35])))

                # 检测跳变
                note = ""
                if prev_amcl_yaw is not None:
                    dyaw = abs(amcl_yaw - prev_amcl_yaw)
                    if dyaw > 30:
                        note = f"⚠️ 大跳变: {prev_amcl_yaw:.1f}° → {amcl_yaw:.1f}° (Δ={dyaw:.1f}°) !!!"
                    elif dyaw > 10:
                        note = f"⚡ 跳变: {prev_amcl_yaw:.1f}° → {amcl_yaw:.1f}° (Δ={dyaw:.1f}°)"

                elapsed = now - t0
                writer.writerow([f"{elapsed:.2f}", f"{amcl_yaw:.2f}", f"{m2o:.2f}",
                                 f"{o2b:.2f}", f"{tf_chain:.2f}", f"{odom_yaw:.2f}",
                                 f"{sigma:.2f}", note])

                if now - last_print >= 1.0 or note:
                    print(f"{elapsed:5.1f}s | {amcl_yaw:7.2f}° | {m2o:7.1f}° | {o2b:7.1f}° | {tf_chain:7.2f}° | {odom_yaw:7.1f}° | {sigma:4.1f}° | {note}")
                    last_print = now

                prev_amcl_yaw = amcl_yaw

        # 总结
        print("-" * 78)
        if self.records or prev_amcl_yaw is not None:
            print(f"\n日志已保存: {log_path}")
            print(f"\n用以下命令快速看 yaw 变化曲线:")
            print(f"  grep -v '^elapsed' {log_path} | awk -F',' '{{print $1, $2}}'")
        else:
            print("\n⚠️  没有采集到任何数据！请确认导航已完全启动。")

        self.node.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init(args=[])
    monitor = InitPoseMonitor()
    try:
        monitor.run()
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n错误: {e}")


if __name__ == "__main__":
    main()
