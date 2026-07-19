#!/usr/bin/env python3
"""AMCL 收敛监控 — 每秒刷新粒子数/散布/收敛状态"""
import math, time, sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray


class ConvergenceMonitor(Node):
    def __init__(self):
        super().__init__('_amcl_monitor')
        self.cov = None
        self.particle_count = 0
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._on_pose, 10)
        self.create_subscription(PoseArray, '/particle_cloud',
                                 self._on_particles, 10)
        print("AMCL 收敛监控 (Ctrl+C 退出)")
        print(f"{'时间':>8s}  {'粒子':>6s}  {'Xσ(cm)':>8s}  {'Yσ(cm)':>8s}  {'Yawσ(°)':>8s}  {'状态'}")
        print("-" * 70)

    def _on_pose(self, msg):
        c = msg.pose.covariance
        self.cov = (c[0], c[7], c[35])  # x_var, y_var, yaw_var

    def _on_particles(self, msg):
        self.particle_count = len(msg.poses)

    def status_str(self):
        if self.cov is None:
            return "等待数据..."
        x, y, yaw = self.cov
        xs = math.sqrt(max(0,x))*100
        ys = math.sqrt(max(0,y))*100
        ysaw = math.sqrt(max(0,yaw))*180/math.pi
        if xs < 5 and ys < 5 and ysaw < 3:
            st = "✅ 已收敛"
        elif xs < 20 and ysaw < 10:
            st = "🟡 收敛中"
        else:
            st = "❌ 未收敛"
        t = time.strftime("%H:%M:%S")
        return f"{t:>8s}  {self.particle_count:>6d}  {xs:>8.1f}  {ys:>8.1f}  {ysaw:>8.1f}  {st}"


def main():
    rclpy.init()
    node = ConvergenceMonitor()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)
            print(node.status_str())
    except KeyboardInterrupt:
        print("\n停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
