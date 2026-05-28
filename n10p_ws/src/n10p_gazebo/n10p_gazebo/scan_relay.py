#!/usr/bin/python3
"""Scan 话题转发: /n10p_lidar_plugin/out → /scan"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.sub = self.create_subscription(
            LaserScan, '/n10p_lidar_plugin/out', self.cb, 10)
        self.get_logger().info('/n10p_lidar_plugin/out → /scan 转发中')

    def cb(self, msg: LaserScan):
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(ScanRelay())
    rclpy.shutdown()
