#!/usr/bin/env python3
"""读取飞控当前 yaw 角，用于 AMCL 自动初始位姿"""
import sys, math, time, subprocess

def get_fc_yaw(timeout=3.0):
    """从 /odom 话题读取飞控四元数，返回 yaw (rad)"""
    cmd = "source /opt/ros/humble/setup.bash && ros2 topic echo /odom --once 2>/dev/null"
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        try:
            r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=2)
            for line in r.stdout.split('\n'):
                if 'z:' in line and 'orientation' not in line and 'angular' not in line:
                    z = float(line.split(':')[1].strip())
                if 'w:' in line:
                    w = float(line.split(':')[1].strip())
                    siny = 2.0 * (w * z)
                    cosy = 1.0 - 2.0 * (z * z)
                    return math.atan2(siny, cosy)
        except Exception:
            pass
        time.sleep(0.5)
    return None

if __name__ == '__main__':
    yaw = get_fc_yaw()
    if yaw is not None:
        print(f"{yaw:.4f}")
    else:
        print("0.0000", file=sys.stderr)
        sys.exit(1)
