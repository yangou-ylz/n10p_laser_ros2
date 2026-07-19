#!/usr/bin/env python3
"""
N10P 系统健康检查脚本 — 一键诊断所有节点/话题/TF/频率

用法:
  python3 n10p_health_check.py              # 自动检测当前模式
  python3 n10p_health_check.py --mode slam  # SLAM 建图模式
  python3 n10p_health_check.py --mode nav   # Nav2 导航模式
  python3 n10p_health_check.py --watch      # 持续监控 (每 5s 刷新)
"""
import subprocess, sys, time, argparse, re
from collections import OrderedDict

# ═══════════════════════════════════════════════════════════
# 每种模式的期望清单
# ═══════════════════════════════════════════════════════════

SLAM_NODES = OrderedDict({
    'lslidar_driver_node': 'N10P 雷达驱动 (C++)',
    'ano_bridge_node':      '凌霄飞控桥接 (→/odom+/imu)',
    'imu_filter_node':      'EKF 互补滤波 (→TF)',
    'slam_toolbox':         'slam-toolbox SLAM建图',
})

NAV_NODES = OrderedDict({
    'lslidar_driver_node': 'N10P 雷达驱动',
    'ano_bridge_node':      '凌霄飞控桥接',
    'imu_filter_node':      'EKF 互补滤波',
    'map_server':           '地图加载',
    'amcl':                 'AMCL 粒子滤波定位',
    'planner_server':       '全局路径规划',
    'controller_server':    '局部路径跟踪',
    'bt_navigator':         '行为树导航器',
})

ALL_TOPICS = OrderedDict({
    '/scan':                'LaserScan, 10Hz',
    '/odom':                'Odometry, 50Hz (飞控原始)',
    '/imu':                 'Imu, ≤100Hz (飞控IMU)',
    '/odometry/filtered':   'Odometry, 50Hz (EKF滤波后)',
    '/tf':                  'TF 动态变换',
    '/tf_static':           'TF 静态变换',
})

NAV_TOPICS = OrderedDict({
    '/map':                 'OccupancyGrid (静态地图)',
    '/amcl_pose':           'PoseWithCovarianceStamped (定位)',
    '/plan':                'Path (全局路径)',
    '/cmd_vel':             'Twist (速度指令)',
    '/ekf_status':          'String (EKF诊断)',
})

TF_CHAIN = [
    ('base_link', 'laser_frame', '静态TF, 雷达安装位'),
    ('odom',      'base_link',   'EKF/ano_bridge, 里程计'),
    ('map',       'odom',        'AMCL/SLAM, 定位修正'),
]


def ros2_cmd(args, timeout=5):
    try:
        r = subprocess.run(f'source /opt/ros/humble/setup.bash && {args}',
                           shell=True, capture_output=True, text=True, timeout=timeout)
        return r.stdout.strip(), r.stderr.strip(), r.returncode
    except subprocess.TimeoutExpired:
        return '', 'TIMEOUT', -1


def check_node(name, desc):
    out, _, _ = ros2_cmd(f'ros2 node list 2>/dev/null | grep -w {name}')
    found = name in out
    return found, desc


def check_topic(name, desc):
    out, _, _ = ros2_cmd(f'ros2 topic info {name} 2>/dev/null', 3)
    pub_match = re.search(r'Publisher count:\s*(\d+)', out)
    sub_match = re.search(r'Subscription count:\s*(\d+)', out)
    pub = int(pub_match.group(1)) if pub_match else 0
    sub = int(sub_match.group(1)) if sub_match else 0
    ok = pub > 0
    return ok, f'{desc} | Pub={pub} Sub={sub}'


def check_tf(parent, child, desc):
    out, _, _ = ros2_cmd(
        f'timeout 3 ros2 run tf2_ros tf2_echo {parent} {child} 2>&1 | head -3', 5)
    ok = 'At time' in out or 'Translation' in out
    return ok, desc


def check_all(mode):
    nodes = NAV_NODES if mode == 'nav' else SLAM_NODES
    topics = {**ALL_TOPICS}
    if mode == 'nav':
        topics.update(NAV_TOPICS)

    print(f"\n{'='*65}")
    print(f"  N10P 系统健康检查 — {'NAV2 导航' if mode == 'nav' else 'SLAM 建图'} 模式")
    print(f"{'='*65}")

    # ── 节点 ─────────────────────────────────────
    print(f"\n── 节点 ({len(nodes)}个) {'─'*50}")
    node_ok = 0
    for name, desc in nodes.items():
        ok, desc = check_node(name, desc)
        tag = '✅' if ok else '❌ 缺失'
        if ok: node_ok += 1
        print(f'  {tag}  {name:<25s} {desc}')

    # ── 话题 ─────────────────────────────────────
    print(f"\n── 话题 ({len(topics)}个) {'─'*50}")
    topic_ok = 0
    for name, desc in topics.items():
        ok, detail = check_topic(name, desc)
        tag = '✅' if ok else '❌ 无发布者'
        if ok: topic_ok += 1
        print(f'  {tag}  {name:<25s} {detail}')

    # ── TF 链 ─────────────────────────────────────
    print(f"\n── TF 坐标链 ({len(TF_CHAIN)}段) {'─'*46}")
    tf_ok = 0
    for parent, child, desc in TF_CHAIN:
        ok, desc = check_tf(parent, child, desc)
        tag = '✅' if ok else '❌ 断裂'
        if ok: tf_ok += 1
        print(f'  {tag}  {parent} → {child:<15s} {desc}')

    # ── 关键频率 ──────────────────────────────────
    print(f"\n── 关键频率 {'─'*55}")
    for topic, expected in [('/scan', 10), ('/odom', 50)]:
        out, _, _ = ros2_cmd(
            f'timeout 8 ros2 topic hz {topic} --window 10 2>/dev/null', 10)
        match = re.search(r'average rate:\s*([\d.]+)', out)
        if match:
            hz = float(match.group(1))
            if topic == '/scan':
                tag = '✅' if 8 <= hz <= 12 else '⚠️'
            else:
                tag = '✅' if hz > 10 else '⚠️'
            print(f'  {tag}  {topic:<25s} {hz:.1f} Hz (期望 ~{expected} Hz)')
        else:
            print(f'  ❌  {topic:<25s} 无数据')

    # ── 总结 ─────────────────────────────────────
    n_total = len(nodes)
    t_total = len(topics)
    total = n_total + t_total + 3
    ok_total = node_ok + topic_ok + tf_ok
    print(f"\n{'='*65}")
    if ok_total == total:
        print(f"  结果: ✅ 全部通过 ({ok_total}/{total})")
    else:
        fail = total - ok_total
        print(f"  结果: ❌ {fail} 项异常 ({ok_total}/{total} 通过)")
        print(f"  节点: {node_ok}/{n_total}  话题: {topic_ok}/{t_total}  TF: {tf_ok}/3")
    print(f"{'='*65}\n")
    return ok_total == total


def auto_detect():
    """自动检测当前是 SLAM 还是 NAV 模式"""
    out, _, _ = ros2_cmd('ros2 node list 2>/dev/null')
    if 'bt_navigator' in out or 'planner_server' in out:
        return 'nav'
    if 'slam_toolbox' in out:
        return 'slam'
    print('⚠️  未检测到 SLAM 或 NAV 节点, 默认使用 SLAM 模式')
    return 'slam'


def main():
    parser = argparse.ArgumentParser(description='N10P 系统健康检查')
    parser.add_argument('--mode', choices=['slam', 'nav'],
                        help='检查模式 (自动检测)')
    parser.add_argument('--watch', action='store_true',
                        help='持续监控 (每 5s 刷新)')
    args = parser.parse_args()

    mode = args.mode or auto_detect()

    if args.watch:
        try:
            while True:
                check_all(mode)
                time.sleep(5)
        except KeyboardInterrupt:
            print('\n停止监控')
    else:
        ok = check_all(mode)
        sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
