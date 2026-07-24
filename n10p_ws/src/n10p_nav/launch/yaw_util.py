"""SLAM/NAV 共享 — FC yaw 读取与 slam_yaw 持久化"""
import math, time, os

MAPS_DIR = '/home/ylz/n10p_leishen/maps'
SLAM_YAW_FILE = os.path.join(MAPS_DIR, 'slam_yaw.txt')


def read_fc_yaw(timeout=6.0, samples=30):
    """用 rclpy 临时节点订阅 /odom, 取多帧四元数平均, 返回稳定 yaw (rad)"""
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

    yaws = []
    last_msg = None

    def _cb(msg: Odometry):
        nonlocal last_msg
        last_msg = msg

    rclpy.init(args=[])
    node = rclpy.create_node('_temp_yaw_reader')
    best_effort = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST, depth=1)
    sub = node.create_subscription(Odometry, '/odom', _cb, best_effort)

    t0 = time.time()
    while time.time() - t0 < timeout and len(yaws) < samples:
        rclpy.spin_once(node, timeout_sec=0.1)
        if last_msg is not None:
            q = last_msg.pose.pose.orientation
            siny = 2.0 * (q.w * q.z)
            cosy = 1.0 - 2.0 * (q.z * q.z)
            yaws.append(math.atan2(siny, cosy))
            last_msg = None

    node.destroy_node()
    rclpy.shutdown()

    if len(yaws) >= 5:
        avg = sum(yaws) / len(yaws)
        return avg
    return None


def save_slam_yaw(yaw_rad):
    """将 SLAM 建图时的 FC yaw 保存到文件"""
    os.makedirs(MAPS_DIR, exist_ok=True)
    with open(SLAM_YAW_FILE, 'w') as f:
        f.write(f'{yaw_rad:.6f}\n')
    print(f'[SLAM初始化] 飞控 yaw = {yaw_rad:.4f} rad ({math.degrees(yaw_rad):.1f}°) 已保存到 {SLAM_YAW_FILE}')


def load_slam_yaw():
    """读取保存的 SLAM 建图时 yaw, 不存在返回 None"""
    try:
        with open(SLAM_YAW_FILE, 'r') as f:
            return float(f.readline().strip())
    except (FileNotFoundError, ValueError):
        return None


def get_initial_yaw():
    """
    导航时调用: 读取当前 FC yaw → 读取 SLAM yaw → 计算偏移 → 返回 AMCL initial_yaw

    initial_yaw = nav_yaw - slam_yaw  (归一化到 [-π, π])

    如果 slam_yaw 文件不存在 (从未建图), 返回 0
    """
    # === 临时硬编码 (2026-07-24) ===
    # 自动计算 nav_yaw - slam_yaw 的结果（约 -1.1°）无法对齐地图。
    # 在 RViz 中手动指定 2.035 rad (116.6°) 后扫描与地图完美吻合。
    # 临时跳过自动计算，直接返回此值以推进后续联调。
    # TODO: 找到自动计算错误的根本原因后移除本段。
    HARDCODED_YAW = 2.035  # 116.6°, RViz 手动确认的正确初始偏航角
    print(f'[初始位姿] ⚠ 使用硬编码 initial_yaw = {HARDCODED_YAW} rad ({math.degrees(HARDCODED_YAW):.1f}°)')
    print(f'          跳过自动计算 nav_yaw - slam_yaw')
    return HARDCODED_YAW
    # ======================================

    nav_yaw = read_fc_yaw(timeout=6.0, samples=30)
    if nav_yaw is None:
        print('[初始位姿] ⚠ 无法读取 FC yaw, 使用 initial_yaw=0')
        return 0.0

    slam_yaw = load_slam_yaw()
    if slam_yaw is None:
        print(f'[初始位姿] ⚠ 未找到 slam_yaw 文件, 请先建图! 使用 initial_yaw=0')
        print(f'            当前 FC yaw = {math.degrees(nav_yaw):.1f}°')
        return 0.0

    offset = nav_yaw - slam_yaw
    # 归一化到 [-π, π]
    while offset > math.pi:
        offset -= 2 * math.pi
    while offset < -math.pi:
        offset += 2 * math.pi

    print(f'[初始位姿] 建图时 FC yaw = {math.degrees(slam_yaw):.1f}°')
    print(f'           导航时 FC yaw = {math.degrees(nav_yaw):.1f}°')
    print(f'           偏移 = {math.degrees(offset):.1f}° → initial_yaw = {offset:.4f} rad')
    return offset
