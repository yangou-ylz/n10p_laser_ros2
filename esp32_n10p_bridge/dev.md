# ESP32-N10P 桥接固件 开发日志

> 创建：2026-05-31 | 每阶段追加

---

## Phase 1：UART 接收验证 ✅ 完成 (2026-05-31)

### 目标

ESP32-S3 通过 UART1 接收 N10P 雷达原始帧，帧同步 + CRC 校验，串口监视器输出统计。

### 硬件连接

```
N10P 雷达 CH9102 模块 (TTL 侧)        ESP32-S3 开发板
─────────────────────────────        ────────────────
TX (N10P 数据输出)  ─────────────→   IO18 (GPIO18, UART1 RX)
GND                  ─────────────→   GND
```

| 引脚 | 方向 | 说明 |
|------|------|------|
| CH9102 TX → ESP32 IO18 | 雷达 → ESP32 | N10P 数据输出，**必须接对，不能接到 RX** |
| CH9102 RX | 未接 | 控制命令回传（Phase 2+ 才需要） |
| GND ↔ GND | 双向 | 共地，必须接 |
| N10P USB 5V | 供电 | 插电脑或充电头给雷达供电 |

### 验证结果

| 指标 | 实测值 | 预期 |
|------|--------|------|
| 数据速率 | ~35.8 KB/s | 35-45 KB/s |
| 帧率 | ~332 fps | 330-425 fps |
| 帧头 (A5 5A) | 100% 正确 | 100% |
| CRC8 通过率 | 100% | >99% |
| 同步丢失 | 0 | <1% |
| 每帧大小 | 108 字节 | 108 字节 |

### 踩坑记录

#### 坑 1：烧录时 ESP32 连不上

**现象**：`Failed to connect to ESP32-S3: No serial data received`
**根因**：ESP32-S3 上电后进入正常运行模式，不自动进烧录模式
**解决**：按住 BOOT 键 → 点按 RST → 松开 BOOT，进入 DOWNLOAD 模式即可烧录

#### 坑 2：接到 CH9102 的 RX 而非 TX — 致命

**现象**：监视器收到 400 字节垃圾（FF EF D9...）后持续 0 字节
**根因**：CH9102 模块有两根数据线 TX 和 RX，**接错到 RX**
  - TX = N10P 向外发送数据（雷达 → 电脑），这根要接 ESP32 RX(IO18)
  - RX = 电脑向 N10P 发命令（电脑 → 雷达），接这根会收到噪声
**解决**：换到 TX 引脚即可
**验证**：换引脚后立即收到 A5 5A 帧头，332fps 稳定
**教训**：先用电脑 USB 直连验证雷达数据正常（`stty -F /dev/ttyACMx 460800 raw && xxd`），
  确认 N10P 正常后再排查接线

#### 坑 3：esp_timer_get_time 编译错误

**现象**：`implicit declaration of function 'esp_timer_get_time'`
**根因**：main.c include 了 esp_timer.h，但 CMakeLists.txt 没声明依赖
**解决**：main/CMakeLists.txt 的 REQUIRES 加 `esp_timer`
**规则**：ESP-IDF 每个 #include 的组件必须在 REQUIRES 里声明

### 关键配置

- UART1: IO18(RX), IO17(TX), 460800-8N1
- 软件缓冲区: 108×20=2160 字节
- 帧同步状态机: WAIT_HEADER0 → WAIT_HEADER1 → COLLECT(108B) → CRC_CHECK
- CRC8: 累加和取低 8 位（与 lslidar_driver 的 N10_CalCRC8 一致）
- 统计任务: uart_rx_task 绑定 Core 1（Core 0 留给 WiFi 协议栈）

---

## Phase 2：WiFi TCP 转发 ✅ 完成 (2026-05-31)

### 目标

ESP32-S3 WiFi Station 连接路由器 → TCP Server (port 8888) → UART 收到的 N10P 帧通过 FreeRTOS 队列转发到 TCP 客户端。

### 架构

```
Core 1: UART RX → 帧同步 → CRC8 → 有效帧 → FreeRTOS 队列 (32帧)
                                                ↓
Core 0: WiFi Stack ← TCP Server → 队列取帧 → 加协议头 → send()
```

### TCP 帧格式

**v3 (Phase 2)**: 116 字节 (8B 头 + 108B N10P)，头含 seq 和 len
**v4 (Phase 3)**: **108 字节纯 N10P 原始帧**，无额外包装。TCP 流式传输，
  lslidar_driver 自带帧同步 (A5 5A)，不需要外层包装。

### 验证结果

| 指标 | 实测值 | 状态 |
|------|--------|------|
| WiFi 连接 | 192.168.0.184, RSSI -24 | ✅ |
| TCP Server | 端口 8888, 监听正常 | ✅ |
| UART 接收 | ~332 fps, CRC 100% | ✅ |
| TCP 转发 (v3) | nc 连接成功收到 116 字节帧 | ✅ |
| TCP 转发 (v4) | nc 连接收到纯 108 字节 N10P 帧 | ✅ |
| 固件大小 | 776KB (1MB 分区) | ✅ |

### 测试命令

**电脑端验证**：
```bash
# v4 纯原始帧
nc 192.168.0.184 8888 | xxd | head -10
# 预期: 每行以 a55a 6c10 直接开头 (无外层包装)
```

### 踩坑记录

#### 坑 4：TCP 层包装是多余的

**发现**：Phase 2 加了 8 字节 TCP 帧头 (sync + seq + len)，后来发现没必要
**原因**：TCP 本身保证有序送达，N10P 帧自带 A5 5A 同步头，lslidar_driver 自带帧解析
**解决**：v4 去掉包装，直接发 108 字节原始帧，更简洁

#### 坑 5：WiFi 编译固件体积暴涨

**现象**：加入 WiFi + TCP 后固件从 237KB → 776KB
**根因**：WiFi 驱动 + lwIP 协议栈 + TCP socket API 占用大量 flash
**影响**：当前 1MB 分区仍有 23% 余量，不影响使用

---

## Phase 3：电脑端 socat 虚拟串口 ✅ 完成 (2026-05-31)

### 目标

电脑端用 socat 将 ESP32 TCP 数据流映射为 PTY 虚拟串口，lslidar_driver 零改动接入。

### 架构

```
ESP32 WiFi TCP (192.168.0.184:8888)
        │
        ▼
socat PTY → /tmp/n10p_esp32 (虚拟串口)
        │
        ▼
lslidar_driver (不改代码, serial_port: /tmp/n10p_esp32)
        │
        ▼
/scan 话题 → SLAM / Nav2 / RViz2
```

### 验证结果

| 指标 | 实测值 | 状态 |
|------|--------|------|
| socat PTY 创建 | /tmp/n10p_esp32 成功 | ✅ |
| 数据流通 | 10KB 传输, A5 5A 帧头正常 | ✅ |
| 帧率 | ~332 fps (与 Phase 1 一致) | ✅ |
| 数据格式 | 108 字节纯 N10P 帧, 与直连串口一致 | ✅ |

### 测试命令

```bash
# 1. 启动 socat 虚拟串口
socat PTY,link=/tmp/n10p_esp32,raw TCP:192.168.0.184:8888 &

# 2. 验证虚拟串口数据
xxd /tmp/n10p_esp32 | head -10
# 预期: a55a 6c10 开头, 108 字节一帧

# 3. 数据量验证
timeout 5 cat /tmp/n10p_esp32 | wc -c
# 预期: 5 秒约 175KB (35KB/s × 5)

# 4. 接入 lslidar_driver (改一行配置)
# lsx10.yaml: serial_port: "/dev/ttyACM0" → "/tmp/n10p_esp32"
```

### 踩坑记录

#### 坑 6：dd 读 PTY 时报 partial read

**现象**：`dd if=/tmp/n10p_esp2 bs=108 count=100` 报 partial read
**根因**：TCP 流到达时机不按帧边界对齐，dd 按 108 字节固定块读会截断
**解决**：使用 `cat` 或流式读取。lslidar_driver 本身是流式读 + 帧同步,
  不受影响。dd 测试时用 `cat > file` 替代

#### 坑 7：socat wait-slave 导致数据不流

**现象**：加了 `wait-slave` 选项后 xxd 读 `/tmp/n10p_esp32` 没数据
**根因**：`wait-slave` 等待从端打开才建立连接，但 PTY 的 slave 端被 xxd
  打开时 TCP 连接可能还没建立好
**解决**：去掉 `wait-slave`，直接用 `raw` 模式

### 关键配置

- socat: `PTY,link=/tmp/n10p_esp32,raw TCP:192.168.0.184:8888`
- PTY 路径: `/tmp/n10p_esp32`
- lslidar_driver 配置: `serial_port: "/tmp/n10p_esp32"`
  （修改 `lsx10.yaml` 第 18 行）

---

## Phase 4：端到端集成测试 ✅ 完成 (2026-05-31)

### 目标

完整链路 N10P → ESP32 → WiFi TCP → ROS2 `/scan` 话题 → RViz2 可视化。

### 架构 (Phase 3 socat 方案失败后的最终方案)

Phase 3 尝试用 socat PTY 让 lslidar_driver 无改动接入，但驱动 `tcsetattr()` 破坏了
PTY 的终端属性导致 poll() 永远等不到数据。

Phase 4 改为**独立 Python ROS2 节点**：连 ESP32 TCP，自行解析 N10P 帧，发布 `/scan`。
不修改任何现有 ROS2 代码，作为有线模式的替代方案独立存在。

```
N10P → ESP32 UART → WiFi TCP (192.168.0.184:8888)
                         │
                         ▼
              n10p_wifi_bridge.py (独立 Python 节点)
                         │
                         ▼
                   /scan (LaserScan, 10Hz)
                         │
                         ▼
                   RViz2 / SLAM / Nav2
```

### 文件

`esp32_n10p_bridge/n10p_wifi_bridge.py` — 独立 ROS2 节点，零依赖现有项目代码

### 验证结果

| 指标 | 实测值 | 状态 |
|------|--------|------|
| TCP 连接 | ESP32 192.168.0.184:8888 | ✅ |
| 帧接收 | ~330 fps, CRC 100% | ✅ |
| /scan 发布 | 稳定 10.000Hz | ✅ |
| 距离数据 | 0.27m ~ 11.02m 真实测距 | ✅ |
| 帧格式 | frame_id=laser_frame, 360° 完整扫描 | ✅ |

### 运行方法

```bash
# 终端 1: ESP32 自动运行 (上电即连 WiFi + TCP Server)
# 终端 2: 启动桥接节点
ros2env
python3 /home/ubuntu22/ROS2/n10p_leishen/esp32_n10p_bridge/n10p_wifi_bridge.py

# 终端 3: 验证
ros2env
ros2 topic hz /scan           # 预期: 10Hz
ros2 topic echo /scan --once --field ranges | head -5  # 预期: 有效距离值
```

### 与有线模式切换

| 模式 | 启动方式 | 优点 |
|------|----------|------|
| 有线 | `ros2 launch lslidar_driver lslidar_launch.py` | 零延迟, 稳定 |
| 无线 | `python3 n10p_wifi_bridge.py` | 雷达可独立移动, 不受数据线约束 |

两个模式互斥——都发布到 `/scan`，只能同时运行一个。

### 踩坑记录

#### 坑 8：socat PTY 被 tcsetattr 破坏

**现象**：驱动 `open_port /tmp/n10p_esp32 OK` 后无限卡住，既不报 poll timeout
  也不处理数据
**根因**：lslidar_driver 的 `tcsetattr()` 设置 PTY 终端属性（波特率、VMIN、VTIME 等），
  改变了 PTY 的行规约(line discipline)，导致 poll() 不报告数据就绪
**解决**：放弃 socat PTY 方案。改用独立 Python 节点，TCP socket 直接读数据，
  完全不经过终端层
**教训**：PTY 不是真正的串口，tcsetattr 对 PTY 的影响与真实串口不同

#### 坑 9：N10P 帧解析角度映射复杂

**现象**：初版桥接节点的 ScanAccumulator 积累点数极少（20-70），/scan 无有效数据
**根因**：N10P 每帧仅覆盖约 6° 扇形（16 个点），angle_increment 和点数计算有误
**解决**：使用与驱动一致的参数（count_num=2000, scan_num=4000），定时器 10Hz
  强制发布，不等待完整一圈

### 关键配置

- 桥接命令: `python3 n10p_wifi_bridge.py [--host IP] [--port PORT]`
- 默认主机: 192.168.0.184:8888
- 发布频率: 10Hz (0.1s 定时器)
- scan_num: 4000 (2×count_num, 每度约 11 个点)
- 帧解析: 6 字节/点 (dist+conf+reserved), CRC8 累加和

---

## 整合总结：ESP32 WiFi 方案接入现有 N10P ROS2 项目

### 你需要关注的文件

| 文件 | 角色 |
|------|------|
| `main/main.c` | ESP32 固件。烧录一次，上电自动运行。不可动除非改硬件引脚 |
| `n10p_wifi_bridge.py` | **你唯一的整合入口。** 独立 Python 节点，产生与 lslidar_driver 完全相同的 `/scan` |
| `user.md` | 使用教程，接线+烧录+运行+排错 |

以下文件是开发过程产物，**整合时不需要**：
- sdkconfig, CMakeLists.txt（编译一次就不用了）
- tcp2pty.py（Phase 3 尝试的失败方案）
- SETUP.md（环境搭建，已装好就不用看了）

### 现有 ROS2 项目怎么接入

你现有的 ROS2 项目数据流是：

```
N10P串口 → lslidar_driver → /scan → SLAM/Nav2/RViz2
```

加入 ESP32 无线方案后，多了一条路径：

```
N10P串口 → ESP32 (WiFi TCP) → n10p_wifi_bridge.py → /scan → SLAM/Nav2/RViz2
```

**两条路径输出完全相同的 `/scan`（LaserScan, laser_frame, 10Hz, 4000点/圈）**，
下游 SLAM/Nav2/RViz2 不需要任何改动。

### 切换方式

```bash
# 有线模式（原方案）
ros2 launch lslidar_driver lslidar_launch.py

# 无线模式（新增）
python3 esp32_n10p_bridge/n10p_wifi_bridge.py
```

只需二选一启动，不可同时运行。其他 launch 文件（SLAM、Nav2）都不受影响。

### 如果你想整合到 launch 文件里

在现有 launch 文件中加一个条件分支即可：

```python
# 在 launch 文件中
use_wireless = LaunchConfiguration('use_wireless', default='false')

# 无线模式
wireless_node = Node(
    package='n10p_bringup',
    executable='n10p_wifi_bridge.py',  # 需加到 setup.py
    condition=IfCondition(use_wireless),
)
```

或者更简单：保持 `n10p_wifi_bridge.py` 独立运行，不在 launch 中管理。SLAM/Nav2 的 launch 不需要感知 `/scan` 来源于有线还是无线。

### 关键参数（可能需要调）

| 参数 | 位置 | 默认值 | 何时改 |
|------|------|--------|--------|
| ESP32 IP | n10p_wifi_bridge.py `--host` | 192.168.0.184 | 换路由器或 DHCP 变了 |
| WiFi SSID/密码 | main/main.c `WIFI_SSID`/`WIFI_PASS` | YLZ / yy060315 | 换 WiFi 环境时重新编译固件 |
| TCP 端口 | main/main.c + n10p_wifi_bridge.py | 8888 | 一般不需要改 |
| IO 引脚 | main/main.c `UART_RX_PIN` | 18 | 只有改硬件接线时改 |

### 性能红线

- ESP32 固件 776KB (1MB 分区, 23% 余量)
- CPU: Core0 (WiFi+TCP) + Core1 (UART接收) 均远未满载
- 无线延时比有线多 2-5ms (WiFi 局域网), 不影响 SLAM
- 当前扫频 332fps / 10Hz 发布, 完全满足原有线方案的节奏
