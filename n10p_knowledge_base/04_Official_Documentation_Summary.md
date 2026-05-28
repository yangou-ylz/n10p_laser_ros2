# 04 — 官方手册存档与资料索引 (Official Documentation Summary)

> 镭神智能（Leishen Intelligent）N10P 公开资料汇总
>
> 最后更新：2026-05-27

---

## 重要声明

> ⚠️ **在本次情报搜集过程中，镭神智能官方网站未公开发布 N10P 的以下文档**：
>
> - 《N10P 产品规格书 / Datasheet》（PDF）
> - 《N10/N10P 通信协议说明书》（PDF）
> - 《N10P 快速入门指南》
> - 《N10P SDK 开发手册》
>
> 上述文档可能需要通过以下渠道获取：
> 1. 联系镭神技术销售邮箱：**sales@lslidar.com**
> 2. 拨打官方热线：**400-830-6266**
> 3. 通过购买渠道（如淘宝/1688 店铺）向客服索取
> 4. 访问镭神中文官网：https://leishen-lidar.com/
>
> 本知识库的技术参数来自官方产品页、社区逆向分析及实测记录，**不构成官方技术承诺**。

---

## 1. 官方在线资源

### 1.1 官方网站

| 站点 | URL | 语言 | 备注 |
|------|-----|------|------|
| 镭神国际站 | https://www.lslidar.com | 英文 | 含 N10 产品介绍页 |
| 镭神中文站 | https://leishen-lidar.com | 中文 | 含新闻与行业方案 |
| N10 产品页 | https://www.lslidar.com/product/n10-navigation-obstacle-avoidance-lidar/ | 英文 | 基础参数速览 |

### 1.2 官方代码仓库

| 仓库 | URL | 分支建议 |
|------|-----|----------|
| Lslidar_ROS2_driver (官方) | https://github.com/Lslidar/Lslidar_ROS2_driver | `main` |
| Lslidar_ROS (ROS1 版) | https://github.com/Lslidar/Lslidar_ROS | `main` |
| Lslidar_C++ SDK (无 ROS) | https://github.com/Lslidar/Lslidar_C | `main`（未确认是否含 N10P） |

### 1.3 社区 Fork 与第三方

| 仓库 | URL | 说明 |
|------|-----|------|
| bjoernellens1 (N10_V1.0) | https://github.com/bjoernellens1/Lslidar_ROS2_driver | 有 `N10_V1.0` 专用分支 |
| ReDiGermany | https://github.com/ReDiGermany/Lslidar_ROS2_driver | 社区 Fork |
| kiwicampus | https://github.com/kiwicampus/Lslidar_ROS2_driver | 社区 Fork |
| nodehubs/sllidar_ros2 | https://github.com/nodehubs/sllidar_ros2 | 另一套 ROS2 驱动 |
| norlab-ulaval/lslidar_ls128s1 | https://github.com/norlab-ulaval/lslidar_ls128s1 | 提及 Humble 适配 |

---

## 2. N10P 产品页核心参数（官方数据）

以下数据直接来源于镭神智能官方产品页，**可视为官方来源**。

来源：https://www.lslidar.com/product/n10-navigation-obstacle-avoidance-lidar/

| 参数 | 官方值 |
|------|--------|
| 激光波段 | 905nm |
| 激光等级 | Class I |
| 探测距离 | 0.02 ~ 12m @70% 反射率 |
| 测距精度 | ±3cm（0~6m）；±4.5cm（6~12m）@70% |
| 视场角 (FOV) | 360° |
| 水平角分辨率 | 0.48° ~ 0.96° |
| 发射频率 | 4.5KHz |
| 数据点 | 4500 pts/s |
| 防护等级 | IPX-4 |
| 重量 | 60g |
| 尺寸 | φ52 × 36.1mm |

> ⚠️ 注：官方产品页未说明数据接口类型（串口/网口）、波特率、通信协议格式等技术细节。

---

## 3. 协议数据（社区逆向分析）

以下信息来自 CSDN 社区博主对 N10P 的 Wireshark 数据包逆向分析，**非官方文档**：

来源：https://blog.csdn.net/2401_84582222/article/details/147636777

| 字段 | 字节 | 内容 |
|------|------|------|
| 帧头 | 2 | `A5 5A` |
| 转速参数 | 2 | `转速(rpm) = 2,500,000 / 值` |
| 起始角度 | 2 | 单位 0.01° |
| 距离数据 | 70×2 | 单位 mm，小端序，`0xFFFF`=无效 |
| 帧尾 | 2 | `FA FB` |

---

## 4. 驱动架构信息（官方源码分析）

以下信息来自对 GitHub 官方仓库 README 和 launch 文件的分析：

### 4.1 支持的单线雷达型号

```
M10 / M10GPS / M10P
N10 / N10Plus (N10P)
N301 (1.6 / 1.7)
L10
```

### 4.2 核心节点与数据流

```
lslidar_driver_node
  ├── 输入：/dev/ttyACM0 (串口)
  └── 输出：
      ├── /scan (sensor_msgs/LaserScan)     ← 主要使用
      └── /diagnostics (diagnostic_msgs)
```

### 4.3 Launch 文件

| 文件 | 用途 |
|------|------|
| `lsn10p_launch.py` | N10P 专用启动文件 |
| `lslidar_serial.launch.py` | 通用串口版 |
| `lslidar_net.launch.py` | 网口版 |
| `viewer_scan_launch.py` | 启动雷达 + RViz2 |

---

## 5. 未能获取的官方 PDF 文档清单

以下文档在本次研究中**未找到**公开下载链接。如果你有购买渠道，建议向官方索取：

- [ ] 《N10P 激光雷达产品规格书》 (Datasheet)
- [ ] 《N10/N10P 通信协议说明书》
- [ ] 《N10P 硬件接口定义》 (引脚图、电气特性)
- [ ] 《N10P SDK 开发手册》
- [ ] 《N10P 快速入门指南》
- [ ] 镭神 ROS2 驱动用户手册

> `n10p_knowledge_base/assets/` 目录已准备好，后续如有获取到的 PDF 可直接放入。

---

## 6. 其他可用参考资料（非 N10P 专用）

镭神官网上有一些通用激光雷达的说明文档（虽然不直接针对 N10P，但可辅助理解）：

| 资源 | URL | 备注 |
|------|-----|------|
| 车载激光雷达画册 | https://www.lslidar.com/wp-content/uploads/2022/10/车规级画册.pdf | 非 N10P |
| 镭神产品线概览 | https://www.lslidar.com/products/ | 全系产品 |

---

## 7. 建议下一步行动

1. **获取官方协议文档**：联系 sales@lslidar.com 索取 N10P 通信协议说明书
2. **验证波特率**：用串口工具（如 `minicom` 或 `screen`）实测 N10P 的实际波特率
3. **克隆官方驱动**：待环境准备完成后，克隆 `Lslidar_ROS2_driver` 并编译测试
4. **配置 Udev 规则**：完成 `02_ROS2_Development_Guide.md` 中的永久权限配置
5. **RViz2 联调**：按 `03_Visualization_and_Troubleshooting.md` 指南验证点云显示

---

## 8. 声明

本知识库中的社区来源信息可能随时间失效或与你的实际硬件版本不符。
请始终以**雷达实际表现**和**官方最新文档**为最终依据，社区资料仅作参考。
