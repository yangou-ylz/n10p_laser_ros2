# N10P ROS2 SLAM 项目 — 参考文档入口

> **给 Claude Code 的指令**：这是 N10P ROS2 SLAM 项目的完整参考文档。
> 按文件编号顺序阅读所有 `.md` 文件（01→13），你就能完全理解这个项目的历史、架构、
> 所有踩过的坑、以及当前状态。当前任务是**从 x86 开发机迁移到树莓派 4B**。

## 文件索引（按阅读顺序）

| # | 文件 | 内容 | 阅读时间 |
|---|------|------|----------|
| 01 | `01_project_overview.md` | 项目目标、总体架构、开发阶段总览 | 2 min |
| 02 | `02_hardware_setup.md` | 所有硬件设备、连接方式、串口配置 | 3 min |
| 03 | `03_environment_setup.md` | ROS2/ESP-IDF安装、工作空间、ros2env | 3 min |
| 04 | `04_workspace_structure.md` | 6个ROS2包的结构、每个节点的功能 | 5 min |
| 05 | `05_architecture_decisions.md` | TF树、SLAM选型、数据流设计 | 3 min |
| 06 | `06_development_phases.md` | Phase 0~5.6 每步完成情况 | 5 min |
| 07 | `07_bug_fixes_and_known_issues.md` | 所有bug+修复+已知坑点（最重要！） | 5 min |
| 08 | `08_launch_files_reference.md` | 所有launch文件、参数、使用场景 | 3 min |
| 09 | `09_config_files_reference.md` | 所有YAML配置参数详解 | 3 min |
| 10 | `10_run_commands_cheatsheet.md` | 编译/启动/验证/排错命令速查 | 2 min |
| 11 | `11_raspberry_pi_migration.md` | **树莓派迁移全流程（当前任务）** | 5 min |
| 12 | `12_user_tutorial.md` | 从零开始的完整使用教程 | 5 min |
| 13 | `13_esp32_bridge_guide.md` | ESP32 WiFi桥接完整指南 | 3 min |

## 当前项目状态

```
Phase 0~5.6 全部完成 ✅
Phase 6（树莓派4B移植）← 当前任务
Phase 7（MAVROS无人机集成）← 待树莓派完成后
```

## 关键信息速查

- **开发机**: Ubuntu 22.04 x86_64, RTX 5060, 30GB RAM, 16核CPU
- **目标机**: 树莓派4B, Ubuntu 22.04.05 LTS Server (arm64), TF卡64G + SSD 512G
- **雷达**: 镭神智能 N10P, 360° 单线, 0.02-12m, 10Hz, 串口通信
- **ROS2版本**: Humble Hawksbill
- **SLAM**: slam-toolbox (online async模式)
- **导航**: Nav2 (SmacPlanner2D + RegulatedPurePursuit)
- **传感器接入**: USB有线(lslidar_driver) + ESP32 WiFi无线(n10p_wifi_bridge_node)
