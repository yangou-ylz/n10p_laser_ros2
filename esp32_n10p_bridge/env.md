# ESP32-S3 开发环境配置文档

> 创建：2026-05-31 | 基于本次搭建的实战经验和踩坑记录

---

## 1. 硬件环境

| 项目 | 配置 |
|------|------|
| CPU | 16 核 (型号待补充) |
| RAM | 30GB |
| GPU | RTX 5060 |
| 系统 | Ubuntu 22.04 |
| 目标芯片 | ESP32-S3 N16R8 |
| 开发板 | ESP32-S3 开发板 (带排针 + USB 转串口) |

---

## 2. 安装 ESP-IDF v5.3.2

### 2.1 克隆仓库

```bash
mkdir -p ~/esp
cd ~/esp
git clone --depth 1 --branch v5.3.2 https://github.com/espressif/esp-idf.git
```

### 2.2 安装工具链

**有外网/VPN 时**（直接下载，约 15-20 分钟）：
```bash
cd ~/esp/esp-idf && bash install.sh esp32s3
```

**国内网络时**（用乐鑫国内镜像）：
```bash
export IDF_GITHUB_ASSETS="dl.espressif.cn/github_assets"
cd ~/esp/esp-idf && bash install.sh esp32s3
```

下载量约 289MB（6 个工具：xtensa-esp-elf-gdb 30MB, xtensa-esp-elf 107MB, riscv32-esp-elf 139MB, esp32ulp-elf 11MB, openocd-esp32 2.3MB, esp-rom-elfs 几MB）。

### 2.3 配置 git 镜像（重要！国内必做）

ESP-IDF 有 20+ 个 git 子模块托管在 GitHub，国内直连经常 TLS 断连。需要配置镜像：

```bash
# 通用镜像（多数子模块走这个）
git config --global url."https://jihulab.com/esp-mirror/espressif/esp-idf".insteadOf "https://github.com/espressif/esp-idf"

# esp32-wifi-lib（1.7GB，最大的子模块）
git config --global url."https://jihulab.com/esp-mirror/espressif/esp32-wifi-lib".insteadOf "https://github.com/espressif/esp32-wifi-lib.git"

# micro-ecc（670KB 但必须精确版本）
git config --global url."https://jihulab.com/esp-mirror/kmackay/micro-ecc".insteadOf "https://github.com/kmackay/micro-ecc.git"
```

装完后可用 `git config --global --list | grep insteadOf` 查看所有镜像配置。

### 2.4 更新子模块

```bash
cd ~/esp/esp-idf

# 跳过 micro-ecc（后面单独处理）
git -c submodule."components/bootloader/subproject/components/micro-ecc/micro-ecc".update=none \
    submodule update --init --recursive
```

### 2.5 修复 micro-ecc 版本

micro-ecc 子模块被跳过后会出问题——bootloader 编译时需要它，且必须是精确版本：

```bash
rm -rf components/bootloader/subproject/components/micro-ecc/micro-ecc
git clone https://jihulab.com/esp-mirror/kmackay/micro-ecc \
    components/bootloader/subproject/components/micro-ecc/micro-ecc
cd components/bootloader/subproject/components/micro-ecc/micro-ecc
git fetch origin
git checkout 24c60e243580c7868f4334a1ba3123481fe1aa48
cd ~/esp/esp-idf
```

> 版本号 `24c60e` 来自 `.gitmodules` 中 micro-ecc 的 `sbom-hash` 字段。
> ESP-IDF 升级时需重新确认此 hash。

### 2.6 验证安装

```bash
source ~/esp/esp-idf/export.sh
xtensa-esp32s3-elf-gcc --version
# 应输出: xtensa-esp-elf-gcc ... 13.2.0
```

---

## 3. 创建新工程

### 3.1 目录结构

```
your_project/
├── CMakeLists.txt              # 顶层 CMake
├── sdkconfig                   # 由 idf.py set-target 生成
├── main/
│   ├── CMakeLists.txt          # 组件 CMake
│   └── main.c                  # 入口
└── README.md
```

### 3.2 顶层 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(你的项目名)
```

### 3.3 main/CMakeLists.txt

```cmake
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash driver esp_wifi esp_timer
)
```

**关键规则**：代码里 `#include "xxx.h"` 了哪个组件，就必须在 `REQUIRES` 里声明它。
否则 cmake 不会报错，但 ninja 编译时会报 `fatal error: xxx.h: No such file or directory`
并提示 `add xxx to PRIV_REQUIRES`。

---

## 4. 编译命令

```bash
# 每次新开终端必须先 source（或在 ~/.bashrc 加别名）
source ~/esp/esp-idf/export.sh

# 首次：设置目标芯片
idf.py set-target esp32s3

# 编译
idf.py build

# 清理后重新编译
rm -rf build && idf.py set-target esp32s3 && idf.py build

# 烧录 + 监视器
idf.py -p /dev/ttyUSB0 flash monitor
```

- `set-target` 只需要执行一次（生成 sdkconfig）
- `build` 是增量编译，已编译的模块不会重编
- bootloader 编译约 1 分钟，完整首次编译约 3-5 分钟
- 二次增量编译约 10-20 秒

---

## 5. 踩坑记录

### 坑 1：GitHub 下载极慢/断连

**现象**：`install.sh` 下载工具链时只有几十 KB/s，经常超时
**根因**：GitHub 服务器在国外
**解决**：加 `IDF_GITHUB_ASSETS="dl.espressif.cn/github_assets"` 环境变量，或连外网

### 坑 2：子模块 esp_wifi/lib 反复 TLS 断连

**现象**：cmake 自动拉子模块时，esp_wifi/lib (1.7GB) 下到一半 `GnuTLS recv error`
**根因**：GitHub 对国内长连接不稳定
**解决**：用 `git config --global url.xxx.insteadOf` 重定向到 jihulab.com 镜像

### 坑 3：micro-ecc "需要一个单独的版本"

**现象**：`git submodule update` 报 `fatal: 需要一个单独的版本`
**根因**：`--depth 1` 克隆的 ESP-IDF 没有 micro-ecc 的历史记录，子模块无法定位到需要的 commit
**解决**：跳过 micro-ecc 更新其他子模块，然后手动 clone + checkout 精确版本

### 坑 4：micro-ecc API 不兼容

**现象**：手动 clone 了最新版 micro-ecc，编译 bootloader 时报 `too few arguments to function 'XYcZ_add'`
**根因**：最新版 micro-ecc 的 API 与 ESP-IDF v5.3.2 封装的 `uECC_verify_antifault.c` 不兼容
**解决**：checkout 到 `.gitmodules` 中指定的精确 commit `24c60e2`

### 坑 5：implicit declaration of function 'esp_timer_get_time'

**现象**：编译 own 代码时报 `implicit declaration of function 'esp_timer_get_time'`
**根因**：代码里 include 了 `esp_timer.h` 但 CMakeLists.txt 没声明依赖
**解决**：两个地方都要改——代码加 `#include "esp_timer.h"`，CMakeLists.txt 的 REQUIRES 加 `esp_timer`

### 坑 6：cmake build 目录残留

**现象**：`idf.py set-target` 报 `doesn't seem to be a CMake build directory`
**根因**：上一次 cmake 中途失败，build 目录不完整
**解决**：`rm -rf build` 后重来

---

## 6. 环境移植清单

在新电脑上按顺序执行：

```bash
# 1. 克隆 ESP-IDF
mkdir -p ~/esp && cd ~/esp
git clone --depth 1 --branch v5.3.2 https://github.com/espressif/esp-idf.git

# 2. 安装工具链
cd ~/esp/esp-idf && bash install.sh esp32s3

# 3. 配置镜像（国内必做）
git config --global url."https://jihulab.com/esp-mirror/espressif/esp-idf".insteadOf "https://github.com/espressif/esp-idf"
git config --global url."https://jihulab.com/esp-mirror/espressif/esp32-wifi-lib".insteadOf "https://github.com/espressif/esp32-wifi-lib.git"
git config --global url."https://jihulab.com/esp-mirror/kmackay/micro-ecc".insteadOf "https://github.com/kmackay/micro-ecc.git"

# 4. 更新子模块（跳过 micro-ecc）
git -c submodule."components/bootloader/subproject/components/micro-ecc/micro-ecc".update=none \
    submodule update --init --recursive

# 5. 修复 micro-ecc
rm -rf components/bootloader/subproject/components/micro-ecc/micro-ecc
git clone https://jihulab.com/esp-mirror/kmackay/micro-ecc \
    components/bootloader/subproject/components/micro-ecc/micro-ecc
cd components/bootloader/subproject/components/micro-ecc/micro-ecc
git checkout 24c60e243580c7868f4334a1ba3123481fe1aa48
cd ~/esp/esp-idf

# 6. 验证
source export.sh && xtensa-esp32s3-elf-gcc --version
```

> 注意：移植时 ESP-IDF 版本可能不同，micro-ecc 的精确 commit 需要从对应版本的 `.gitmodules` 重新获取。

---

## 7. ESP32-S3 N10P 桥接工程

### 当前状态

| 项目 | 路径 |
|------|------|
| 工程根目录 | `/home/ubuntu22/ROS2/n10p_leishen/esp32_n10p_bridge/` |
| 固件 | `build/esp32_n10p_bridge.bin` (237KB) |
| 分区表 | `build/partition_table/partition-table.bin` |
| bootloader | `build/bootloader/bootloader.bin` (21KB) |

### 快速命令

```bash
# 激活环境
source ~/esp/esp-idf/export.sh

# 编译
cd /home/ubuntu22/ROS2/n10p_leishen/esp32_n10p_bridge && idf.py build

# 烧录 + 监视
idf.py -p /dev/ttyUSB0 flash monitor
```
