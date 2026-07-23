# 任务二：桌面小云台视觉跟踪系统

> Aurora 27 赛季视觉组暑训考核 — 任务二 方向A：自瞄——小云台从零重构

## 项目概述

从零构建的桌面小云台视觉跟踪系统，实现对自定义移动目标（装甲板）的实时检测与跟踪。核心链路：**相机采集 → YOLOv8 目标检测 → PnP 位姿解算 → 跟踪器（EMA + 速度前馈）→ 串口控制云台**。

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                      主线程 (~33fps)                     │
│                                                         │
│  相机 (大恒 MER-139)                                     │
│    │                                                    │
│    ▼                                                    │
│  YOLOv8 (OpenVINO CPU推理)                               │
│    │ 输出: 4个角点像素坐标                                │
│    ▼                                                    │
│  PnP 解算 (solvePnP IPPE)                               │
│    │ 输出: 3D位置 + yaw/pitch角度                        │
│    ▼                                                    │
│  SimpleTracker (EMA平滑 + 状态机)                        │
│    │ 输出: 平滑后的目标角度 + 角速度                      │
│    ▼                                                    │
│  控制指令生成                                            │
│    │ cmd = 当前云台角 + P增益×偏角 + 速度前馈             │
│    ▼                                                    │
│  串口发送 (VisionToGimbal, 29字节)                       │
│    │                                                    │
│    └──────────▶ STM32 (云台下位机)                       │
│                      │                                  │
│  串口接收 ◀──────────┘                                  │
│  (GimbalToVision, 43字节)                                │
│    当前yaw/pitch角度、四元数                              │
│                                                         │
│  可视化窗口: 实时显示检测结果 + 云台角度                   │
└─────────────────────────────────────────────────────────┘
```

## 文件结构

```
small_gimbal_tracker/
├── src/
│   └── main.cpp                 ★ 主程序：全部控制逻辑在这里
├── configs/
│   └── gimbal.yaml              ★ 配置文件（检测参数、串口、标定参数）
├── io/                          ── 硬件抽象层 ──
│   ├── camera.cpp/hpp           相机驱动（大恒Galaxy SDK）
│   ├── gimbal/gimbal.cpp/hpp    串口通信（Gimbal/CBoard双协议）
│   ├── galaxy/                  大恒相机SDK头文件
│   └── serial/                  C++串口库
├── tasks/auto_aim/              ── 视觉算法层 ──
│   ├── armor.cpp/hpp            装甲板数据结构（坐标、角度、类型）
│   ├── solver.cpp/hpp           ★ PnP 位姿解算 + yaw角优化 + 重投影
│   ├── yolo.cpp/hpp             YOLO 检测接口
│   ├── yolos/yolov8.cpp/hpp     YOLOv8 (OpenVINO) 实现
│   ├── detector.cpp/hpp         传统CV检测（备用）
│   └── classifier.cpp/hpp       装甲板分类器（数字识别）
├── tools/                       ── 工具层 ──
│   ├── math_tools.cpp/hpp       数学工具（欧拉角、atan2、限幅）
│   ├── logger.cpp/hpp           日志系统（spdlog）
│   ├── img_tools.cpp/hpp        图像绘制工具
│   ├── plot_logger.hpp          PlotJuggler数据输出（已关闭）
│   ├── yaml.hpp                 YAML配置解析
│   ├── thread_safe_queue.hpp    线程安全队列
│   └── exiter.cpp/hpp           Ctrl+C优雅退出
└── assets/
    └── best.bin                 OpenVINO模型权重（YOLOv8）
```

## 核心模块详解

### 1. 主控制循环 (`src/main.cpp`)

| 组件 | 类/函数 | 功能 |
|---|---|---|
| 跟踪器 | `SimpleTracker` | 三状态状态机 (LOST → DETECTING → TRACKING)，EMA平滑滤波 |
| 控制律 | 内联代码 | `cmd = gs + GAIN × target + FF × vel`，死区 + 限幅 |
| 测试模式 | `TEST_MODE` | 三角波缓慢扫描（±35°, 8秒半周期），用于验证串口通信 |
| 可视化 | OpenCV窗口 | 显示检测框、云台角度、跟踪状态、控制信息 |

**跟踪器状态机：**
```
LOST ──检测到目标──▶ DETECTING ──连续2帧确认──▶ TRACKING
  ▲                      │                          │
  │                      └──连续8帧丢失──▶ LOST      │
  │                                                 │
  └──────────连续50帧丢失────────────────────────────┘
                                          TRACKING丢失时 → EXTRAP外推
```

**控制参数：**
| 参数 | 值 | 说明 |
|---|---|---|
| YAW_GAIN | 0.28 | yaw P增益 |
| PITCH_GAIN | 0.55 | pitch P增益 |
| YAW_FF / PITCH_FF | 0.03 | 速度前馈系数 |
| FF_RAMP | 5° | 前馈渐变区间（接近中心时前馈线性减弱） |
| DEAD_ZONE_YAW | ±2.5° | yaw死区 |
| DEAD_ZONE_PITCH | ±0.5° | pitch死区 |
| EMA_ALPHA | 0.25 | 平滑系数 |
| YAW_LIMIT | ±35° | yaw软限位 |
| PITCH_LIMIT | ±35° | pitch软限位 |

### 2. 串口通信 (`io/gimbal/gimbal.cpp`)

| 属性 | 说明 |
|---|---|
| 协议 | Gimbal（帧头 5A A5，帧尾 7F FE）/ CBoard（帧头 FF，帧尾 0D） |
| 发送帧 | VisionToGimbal，29字节（mode + yaw/pitch绝对角度 + 角速度/角加速度） |
| 接收帧 | GimbalToVision，43字节（mode + 四元数 + yaw/pitch角度/角速度） |
| 角度单位 | 上位机内部 rad，发送/接收时转换为 deg |
| 线程 | 独立发送线程（背压机制，只发最新控制量）+ 独立接收线程 |

### 3. PnP 位姿解算 (`tasks/auto_aim/solver.cpp`)

```
像素坐标(2D) → solvePnP(IPPE) → 相机坐标(3D)
  → [手眼矩阵] → 云台坐标(3D)
  → [云台姿态四元数] → 世界坐标(3D)
  → atan2 → yaw/pitch角度
```

- `solvePnP` 使用 IPPE 算法（适合平面目标）
- yaw 角通过重投影误差搜索优化（±70° 范围，1° 步长）

### 4. YOLOv8 检测 (`tasks/auto_aim/yolos/yolov8.cpp`)

- 模型格式：OpenVINO IR（.bin + .xml）
- 推理设备：CPU
- ROI 区域检测（启用后减少背景干扰）
- 最小置信度：0.5

## 编译与运行

### 环境要求
- Ubuntu 24.04
- C++17, CMake ≥ 3.16
- OpenCV ≥ 4.x
- OpenVINO 2024+
- 依赖：spdlog, fmt, Eigen3, yaml-cpp, serial

### 安装依赖
```bash
sudo apt install -y cmake g++ libopencv-dev libeigen3-dev \
    libspdlog-dev libfmt-dev libyaml-cpp-dev
```

### 编译
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
make -C build -j$(nproc)
```

### 运行
```bash
./build/small_gimbal
```

## 配置文件说明 (`configs/gimbal.yaml`)

| 参数 | 类型 | 说明 |
|---|---|---|
| `com_port` | string | 串口设备路径（如 /dev/ttyUSB0） |
| `protocol` | string | 通信协议（gimbal / cboard） |
| `device` | string | 推理设备（CPU / GPU） |
| `min_confidence` | float | YOLO检测最小置信度 |
| `use_roi` | bool | 是否启用ROI区域检测 |
| `camera_matrix` | float[9] | 相机内参矩阵（3×3） |
| `distort_coeffs` | float[5] | 畸变系数 |
| `R_camera2gimbal` | float[9] | 手眼标定旋转矩阵 |
| `t_camera2gimbal` | float[3] | 手眼标定平移向量 |

## 开发过程中的关键问题与解决

1. **方向反**：YAW_DIR/PIT_DIR 方向系数错误 → 通过三级诊断（像素→PnP→cmd）逐级验证方向一致性
2. **抖动剧烈**：控制增益过高（1.5），死区过小（0.3°）→ 增益降至0.28/0.55，死区增至2.5°/0.5°
3. **外推漂移**：丢失目标后外推无限制 → 添加角度限幅（yaw ±45°, pitch ±30°）+ 衰减系数
4. **慢速目标超调**：前馈持续推动靠近中心的目标 → 引入FF_RAMP渐变缩放（偏角<5°时前馈线性减弱）
5. **YOLO误检**：min_confidence过低（0.3），未启用ROI → 阈值提至0.5，开启ROI区域检测
6. **速度噪声放大**：前馈直接使用瞬时速度噪声大 → 对目标角速度做EMA平滑后再用于前馈
7. **串口单向通信**：USB-TTL模块TX引脚接触不良 → 更换杜邦线

## 考核相关

- **考核任务**：任务二 方向A — 自瞄：小云台从零重构（100分）
- **核心考察**：从零构建完整视觉闭环（检测→解算→控制），跟踪响应速度
- **截止时间**：2026年7月25日
