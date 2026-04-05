# Software — 软件系统

> 飞控固件与地面站，构成 DIY-Drone 的完整软件链路。

## 目录结构

```text
software/
├── firmware/          # 飞控固件（ESP-IDF 工程）
└── groundstation/     # 地面站（Node.js Web 应用）
```

## 飞控固件 — `firmware/`

基于 ESP-IDF 5.x 构建，运行在 ESP32-S3 上。

| 模块 | 说明 |
|------|------|
| 传感器驱动 | BMI088（SPI）、气压计（I2C）、光流（UART）中断采集与滤波 |
| 状态估计 | 扩展卡尔曼滤波器，融合 IMU / 气压 / 光流 |
| 控制链路 | PID 级联：位置 → 速度 → 姿态 → 电机输出 |
| 飞行模式 | 姿态稳定 · 定高 · 定点 |
| 安全机制 | 遥控失联降级 · 控制源超时切换 · 解锁保护 · 紧急停机 |
| 通信 | CRTP/WiFi · SBUS 遥控 · TCP 远程服务器 |

### 快速构建

```bash
cd software/firmware
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

> 需要 ESP-IDF 5.x 环境。Windows 下可通过 ESP-IDF PowerShell 或 VS Code ESP-IDF 扩展进入环境。

## 地面站 — `groundstation/`

基于 Node.js 的 Web 地面站，提供遥测可视化与远程控制能力。

| 功能 | 说明 |
|------|------|
| 遥测仪表盘 | 姿态角 · 位置 · 速度 · 电池状态 · 飞行模式 |
| 3D 可视化 | Three.js 实时姿态渲染 |
| 飞行控制 | 解锁 / 上锁 · 起飞 / 降落 · 悬停 · 紧急停止 |
| 电机测试 | 独立电机推力测试工具 |

### 快速启动

```bash
cd software/groundstation
npm install
npm start
# 浏览器访问 http://localhost:3000
```

## 开发建议

- 飞控开发：以 `software/firmware/` 为工作区，使用 ESP-IDF 工具链
- 地面站开发：进入 `software/groundstation/`，使用 `npm run dev` 热重载开发
