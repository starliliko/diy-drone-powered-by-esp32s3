# ESP-Drone Firmware (ESP32-S3)

> 基于 [Bitcraze Crazyflie](https://github.com/bitcraze/crazyflie-firmware) + [Espressif ESP-Drone](https://github.com/espressif/esp-drone) 的二次开发 DIY 飞控固件。
> 目标硬件:**ESP32-S3** + BMI088 (SPI) + MS5611 (I2C) + QMC5883P (I2C) + MTF01 (UART/光流)。

---

## 1. 工具链

| 项目 | 版本 |
| --- | --- |
| ESP-IDF | **5.5.1** |
| 目标芯片 | esp32s3 |
| 编译框架 | CMake + Ninja |
| 调试器 | ESP-PROG / 内置 USB-JTAG |

确认环境:

```powershell
idf.py --version           # 应输出 v5.5.1
echo $env:IDF_TARGET        # 应为 esp32s3 (或留空让 sdkconfig 决定)
```

---

## 2. 构建 / 烧录 / 监控

```powershell
# 配置(首次必做,选 STA SSID/密码、地面站 IP)
idf.py menuconfig
#   → diy-drone config → wireless config
#   → Component config → ESP-Drone Config

# 构建
idf.py build

# 烧录 + 监控(替换 COM 口)
idf.py -p COM3 flash monitor
#   退出 monitor: Ctrl + ]

# 单独烧录 / 单独监控
idf.py -p COM3 flash
idf.py -p COM3 monitor

# 清理
idf.py fullclean
```

---

## 3. 目录结构

```text
firmware/
├── main/                       # 应用入口
│   └── main.c                  #   app_main(): NVS → platformInit → BMI088 自检
├── components/
│   ├── config/                 # 全局配置头(任务优先级、栈、GPIO)
│   ├── platform/               # 硬件平台抽象 (EP20 / EP30 等型号)
│   ├── drivers/                # 驱动层
│   │   ├── general/            #   电机 PWM / SBUS / WiFi / MTF01 光流 / LED / 蜂鸣器 / ADC
│   │   ├── i2c_bus/            #   I2C 总线
│   │   ├── i2c_devices/        #   MS5611 气压计 / QMC5883P 磁力计 / EEPROM
│   │   └── spi_devices/        #   BMI088 IMU
│   ├── core/crazyflie/         # 飞控核心(Crazyflie 框架)
│   │   ├── hal/                #   传感器统一抽象 (sensors_bmi088_spi_ms5611_qmc5883p.c)
│   │   ├── modules/            #   stabilizer / commander / estimator / controller / ...
│   │   └── utils/              #   工具函数
│   └── lib/dsp_lib/            # Xtensa DSP 库(矩阵 / PID / 滤波)
├── docs/                       # 中文设计与调试文档(详见 docs/README.md)
├── partitions.csv              # 分区表 (factory 2 MB + storage 1 MB)
├── sdkconfig.defaults          # 默认配置
└── CMakeLists.txt
```

---

## 4. 关键任务

| 任务 | 文件 | 频率 / 优先级 | 说明 |
| --- | --- | --- | --- |
| `sensorsTask` | `core/crazyflie/hal/src/sensors_bmi088_spi_ms5611.c` | 1 kHz / 6 | BMI088 中断驱动,IMU 读取 + 磁力计 50 Hz 分频 + 倾斜补偿 yaw |
| `stabilizerTask` | `modules/src/stabilizer.c` | 1 kHz / 7 | 主稳定器循环:估计器 → 控制器 → 电机分配 |
| `estimatorKalmanTask` | `modules/src/estimator_kalman_task.c` | 500 Hz 预测 / 4 | EKF;TOF 主观测,Baro 可选(`KALMAN_USE_BARO_UPDATE`) |
| `commanderTask` | `modules/src/commander.c` | 事件 / 3 | 多源 setpoint 聚合(SBUS > TCP > CRTP) |
| `extRxTask` | `modules/src/extrx.c` | UART / 2 | SBUS 解析 |
| `remoteServerTx/Rx` | `modules/src/remote_server.c` | 100 Hz 上行 / 2 | MAVLink v2 + 私有 0xABCD 双栈,WiFi/TCP+UDP |
| `crtpRx/Tx` | `modules/src/crtp.c` | 事件 / 2 | CRTP(WiFi UDP / cfclient) |
| `param/log` | `modules/src/{param,log}.c` | 事件 / 2 | 参数与日志框架(注册/查询/订阅) |

详见 [docs/双核部分.md](docs/双核部分.md) 与 [docs/无人机控制流程.md](docs/无人机控制流程.md)。

---

## 5. 状态估计与控制

- **姿态解算**:`sensfusion6.c`(陀螺+加速度互补滤波)+ 磁力计倾斜补偿 yaw。
- **位置/高度估计**:`estimator_kalman.c` + `kalman_core.c`,21 维状态向量。
  - 高度主观测:**TOF (MTF01)**,默认开启
  - Baro:代码已就位,默认未编译开启
  - 详见 [docs/卡尔曼估计器详解.md](docs/卡尔曼估计器详解.md) / [docs/飞控高度估计总结与项目现状.md](docs/飞控高度估计总结与项目现状.md)
- **姿态控制**:`controller_pid.c`(默认)/ `controller_indi.c` / `controller_mellinger.c`,通过 `STABILIZER_CONTROLLER_TYPE` 切换。
- **电机分配**:`power_distribution_stock.c`(X 型四旋翼)。

---

## 6. 通信链路

```text
                       ┌─────────────────────┐
                       │      ESP32-S3       │
                       └─────────┬───────────┘
                                 │
   ┌─────────────────────────────┼────────────────────────┐
   │                             │                        │
   ▼ AP (192.168.43.42)          ▼ STA (路由器)            ▼ UART
 cfclient / 手机 APP        Ground Station (本仓 GS)    SBUS 接收机
 (CRTP over UDP)            ├─ TCP 8080  MAVLink/0xABCD  (extrx.c)
                            └─ UDP 8082  ESPU 高频遥测
                            └─ UDP 8081  ESPD 自动发现
```

- 地面站协议:固件 `remote_server.c` 已实现 **MAVLink v2 双栈**(HEARTBEAT / SYS_STATUS / ATTITUDE / SERVO_OUTPUT_RAW / LOCAL_POSITION_NED / HIGHRES_IMU / PARAM_SET / MANUAL_CONTROL / COMMAND_LONG / COMMAND_ACK)。
- 长参数键(>16 B)使用别名:`pa_*` ↔ `pid_attitude.*`,`pp_*` ↔ `posCtlPid.*`。
- 详见 [docs/远程服务器通信配置指南.md](docs/远程服务器通信配置指南.md) / [docs/MAVLink重构方案.md](docs/MAVLink重构方案.md) / [docs/通信部分.md](docs/通信部分.md)。

---

## 7. 配置项 (`menuconfig`)

主要分组:

```
diy-drone config
 ├── platform                  机型(EP20/EP30)、传感器组合
 └── wireless config
     ├── STA Mode & Remote Server   STA SSID/密码、远程服务器 IP/端口
     └── Remote Server Settings     心跳/遥测频率、重连间隔

Component config
 └── ESP-Drone Config
     ├── Stabilizer            控制器类型、控制频率
     ├── Estimator             EKF 开关、Baro 更新开关
     └── Debug                 日志级别、模块过滤
```

参数(运行时,通过 `param.c` 注册):见 [docs/程序配置结构.md](docs/程序配置结构.md)。

---

## 8. 分区表

[`partitions.csv`](partitions.csv):

| 名称 | 类型 | 大小 | 用途 |
| --- | --- | --- | --- |
| nvs | data, nvs | 24 KB | NVS(WiFi/参数) |
| phy_init | data, phy | 4 KB | RF 校准 |
| factory | app, factory | 2 MB | 固件镜像 |
| storage | data, spiffs | 1 MB | 可选文件系统 |

---

## 9. 调试

- **日志**:`idf.py monitor`,模块前缀如 `[STAB] [SENSORS] [EKF] [REMOTE] [PARAM]`。
- **JTAG**:见 [docs/ESP32S3_VSCode调试指南.md](docs/ESP32S3_VSCode调试指南.md)。
- **PID 实时调参**:用配套地面站 [`groundstation/`](../groundstation/),PID 写入持久化 + 上电自动下发。
- **磁力计校准**:地面站 🧭 标签页,3D 点云采集后自动写 `mag.offX/Y/Z` + `mag.scaleX/Y/Z`。
- **VOFA+ 频谱**:`groundstation` 转发 13 通道 FireWater UDP 8083,详见 [docs/频谱分析与滤波调参指南.md](docs/频谱分析与滤波调参指南.md)。

---

## 10. 文档

中文设计与调试文档全在 [docs/](docs/) 下,导航见 [docs/README.md](docs/README.md)。

---

## 11. 配套地面站

详见 [`../groundstation/`](../groundstation/)(Node.js,启动 `npm start` → <http://localhost:3000>)。

---

## 12. 许可

GPL-3.0(来自上游 Crazyflie 与 Espressif ESP-Drone)。
