# flix vs diy-drone（ESPDrone）：差异对比说明

> 本文面向当前工作区中的两个工程：`flix/` 与 `diy-drone/`，总结它们在定位、技术栈、架构、硬件假设与配套工具上的主要差异，并给出适合的应用场景与融合建议。

## 1. 项目定位与目标

### flix

- 定位：教育/研究向的开源四轴项目（flight + X），强调“从零做”和易读性。
- 目标：用尽量少、尽量清晰的固件代码把“能飞 + 能调 + 能看数据 + 能仿真”串起来。
- 配套：提供装配文档、使用说明、日志分析、Python 工具/库，以及 Gazebo 仿真环境。

### diy-drone（ESPDrone）

- 定位：更工程化的飞控固件工程，结构上接近 Crazyflie 系的模块化飞控框架。
- 目标：提供可扩展的飞控能力（多估计器、多控制器、任务/服务分层、可组合的传感器与外设），适合长期迭代。
- 配套：以 ESP-IDF 组件化组织驱动/平台/核心逻辑，并在仓库内积累大量“飞控实现与调参/驱动排障”文档。

## 2. 构建系统与开发体验

### flix

- 构建方式：Arduino 生态（arduino-cli），通过 Makefile 封装编译/上传/串口监视。
- 依赖管理：arduino-cli 安装 Arduino ESP32 core 与 Arduino 库（如 MAVLink 等）。
- 开发体验特征：
  - 上手快：一键 build/upload/monitor。
  - 代码入口清晰：典型 Arduino `setup()/loop()`。
  - 仿真/工具链与固件在同仓库：Gazebo、日志工具、Python 工具。

### diy-drone（ESPDrone）

- 构建方式：ESP-IDF（CMake + components）+ FreeRTOS。
- 目标芯片：当前配置为 ESP32S3（见 `diy-drone/sdkconfig` 的 target 配置）。
- 开发体验特征：
  - 适合复杂系统：多组件、更多 Kconfig/编译选项、调试与性能控制更细。
  - 模块化清晰：核心飞控、驱动、平台、配置、库分层明显。

## 3. 固件架构与代码组织

### flix：单主循环管线（学习成本低）

- 主流程是“固定顺序的管线式调用”，大体可概括为：
  1) 读 IMU
  2) 估计（姿态/状态）
  3) 控制（姿态/角速度/油门等）
  4) 输出电机
  5) 处理输入与通信（Wi‑Fi/MAVLink）
  6) 记录日志/同步参数
- 优点：调用链短、时序直观，适合教学、快速验证和小规模功能迭代。
- 代价：当系统复杂度上升（多传感器融合、多任务通信、多种控制模式并存）时，需要额外设计来保持可维护性。

### diy-drone：Crazyflie 风格模块化飞控（能力上限高）

- 以 component + module 方式组织，核心组件（`components/core/crazyflie`）包含：
  - 估计器：互补滤波、Kalman 相关核心与监管
  - 控制器：PID、Mellinger、INDI 等
  - 通信：CRTP 相关模块、comm/console 等
  - 系统：stabilizer/system/sysload/worker/trigger 等
  - 参数与日志：param/log/mem 等
  - HAL：传感器封装、wifilink、espnow 控制等
- 优点：边界清晰、可插拔能力强，适合不断增加外设/算法/通信方式。
- 代价：理解成本更高，需要熟悉模块关系、任务调度与配置系统。

## 4. 硬件假设与动力系统

这是两者“最影响能否直接复用代码”的差异点之一。

### flix

- README 描述的默认硬件方案更偏“小型有刷”路线：
  - 8520 3.7V 有刷电机
  - MOSFET 直驱
  - 目标是通用、易买、易装

### diy-drone（ESPDrone）

- 当前电机驱动代码体现的是“无刷 + ESC 标准 PWM”路线：
  - 400Hz PWM
  - 脉宽 1000–2000us
  - 使用 ESP-IDF 的 MCPWM 驱动
- 这意味着：两者在电机层（PWM 频率、输出方式、解锁/校准、安全策略）上并不能直接对拷，需要按硬件体系分别适配。

## 5. 传感器与外设覆盖面

### flix

- 面向“通用 IMU 小板”与教学场景，强调可替换与易上手。
- 通信与外部生态（Wi‑Fi/MAVLink、Python、仿真）是突出部分。

### diy-drone（ESPDrone）

- 外设组件化更丰富，驱动树中可见（示例）：
  - SPI：BMI088、PMW3901（光流）
  - I2C：VL53L0/VL53L1（测距）、MS5611（气压计）、HMC5883L（磁力计）、EEPROM、MPU6050 等
- 更偏“飞控平台化”：传感器融合/控制/上层服务可以围绕这些组件长期扩展。

## 6. 通信与工具生态

### flix

- Wi‑Fi + MAVLink 是核心亮点之一：更容易对接地面站/遥控/调参/遥测。
- 仿真（Gazebo）与日志工具链是“端到端体验”的重要部分：
  - 同仓库提供仿真构建
  - 提供日志采集/分析脚本与 Python 相关工具

### diy-drone（ESPDrone）

- 具备系统级通信模块（CRTP 体系等）以及 Wi‑Fi/ESP-NOW 相关实现。
- 工具生态主要沉淀为“工程文档与排障经验”：对硬件驱动、双核、同步机制、姿态解算与调参等有专门文档。
### 详细对比：MAVLink/Wi-Fi vs CRTP+WiFi

这是影响地面站对接、遥控方式、调参/日志体验的关键差异。

#### flix 的 MAVLink/Wi-Fi 方案

**实现特点**（见 `flix/flix/mavlink.ino` 和 `flix/flix/wifi.ino`）：
- 协议：MAVLink v2（标准航空/无人机通信协议），消息类型丰富且通用
- 传输层：UDP（端口 14550，广播或点对点）
- 网络模式：ESP32 开启 AP（SSID "flix"），地面站/遥控设备连上来
- 消息示例：
  - 心跳（HEARTBEAT）、姿态四元数（ATTITUDE_QUATERNION）
  - RC 通道（RC_CHANNELS_RAW）、执行器输出（ACTUATOR_CONTROL_TARGET）
  - 参数读写（PARAM_REQUEST_LIST/PARAM_SET 等）
  - 手动控制（MANUAL_CONTROL）
  - IMU 原始数据（SCALED_IMU）

**优势**：
1. **生态成熟**：可直接对接 QGroundControl、Mission Planner 等主流地面站，开箱即用。
2. **标准化**：参数、遥测、任务规划等协议都是业界标准，可与其他 MAVLink 设备互通。
3. **易学易用**：遥控 APP/地面站丰富，不需要定制客户端。
4. **社区资源多**：文档、工具、示例应用非常全面。

**劣势**：
1. **协议开销稍大**：MAVLink 消息有头部/校验，对于低速链路或需极致低延迟的场景相对"重"。
2. **不太适合微小设备**：虽然可用，但如果设备资源极度受限（如 8 位单片机）会稍显复杂。

**适用场景**：
- 需要对接主流地面站（QGC/MP）做飞行监视、调参、航点任务
- 希望借助 MAVLink 生态快速实现遥控/遥测功能
- 教学/演示时希望用户能用手机 APP/电脑软件直接连上观测数据

#### diy-drone 的 CRTP+WiFi 方案

**实现特点**（见 `diy-drone/components/core/crazyflie/modules/src/crtp.c`、`wifilink.c` 等）：
- 协议：CRTP（CrazyRealtime Transfer Protocol），Crazyflie 系飞控的私有轻量级协议
- 传输层：同样是 UDP over Wi-Fi（或可搭配 ESP-NOW 做更低延迟通道）
- 网络模式：ESP32 Wi-Fi，通过 `wifilink.c` 作为 CRTP 的链路层实现
- 包结构：header（端口+通道，1 字节）+ data（最多 30 字节）
- 端口示例（见 `crtp.h`）：
  - `CRTP_PORT_CONSOLE`（0x00）：调试/日志输出
  - `CRTP_PORT_PARAM`（0x02）：参数系统
  - `CRTP_PORT_SETPOINT`（0x03）：控制指令（姿态/速度/位置）
  - `CRTP_PORT_LOG`（0x05）：日志变量实时订阅
  - `CRTP_PORT_MEM`（0x04）：内存访问（如读取轨迹）
  - 其他高层控制、定位等端口

**优势**：
1. **协议轻量**：包头只 1 字节，最小化开销，适合高频率控制指令与低延迟场景。
2. **模块化强**：每个端口对应一个功能服务，内部可按 port 分发到不同模块/任务（见 CRTP 任务队列机制）。
3. **与 Crazyflie 体系兼容**：可复用 Crazyflie 的地面站客户端（cfclient）、Python 库、日志/参数系统设计。
4. **扩展灵活**：支持 ESP-NOW（低延迟遥控）+ Wi-Fi（数据/日志/参数）混合链路。

**劣势**：
1. **生态较封闭**：只能对接 Crazyflie 客户端或自己写客户端，不能直接用 QGC/MP。
2. **学习曲线**：需要理解 CRTP 端口/通道、参数系统、日志订阅机制，文档相对小众。
3. **调参/监视工具需定制**：如果想在手机 APP 或其他平台上监控，需要自己实现 CRTP 协议栈。

**适用场景**：
- 已经在 Crazyflie 生态内，或希望继承其模块化飞控架构
- 对延迟敏感（如需要高频率姿态控制指令、密集日志订阅）
- 希望构建高度定制化的地面站/调参工具（Python 脚本或自定义 APP）

#### 综合对比表

| 维度           | flix（MAVLink/Wi-Fi）                      | diy-drone（CRTP+WiFi）                       |
| -------------- | ------------------------------------------ | -------------------------------------------- |
| **协议类型**   | 航空业标准（MAVLink v2）                   | Crazyflie 私有轻量级协议                     |
| **地面站对接** | QGroundControl、Mission Planner 等开箱即用 | Crazyflie Client 或自己实现客户端            |
| **协议开销**   | 相对较大（消息头/CRC/类型字段等）          | 极小（1 字节头 + 数据，最多 31 字节/包）     |
| **学习曲线**   | 低（标准协议、文档/工具丰富）              | 中（需理解 CRTP 端口、参数/日志机制）        |
| **参数系统**   | 标准 MAVLink 参数协议                      | CRTP 参数端口 + TOC（Table of Contents）机制 |
| **日志/遥测**  | 标准消息类型，频率可配置                   | 日志订阅（按变量订阅，灵活高效）             |
| **遥控方式**   | MANUAL_CONTROL 消息 + RC_CHANNELS          | SETPOINT 端口 + 兼容旧协议/ESP-NOW           |
| **延迟敏感度** | 中（适合中等频率遥测/控制）                | 低（更适合高频控制与低延迟）                 |
| **生态开放性** | 高（可与任何 MAVLink 设备/工具互通）       | 低（Crazyflie 生态内循环）                   |
| **适合规模**   | 教学/演示/集成主流地面站                   | 工程化飞控开发/定制化深度集成                |

#### 结论与融合建议

**如果你更看重"开箱即用的地面站体验"**：
- flix 的 MAVLink 方案更友好，建议在 diy-drone 里也增加 MAVLink 支持作为"可选通信模式"。
- 具体做法：在 diy-drone 新增一个 MAVLink 组件，把姿态/传感器/参数映射到 MAVLink 消息，与现有 CRTP 并行工作。

**如果你更看重"轻量/低延迟/模块化"**：
- diy-drone 的 CRTP 方案更专业，保持现有架构；如需地面站体验可用 Crazyflie Client 或基于 Python 定制。

**最佳融合策略**：
- 在 diy-drone 中保留 CRTP 作为主通信栈（参数/日志/控制），同时增加一个"MAVLink 遥测桥"模块：
  - 周期性读取飞控状态并发送 MAVLink 遥测消息（HEARTBEAT、ATTITUDE_QUATERNION、SCALED_IMU 等）
  - 接收 MAVLink PARAM_REQUEST/SET，转发到 CRTP 参数系统
  - 接收 MANUAL_CONTROL，转换为 CRTP SETPOINT
- 这样既保留 CRTP 的模块化与轻量优势，又能对接主流地面站。
## 7. 各自更适合的使用场景

### 更适合选 flix 的情况

- 目标是教学/学习飞控基本链路：传感器→估计→控制→电机→通信。
- 希望快速搭建一个可飞/可仿真/可遥测的最小系统。
- 更看重“可用性与配套体验”（Makefile、仿真、Python、MAVLink）。

### 更适合选 diy-drone（ESPDrone）的情况

- 目标是做“可持续迭代”的飞控平台：更多传感器、更复杂控制模式、更强可维护性。
- 需要 ESP-IDF 级别的系统能力：多任务、精细时序控制、组件化驱动与配置管理。
- 希望在 Crazyflie 风格模块基础上继续扩展（估计器/控制器/通信/服务）。

## 8. 能否融合？可以，但建议以“抽取/迁移模块”的方式融合

两者可以互相借鉴，但不建议直接把两个工程硬合并（构建系统、实时模型、硬件假设差异太大）。更现实的融合路径：

1) 将 flix 的“外壳能力”迁入 diy-drone：
   - 优先迁移：MAVLink/Wi‑Fi 遥测与交互、日志/可视化工具链、仿真相关思路。
   - 好处：通常位于系统边缘，对飞控主环侵入小，收益大（调参、观测、地面站对接）。

2) 将 diy-drone 的“内核算法”迁入 flix（工作量更大）：
   - 只抽取“纯算法”部分（不带 FreeRTOS/驱动依赖），比如 Kalman 核心、控制律核心。
   - 由 flix 的 loop 负责调度与数据喂入，从而保持 flix 的易读框架。

3) 共用中立的基础库：
   - 四元数/向量、滤波器、PID、参数与日志抽象等可以整理成独立模块，降低两边重复实现成本。

## 9. 结论（一句话版）

- flix：更像“从零到能飞的教学型闭环系统”，强在易读与配套生态（仿真/日志/MAVLink/Python）。
- diy-drone：更像“可持续演进的工程化飞控平台”，强在模块化与能力上限（多估计器/控制器/传感器/服务）。
- 融合建议：优先把 flix 的 MAVLink/日志/仿真经验迁到 diy-drone；算法层再做有选择的抽取复用。
---

## 10. 深入：为什么 diy-drone 看起来十分繁杂，而 flix 看起来十分简洁？

这种感受差异的根源在于"单循环裸机式架构" vs "多任务 RTOS 架构"的设计哲学。

### flix 的简洁之道：压缩一切到 loop()

**代码示例**（见 `flix/flix/flix.ino`）：
```cpp
void loop() {
    readIMU();        // 读传感器（一个函数调用）
    estimate();       // 姿态估计（50 行互补滤波）
    control();        // 控制律（160 行 PID）
    sendMotors();     // 输出电机（直接写 PWM）
    processMavlink(); // 处理通信
    logData();        // 日志记录
}
```

**为什么一眼看到底**：
1. **无任务/队列/信号量**：所有逻辑在同一线程顺序执行，没有 FreeRTOS 的任务切换、优先级、队列传递。
2. **全局变量共享数据**：`attitude`、`rates`、`motors[]` 是全局的，任何函数都能直接读写。
3. **模块即函数文件**：每个功能就是几个函数（`estimate.ino` 只有 50 行）。
4. **单一实现**：只有一种估计器（互补滤波）、一种控制器（PID），不需要抽象层。

**代价**：要加 Kalman 估计器或 Mellinger 控制器时，需要大幅修改 loop 逻辑。

---

### diy-drone 的繁杂之源：多任务/模块化/抽象层

**系统启动流程**（见 `system.c`）：
```c
void systemTask(void *arg) {
    systemInit();      // 创建互斥锁、初始化 worker/LED/ADC/电源
    commInit();        // 启动 CRTP 收发任务 + 参数任务 + 日志任务
    commanderInit();   // 启动指令解析任务
    estimatorKalmanTaskInit(); // 启动独立的卡尔曼滤波任务（500Hz）
    stabilizerInit();  // 启动主控制循环任务（1000Hz）
    
    // 运行自检
    pass &= systemTest() & commTest() & stabilizerTest() & ...;
    
    // 释放启动信号量，让其他任务开始工作
    systemStart();
    
    // 进入 worker 队列循环
    workerLoop();
}
```

**典型任务列表**（部分）：
- `crtpTxTask` / `crtpRxTask`：通信收发
- `estimatorKalmanTask`：卡尔曼滤波（独立高频运行）
- `stabilizerTask`：主控制循环
- `paramTask`：参数系统
- `logTask`：日志订阅
- `wifilinkTask`：Wi-Fi 链路

**为什么看起来繁杂**：
1. **一条遥控指令的完整路径**：
   - `wifilinkTask` 从 UDP 收到数据 → 放入 CRTP 队列
   - `crtpRxTask` 从队列取出 → 按端口分发
   - `commanderTask` 解析控制指令 → 写入 `setpoint` 结构体
   - `stabilizerTask` 读取 `setpoint` → 调用控制器 → 输出电机
   - 每一步都涉及任务切换、队列拷贝、优先级调度

2. **多种算法可切换**：
   - 估计器：`estimator.c` 是抽象层，可切换 `complementaryEstimator` / `kalmanEstimator`
   - 控制器：`controller.c` 是抽象层，可切换 `controllerPID` / `controllerMellinger` / `controllerINDI`
   - 每种算法都是独立模块（`estimator_kalman.c` 有 700+ 行）

3. **参数/日志是完整子系统**：
   - `param.c`：动态参数表、CRC 校验、持久化（760 行）
   - `log.c`：变量订阅、打包发送（600+ 行）
   - `mem.c`：内存访问服务（400+ 行）

4. **大量监控与自检**：
   - 每个模块有 `xxxTest()` 函数
   - `queuemonitor.c`：监控所有队列深度
   - `rateSupervisor.c`：监控任务执行频率

**代码量对比**：

| 功能         | flix                      | diy-drone                                  |
| ------------ | ------------------------- | ------------------------------------------ |
| **估计器**   | 50 行（互补滤波）         | 700 行（Kalman）+ 抽象层                   |
| **控制器**   | 160 行（PID）             | 300 行（PID）+ 800 行（Mellinger）+ 抽象层 |
| **通信栈**   | 300 行（MAVLink + Wi-Fi） | 2500 行（CRTP + 参数 + 日志 + WiFi）       |
| **总模块数** | 15 个 `.ino` 文件         | 52 个 `.c` 模块（仅 modules/src）          |
| **代码总量** | < 2000 行                 | > 15000 行（仅 core/crazyflie）            |

---

### 总结表：简洁 vs 繁杂的根源

| 维度         | flix（简洁）    | diy-drone（繁杂）               |
| ------------ | --------------- | ------------------------------- |
| **核心模型** | 单循环顺序执行  | 多任务并发 + 队列通信           |
| **模块通信** | 全局变量        | FreeRTOS 队列 + 消息传递        |
| **算法组合** | 单一实现        | 多种可切换（需抽象层）          |
| **代码总量** | < 2000 行       | > 15000 行                      |
| **学习曲线** | 低（一眼看懂）  | 高（需理解 RTOS/队列/模块关系） |
| **"看起来"** | 像教科书示例    | 像成熟商业飞控                  |
| **适合人群** | 初学者/快速验证 | 有 RTOS 经验/长期迭代           |

**一句话总结**：
- flix 牺牲可扩展性换取易读性（教学友好）。
- diy-drone 牺牲初学友好度换取能力上限（工程化平台）。

---

## 11. flix 的核心亮点可以借鉴到 diy-drone 吗？

### 总结：flix 的两大核心亮点

根据前面分析，flix 最值得借鉴的亮点是：

1. **MAVLink + WiFi + QGroundControl（主流地面站生态）**
   - 开箱即用对接 QGC/Mission Planner
   - 标准化参数/遥测/遥控协议
   - 手机/电脑地面站丰富

2. **SBUS + 遥控器（传统遥控方式）**
   - 支持标准航模遥控器（如 BetaFPV LiteRadio + DF500 接收器）
   - 低延迟、可靠性高
   - 与 WiFi 遥控并存（见 `flix/flix/rc.ino`）

### 可行性分析：迁移到 diy-drone

| 功能                    | 迁移难度 | diy-drone 现状                                     | 迁移建议                                                                        |
| ----------------------- | -------- | -------------------------------------------------- | ------------------------------------------------------------------------------- |
| **SBUS + 遥控器**       | ★☆☆☆☆    | **已支持**（见 `CONFIG_ENABLE_SBUS` 与 `extrx.c`） | ✅ 已有，只需启用配置并测试                                                      |
| **MAVLink 通信协议**    | ★★★☆☆    | 无，当前只有 CRTP                                  | 需新增 MAVLink 组件，建议作为"遥测桥"与 CRTP 并行                               |
| **WiFi MAVLink 遥控**   | ★★☆☆☆    | WiFi 已有（`wifilink.c`），但承载 CRTP             | 在现有 WiFi 上新增 UDP MAVLink 通道（与 CRTP 共存）                             |
| **QGC 地面站对接**      | ★★★☆☆    | 无（只能用 Crazyflie Client）                      | 实现 MAVLink 遥测桥后自动支持                                                   |
| **参数系统对接 QGC**    | ★★★★☆    | 有完整的 CRTP 参数系统（`param.c`，760 行）        | 需写适配层：MAVLink PARAM_REQUEST/SET ↔ CRTP 参数系统                           |
| **日志/遥测发送到 QGC** | ★★★☆☆    | 有 CRTP 日志订阅系统（`log.c`，600+ 行）           | 需写适配层：周期读取姿态/传感器 → 打包为 MAVLink 消息（ATTITUDE/SCALED_IMU 等） |

### 具体迁移路线图

#### 阶段 1：验证 SBUS 遥控（最容易，立即可做）

**当前状态**：diy-drone 已有 SBUS 支持（`components/drivers/general/sbus/` 和 `extrx.c`）。

**操作步骤**：
1. 在 `menuconfig` 里启用 `CONFIG_ENABLE_SBUS`
2. 连接 SBUS 接收器到 ESP32 的 UART2（GPIO4，见系统初始化代码）
3. 用遥控器测试油门/俯仰/横滚/偏航控制

**收益**：获得传统航模遥控器的低延迟控制体验（与 flix 一致）。

---

#### 阶段 2：增加 MAVLink 遥测桥（中等难度，推荐优先级最高）

**目标**：在保留 CRTP 体系的同时，增加 MAVLink over WiFi/UDP 通道，让 QGC 能连上飞控。

**实现方案**（新增一个 `mavlink_bridge` 组件）：

1. **新增组件目录**：`diy-drone/components/mavlink_bridge/`
   - 依赖 MAVLink C 库（可从 flix 复用或用 ESP-IDF component）
   - 创建独立的 `mavlinkBridgeTask`（优先级低于主控制循环）

2. **核心功能**：
   ```c
   void mavlinkBridgeTask(void *arg) {
       while(1) {
           // 1. 发送遥测（10Hz）
           sendMavlinkHeartbeat();      // 心跳
           sendMavlinkAttitude();        // 从 state_t 读姿态 → ATTITUDE_QUATERNION
           sendMavlinkScaledIMU();       // 从 sensorData_t 读 IMU → SCALED_IMU
           sendMavlinkBattery();         // 从 pmGetBatteryVoltage() → SYS_STATUS
           
           // 2. 接收 MAVLink 命令
           if (receiveMavlinkPacket(&msg)) {
               handleMavlinkManualControl(&msg);  // MANUAL_CONTROL → setpoint
               handleMavlinkParam(&msg);          // PARAM_REQUEST/SET → param.c
           }
           
           vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz
       }
   }
   ```

3. **UDP 传输层**：
   - 复用现有 WiFi（`wifi_esp32.c`）
   - 新增 UDP socket（端口 14550，与 flix 一致）
   - 与 CRTP 的 UDP 端口分开（CRTP 可能用其他端口）

4. **参数对接**（适配层）：
   ```c
   void handleMavlinkParamRequest(mavlink_message_t *msg) {
       // MAVLink PARAM_REQUEST_LIST → 遍历 CRTP 参数表
       int count = paramGetCount(); // 从 param.c
       for (int i = 0; i < count; i++) {
           char name[16];
           float value;
           paramGetNameValue(i, name, &value);
           sendMavlinkParamValue(name, value, i, count);
       }
   }
   ```

**收益**：
- ✅ QGC 能连上飞控，查看姿态/传感器/电池
- ✅ QGC 能调参（读写参数）
- ✅ QGC 的虚拟摇杆能控制飞行
- ⚠️ 不影响现有 CRTP 体系（Crazyflie Client 仍可用）

---

#### 阶段 3：完整 MAVLink 功能（可选，长期演进）

更高级的 MAVLink 功能（如果需要）：
- 任务规划（MISSION_ITEM）→ 需对接 `planner.c` / `pptraj.c`
- 传感器校准（MAG_CAL/GYRO_CAL）→ 需对接传感器校准流程
- 数据流订阅（REQUEST_DATA_STREAM）→ 按需发送高频遥测

---

### 迁移优先级建议

| 优先级 | 功能                   | 难度 | 收益           | 建议时机               |
| ------ | ---------------------- | ---- | -------------- | ---------------------- |
| **P0** | 启用 SBUS 遥控         | 极低 | 立即提升操控感 | 现在就做               |
| **P1** | MAVLink 遥测桥（核心） | 中等 | 对接 QGC       | 优先做（1-2 周工作量） |
| **P2** | MAVLink 参数对接       | 中高 | QGC 调参       | 遥测桥稳定后           |
| **P3** | MAVLink 任务规划       | 高   | 航点飞行       | 长期演进               |

---

### 总结：借鉴 flix 的最佳实践

**确定可以借鉴的**：
1. ✅ **SBUS 遥控**：diy-drone 已支持，只需启用配置
2. ✅ **MAVLink 遥测桥**：可以新增组件实现，与 CRTP 并行工作

**不建议完全替换**：
- ❌ 不要废弃 CRTP 改用 MAVLink：CRTP 的轻量/模块化优势会丢失
- ❌ 不要把 flix 的单循环架构搬过来：会破坏 diy-drone 的多任务体系

**最佳融合策略**：
- 保留 diy-drone 的 CRTP + 多任务内核
- 新增 MAVLink 作为"可选的遥测/地面站通道"
- 两者共存：开发者用 Crazyflie Client 深度调试，演示/教学时用 QGC 展示

**实际效果**（融合后）：
- 遥控方式：SBUS 遥控器（低延迟）+ WiFi MAVLink 虚拟摇杆（备用）
- 地面站：Crazyflie Client（开发调试）+ QGroundControl（演示/监控）
- 参数/日志：CRTP 订阅（高效）+ MAVLink 遥测（兼容主流工具）