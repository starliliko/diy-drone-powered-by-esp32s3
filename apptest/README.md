# ESP-Drone QGroundControl Lite - 地面站

这是一个基于 Node.js 的模块化地面站系统，用于接收、可视化和控制 ESP-Drone 飞控。采用前后端分离的现代化架构。

## 功能特性

✅ **TCP 服务器** - 接收飞控发送的遥测数据（端口 8080）  
✅ **Web 可视化界面** - 实时显示飞行状态和3D姿态（端口 3000）  
✅ **WebSocket 推送** - 实时双向数据流传输  
✅ **飞行控制** - 解锁、起飞、降落、返航等控制命令  
✅ **电机测试** - 独立电机推力测试工具  
✅ **模块化架构** - 前后端分离，易于维护和扩展

## 项目结构

```
apptest/
├── server.js                  # 服务器入口文件
├── package.json               # 项目配置
├── src/                       # 后端模块
│   ├── config.js             # 服务器配置 (端口、协议参数)
│   ├── protocol.js           # 协议定义 (PacketType、FlightMode等)
│   ├── parser.js             # 数据包解析与创建
│   ├── DroneClient.js        # TCP客户端连接管理
│   └── DroneServer.js        # 主服务器类 (TCP/HTTP/WebSocket)
└── public/                    # 前端静态文件
    ├── index.html            # HTML模板
    ├── css/
    │   └── style.css         # QGC风格样式表
    └── js/
        ├── app.js            # 主入口 (ES6模块)
        ├── visualization.js  # Three.js 3D飞机模型
        ├── websocket.js      # WebSocket连接管理
        ├── control.js        # 飞行控制命令
        ├── motorTest.js      # 电机测试功能
        └── dashboard.js      # 实时遥测数据仪表盘
```

## 快速开始

### 1. 安装依赖

```bash
npm install
```

### 2. 启动服务器

```bash
npm start
```

服务器将在以下端口启动：
- **TCP 端口**: 8080 (飞控连接)
- **HTTP/WebSocket 端口**: 3000 (浏览器访问)

输出示例：
```
╔════════════════════════════════════════════════════════════╗
║         ESP-Drone QGroundControl Lite 地面站               ║
╠════════════════════════════════════════════════════════════╣
║  HTTP 服务:    http://localhost:3000                      ║
║  WebSocket:    ws://localhost:3000                        ║
║  TCP 端口:     8080 (等待无人机连接)                      ║
╚════════════════════════════════════════════════════════════╝
```

### 3. 打开监控界面

在浏览器中访问：http://localhost:3000

## 飞控端配置

### 方法 1: 通过 menuconfig 配置

```bash
idf.py menuconfig
```

进入 `Component config -> ESP-Drone Config -> Remote Server Settings`

设置：
- Remote Server Enable: ✓
- Server IP: 你的电脑 IP 地址（见下方如何获取）
- Server Port: 8080

**获取电脑 IP 地址：**

在 PowerShell 中运行：
```powershell
ipconfig
```

或更简洁地获取 IPv4 地址：
```powershell
Get-NetIPAddress -AddressFamily IPv4 | Where-Object {$_.InterfaceAlias -notlike '*Loopback*'} | Select-Object IPAddress, InterfaceAlias
```

选择与 ESP32 在同一网络的 IP 地址（通常是 WLAN 或以太网接口）

### 方法 2: 直接修改代码

在飞控代码中找到远程服务器配置部分，设置：

```c
#define REMOTE_SERVER_ENABLE 1
#define REMOTE_SERVER_IP "192.168.x.x"  // 你的电脑 IP
#define REMOTE_SERVER_PORT 8080
```

## 数据协议

### 包结构

```
包头 (8 bytes):
+-------+-------+---------+------+-----+--------+
| Magic | Magic | Version | Type | Seq | Length |
| 0xAB  | 0xCD  |  0x01   | 1B   | 2B  |   2B   |
+-------+-------+---------+------+-----+--------+

包体 (可变长度):
根据 Type 类型不同而不同
```

### 包类型

| Type | 名称      | 说明        |
| ---- | --------- | ----------- |
| 0x00 | HEARTBEAT | 心跳包      |
| 0x01 | TELEMETRY | 遥测数据    |
| 0x02 | CONTROL   | 控制指令    |
| 0x03 | CRTP      | CRTP 协议包 |
| 0x04 | ACK       | 应答包      |
| 0x05 | CONFIG    | 配置包      |
| 0x06 | LOG       | 日志消息    |

### 遥测数据结构 (44 bytes)

```c
struct TelemetryData {
    int16_t roll;           // 横滚角 (度 * 100)
    int16_t pitch;          // 俯仰角 (度 * 100)
    int16_t yaw;            // 航向角 (度 * 100)
    int16_t gyro_x;         // 陀螺仪 X (度/秒 * 10)
    int16_t gyro_y;         // 陀螺仪 Y
    int16_t gyro_z;         // 陀螺仪 Z
    int16_t acc_x;          // 加速度 X (mg)
    int16_t acc_y;          // 加速度 Y
    int16_t acc_z;          // 加速度 Z
    int32_t pos_x;          // 位置 X (米 * 1000)
    int32_t pos_y;          // 位置 Y
    int32_t pos_z;          // 位置 Z
    int16_t vel_x;          // 速度 X (米/秒 * 1000)
    int16_t vel_y;          // 速度 Y
    int16_t vel_z;          // 速度 Z
    uint16_t batt_voltage;  // 电池电压 (伏特 * 1000)
    uint8_t batt_percent;   // 电量百分比
    uint8_t flight_mode;    // 飞行模式
    uint8_t is_armed;       // 解锁状态
    uint8_t is_low_battery; // 低电量标志
    uint8_t reserved[2];    // 保留
    uint32_t timestamp;     // 时间戳 (ms)
};
```

## API 接口

### GET /api/status

获取服务器和连接状态

**响应示例:**
```json
{
  "timestamp": 1737449400000,
  "clients": [
    {
      "addr": "192.168.1.100:12345",
      "connectTime": 1737449300000,
      "rxCount": 1234,
      "txCount": 567,
      "lastHeartbeat": 1737449399000,
      "telemetry": { ... }
    }
  ],
  "webClients": 2
}
```

### POST /api/control

发送控制指令到飞控

**请求参数:**
```json
{
  "cmdType": 0,    // 控制类型
  "roll": 0.0,     // 横滚角 (度)
  "pitch": 0.0,    // 俯仰角 (度)
  "yaw": 0.0,      // 偏航速度 (度/秒)
  "thrust": 0,     // 油门 (0-65535)
  "mode": 0        // 模式
}
```

**控制类型:**
- 0: RPYT (Roll/Pitch/Yaw/Thrust)
- 1: VELOCITY (速度控制)
- 2: POSITION (位置控制)
- 3: HOVER (悬停)
- 4: LAND (降落)
- 5: EMERGENCY (紧急停止)
- 6: ARM (解锁)
- 7: DISARM (上锁)

## Web 界面功能

### 实时数据显示

- ✈️ **姿态角**: Roll/Pitch/Yaw
- 🔋 **电池状态**: 电压、电量百分比、低电量警告
- 📍 **位置速度**: 3D 位置和速度
- 🔧 **飞行模式**: DISABLED/STABILIZE/ALTITUDE_HOLD/POSITION_HOLD
- 🔓 **解锁状态**: 已解锁/已上锁

### 快捷控制按钮

- 🔓 **解锁** - ARM 命令
- 🔒 **上锁** - DISARM 命令
- 🛑 **悬停** - HOVER 命令
- ⚠️ **紧急停止** - EMERGENCY 命令
- 🛬 **降落** - LAND 命令

## 控制来源自动切换机制

### 控制优先级系统

飞控支持多控制源同时接入，采用优先级机制自动管理：

| 优先级   | 控制来源   | 图标 | 说明                   |
| -------- | ---------- | ---- | ---------------------- |
| 3 (最高) | SBUS遥控器 | 🎮    | 传统遥控器，最高优先级 |
| 2        | TCP地面站  | 🖥️    | 远程服务器控制         |
| 1        | CRTP/WiFi  | 📶    | 手机APP/UDP控制        |
| 0        | 无控制     | ⚪    | 无活跃控制源           |

### 自动切换规则

1. **高优先级优先**：当多个控制源同时活跃时，采用最高优先级的控制源
2. **超时降级**：控制源超过 1 秒无数据时，自动切换到下一可用控制源
3. **自动恢复**：超时的控制源恢复后，可重新获取控制权（需等待锁定时间）

### 状态显示

Web 界面顶部状态栏显示：
- 🎮 **RC** - 遥控器连接状态（绿色=已连接，红色=断开）
- 🖥️ **GCS** - 地面站连接状态
- 当前活跃控制源带有颜色标识

### 故障转移日志

控制源切换时，系统日志会显示：
```
控制源切换: 遥控器 → 地面站 (TCP远程服务器)
```

## 故障排除

### 飞控连接不上

1. 检查电脑防火墙是否允许 TCP 端口 8080
2. 确认飞控和电脑在同一网络
3. 检查飞控配置中的服务器 IP 是否正确
4. 查看飞控串口输出，确认是否有连接错误信息

### Web 界面无数据

1. 检查浏览器控制台是否有错误
2. 确认 WebSocket 连接状态（界面左上角状态指示器）
3. 重新加载页面
4. 检查服务器日志输出

### 控制指令无响应

1. 确认飞控已连接（TCP 连接建立）
2. 检查飞控是否支持该控制模式
3. 查看服务器控制台日志
4. 确认飞控端接收到控制包

## 开发说明

### 模块化架构

项目采用前后端分离、多层模块化设计：

**后端模块 (src/)**
- `config.js` - 全局配置常量
- `protocol.js` - 数据包协议定义
- `parser.js` - 数据包编解码
- `DroneClient.js` - 单个飞控客户端连接管理
- `DroneServer.js` - 主服务器（TCP/HTTP/WebSocket）

**前端模块 (public/js/)**
- `app.js` - 主入口和全局函数
- `visualization.js` - Three.js 3D 可视化
- `websocket.js` - WebSocket 连接管理
- `control.js` - 飞行控制命令
- `motorTest.js` - 电机测试功能
- `dashboard.js` - 遥测数据更新

### 修改配置

编辑 `src/config.js`：

```javascript
const config = {
    TCP_PORT: 8080,           // TCP 飞控连接端口
    HTTP_PORT: 3000,          // HTTP/WebSocket 端口
    HEARTBEAT_INTERVAL: 100,  // 心跳间隔 (ms)
    HEARTBEAT_TIMEOUT: 10000, // 心跳超时 (ms)
    ...
};
```

### 修改协议

编辑 `src/protocol.js` 定义数据包类型、飞行模式等常量。

### 修改解析逻辑

编辑 `src/parser.js` 中的 `parseHeader()` 和 `parseTelemetry()` 函数。

### 前端开发

所有前端代码使用 ES6 模块化。在 `public/js/` 中：
- 导入其他模块：`import { func } from './module.js';`
- 导出函数：`export function myFunc() { ... }`
- 全局函数通过 `window.funcName = ...` 暴露给 HTML

### npm 命令

```bash
npm start       # 启动生产环境服务器
npm run dev     # 启动开发环境（热重载）
npm run legacy  # 运行旧版单文件服务器 (app.js)
```

### 扩展功能

可以基于此服务器扩展以下功能：

- 📊 历史数据记录和回放
- 📈 实时数据图表
- 🗺️ 飞行轨迹地图
- 📹 视频流集成
- 🎮 游戏手柄控制
- 💾 数据库存储
- 📡 多机协同

## 技术栈

- **Node.js** - 运行时环境
- **Express** - Web 框架
- **ws** - WebSocket 库
- **原生 net 模块** - TCP 服务器

## 许可证

与 ESP-Drone 项目保持一致 (GPL3.0)

## 相关文档

- [ESP-Drone 项目主页](https://github.com/espressif/esp-drone)
- [远程服务器通信配置指南](../文档/远程服务器通信配置指南.md)
- [通信架构改进记录](../文档/通信架构改进记录.md)
