# ESP-Drone Ground Station

> 基于 Node.js 的模块化地面站，提供遥测可视化、飞行控制与调试工具。

## 功能概览

- **TCP 服务器** — 接收飞控遥测数据（端口 8080）
- **Web 可视化** — 实时遥测仪表盘 + Three.js 3D 姿态（端口 3000）
- **WebSocket** — 实时双向数据推送
- **飞行控制** — 解锁 / 起飞 / 降落 / 悬停 / 紧急停止
- **电机测试** — 独立电机推力测试工具
- **模块化架构** — 前后端分离，易于维护和扩展

## 快速开始

```bash
npm install
npm start
```

服务器启动后：

| 服务 | 地址 |
|------|------|
| Web 界面 | http://localhost:3000 |
| WebSocket | ws://localhost:3000 |
| TCP（飞控连接） | 端口 8080 |

## 项目结构

```text
groundstation/
├── server.js                  # 服务器入口
├── package.json
├── src/                       # 后端模块
│   ├── config.js              #   服务器配置
│   ├── protocol.js            #   协议定义（PacketType / FlightMode 等）
│   ├── parser.js              #   数据包编解码
│   ├── DroneClient.js         #   TCP 客户端连接管理
│   └── DroneServer.js         #   主服务器（TCP / HTTP / WebSocket）
└── public/                    # 前端静态文件
    ├── index.html
    ├── css/style.css           #   QGC 风格样式
    └── js/
        ├── app.js              #   主入口（ES6 模块）
        ├── visualization.js    #   Three.js 3D 姿态
        ├── websocket.js        #   WebSocket 连接管理
        ├── control.js          #   飞行控制命令
        ├── motorTest.js        #   电机测试
        └── dashboard.js        #   遥测仪表盘
```

## 飞控端配置

### 方法 1：menuconfig

```bash
idf.py menuconfig
# → Component config → ESP-Drone Config → Remote Server Settings
```

- **Remote Server Enable**: ✓
- **Server IP**: 运行地面站的电脑 IP
- **Server Port**: 8080

获取电脑 IP：

```powershell
# PowerShell
Get-NetIPAddress -AddressFamily IPv4 |
  Where-Object { $_.InterfaceAlias -notlike '*Loopback*' } |
  Select-Object IPAddress, InterfaceAlias
```

### 方法 2：直接修改代码

```c
#define REMOTE_SERVER_ENABLE 1
#define REMOTE_SERVER_IP     "192.168.x.x"   // 电脑 IP
#define REMOTE_SERVER_PORT   8080
```

---

## 数据协议

### 包结构

```text
包头 (8 bytes):
┌───────┬───────┬─────────┬──────┬─────┬────────┐
│ 0xAB  │ 0xCD  │ Version │ Type │ Seq │ Length │
│ 1B    │ 1B    │ 1B      │ 1B   │ 2B  │ 2B     │
└───────┴───────┴─────────┴──────┴─────┴────────┘
包体: 可变长度，由 Type 决定
```

### 包类型

| Type | 名称 | 说明 |
|------|------|------|
| 0x00 | HEARTBEAT | 心跳 |
| 0x01 | TELEMETRY | 遥测数据 |
| 0x02 | CONTROL | 控制指令 |
| 0x03 | CRTP | CRTP 协议透传 |
| 0x04 | ACK | 应答 |
| 0x05 | CONFIG | 配置 |
| 0x06 | LOG | 日志消息 |

### 遥测数据结构（44 bytes）

```c
struct TelemetryData {
    int16_t  roll, pitch, yaw;         // 姿态角 (° × 100)
    int16_t  gyro_x, gyro_y, gyro_z;   // 角速度 (°/s × 10)
    int16_t  acc_x, acc_y, acc_z;      // 加速度 (mg)
    int32_t  pos_x, pos_y, pos_z;      // 位置 (m × 1000)
    int16_t  vel_x, vel_y, vel_z;      // 速度 (m/s × 1000)
    uint16_t batt_voltage;             // 电压 (V × 1000)
    uint8_t  batt_percent;             // 电量 %
    uint8_t  flight_mode;              // 飞行模式
    uint8_t  is_armed;                 // 解锁状态
    uint8_t  is_low_battery;           // 低电量标志
    uint8_t  reserved[2];
    uint32_t timestamp;                // 时间戳 (ms)
};
```

---

## API

### GET /api/status

返回服务器与飞控连接状态。

```json
{
  "timestamp": 1737449400000,
  "clients": [{
    "addr": "192.168.1.100:12345",
    "connectTime": 1737449300000,
    "rxCount": 1234,
    "txCount": 567,
    "lastHeartbeat": 1737449399000,
    "telemetry": { ... }
  }],
  "webClients": 2
}
```

### POST /api/control

发送控制指令。

```json
{
  "cmdType": 0,
  "roll": 0.0,
  "pitch": 0.0,
  "yaw": 0.0,
  "thrust": 0,
  "mode": 0
}
```

**cmdType 定义：**

| 值 | 类型 | 说明 |
|----|------|------|
| 0 | RPYT | 姿态 + 油门 |
| 1 | VELOCITY | 速度控制 |
| 2 | POSITION | 位置控制 |
| 3 | HOVER | 悬停 |
| 4 | LAND | 降落 |
| 5 | EMERGENCY | 紧急停止 |
| 6 | ARM | 解锁 |
| 7 | DISARM | 上锁 |

---

## Web 界面

### 实时数据

- 姿态角（Roll / Pitch / Yaw）
- 电池状态（电压 · 电量 · 低电量警告）
- 3D 位置与速度
- 飞行模式（DISABLED / STABILIZE / ALTITUDE_HOLD / POSITION_HOLD）
- 解锁状态

### 控制按钮

解锁 · 上锁 · 悬停 · 紧急停止 · 降落

---

## 控制源优先级

飞控支持多控制源同时接入，按优先级自动切换：

| 优先级 | 控制源 | 说明 |
|--------|--------|------|
| 3（最高） | SBUS 遥控器 | 传统遥控器，始终最优先 |
| 2 | TCP 地面站 | 本地面站远程控制 |
| 1 | CRTP / WiFi | 手机 APP / UDP 控制 |
| 0 | 无控制 | 无活跃控制源 |

- **高优先级优先**：多源同时活跃时，采用最高优先级控制源
- **超时降级**：控制源超过 1 秒无数据 → 自动切换到下一可用源
- **自动恢复**：超时源恢复后可重新获取控制权

Web 界面顶部状态栏显示各控制源连接状态与当前活跃源。

---

## 故障排除

| 问题 | 排查步骤 |
|------|----------|
| 飞控连不上 | ① 检查防火墙是否放行 TCP 8080 ② 确认飞控与电脑同一网络 ③ 核实飞控配置的服务器 IP ④ 查看飞控串口日志 |
| Web 无数据 | ① 浏览器控制台是否有错误 ② 检查 WebSocket 连接状态（左上角指示器）③ 刷新页面 ④ 检查服务器日志 |
| 控制无响应 | ① 确认 TCP 连接已建立 ② 确认当前飞行模式支持该指令 ③ 查看服务器日志 |

---

## 开发

### 修改配置

编辑 `src/config.js`：

```javascript
const config = {
    TCP_PORT: 8080,
    HTTP_PORT: 3000,
    HEARTBEAT_INTERVAL: 100,   // ms
    HEARTBEAT_TIMEOUT: 10000,  // ms
};
```

### npm 脚本

```bash
npm start       # 启动服务器
npm run dev     # 开发模式（热重载）
npm run legacy  # 旧版单文件服务器
```

### 前端开发

前端使用 ES6 模块（`public/js/`），通过 `import/export` 组织，全局函数通过 `window.funcName` 暴露给 HTML。

## 技术栈

| 层 | 技术 |
|----|------|
| 运行时 | Node.js |
| Web 框架 | Express |
| WebSocket | ws |
| TCP | 原生 net 模块 |
| 3D 渲染 | Three.js |

## 许可证

GPL-3.0，与 ESP-Drone 项目保持一致。
