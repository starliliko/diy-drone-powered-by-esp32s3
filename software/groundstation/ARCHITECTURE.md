# ESP-Drone QGroundControl Lite - 架构设计文档

## 概述

这是一个现代化的模块化地面站架构，采用**前后端完全分离**的设计理念。后端使用 Node.js CommonJS 模块，前端使用 ES6 模块和 Three.js 3D 可视化。

## 系统架构图

```
┌─────────────────────────────────────────────────────────────┐
│                     ESP-Drone 飞控硬件                      │
│              (ESP32/S2/S3 运行 FreeRTOS)                    │
└────────────────┬────────────────────────────────────────────┘
                 │ TCP 连接
                 │ (Binary Protocol)
                 ▼
┌─────────────────────────────────────────────────────────────┐
│                  Node.js 服务器 (server.js)                 │
├─────────────────────────────────────────────────────────────┤
│ ┌───────────────────────────────────────────────────────┐   │
│ │ DroneServer (src/DroneServer.js)                      │   │
│ │ - TCP 服务器 (端口 8080)                              │   │
│ │ - HTTP 服务器 (端口 3000)                             │   │
│ │ - WebSocket 服务器                                    │   │
│ │ - REST API 路由                                       │   │
│ └───────────────────────────────────────────────────────┘   │
│           ▲                              ▲                   │
│           │                              │                   │
│    TCP/Binary                        HTTP/WS                │
│    Protocol                          Protocol                │
│           │                              │                   │
└─────────────────────────────────────────────────────────────┘
           │                              │
           │                    ┌─────────┴─────────┐
           │                    │                   │
           ▼                    ▼                   ▼
┌──────────────────┐  ┌──────────────────┐  ┌──────────────┐
│  DroneClient     │  │   Web Browser    │  │  Mobile App  │
│  (TCP Socket)    │  │   (HTTP/WS)      │  │  (WebSocket) │
└──────────────────┘  └──────────────────┘  └──────────────┘
                               │
                               │ GET /
                               ▼
                      ┌──────────────────┐
                      │  Static Files    │
                      │  (public/)       │
                      └──────────────────┘
```

## 模块层次结构

### 第1层: 应用层 (app.js)

```javascript
├── 初始化模块
│   ├── visualization.js (Three.js)
│   ├── websocket.js (连接管理)
│   ├── control.js (命令发送)
│   ├── motorTest.js (电机测试)
│   └── dashboard.js (UI更新)
│
└── 事件循环
    ├── WebSocket 消息处理
    ├── 按钮点击处理
    └── 页面交互处理
```

### 第2层: 通信层 (websocket.js)

- **入向**: WebSocket 消息 → 解析遥测数据 → 调用回调
- **出向**: 用户命令 → 打包为 JSON → 通过 WebSocket 发送

```
浏览器 WebSocket      →      服务器 WebSocket
   │                            │
   ├─ "telemetry" 消息    →     接收遥测
   ├─ "control" 消息      →     发送控制
   ├─ "motor_test" 消息   →     电机测试
   └─ "status" 消息       →     获取状态
```

### 第3层: 数据展示层

- **visualization.js** - Three.js 3D 模型更新 (姿态角)
- **dashboard.js** - DOM 元素更新 (遥测数据)
- **control.js** - 按钮和 UI 状态管理

### 第4层: HTTP 静态文件服务

```
GET /           → public/index.html
GET /css/style.css → public/css/style.css
GET /js/app.js     → public/js/app.js
```

## 后端架构

### DroneServer (核心服务器)

```javascript
class DroneServer {
  constructor() {
    this.tcpServer  // net.Server - TCP 服务器
    this.httpServer // Express + HTTP 服务器
    this.wss        // WebSocket.Server - WebSocket 服务器
    this.clients    // Map<remoteAddr, DroneClient>
    this.wsClients  // Set<WebSocket>
  }

  start()           // 启动服务器
  stop()            // 停止服务器
  handleTcpConnect()    // 处理新的 TCP 连接
  broadcastTelemetry()  // 广播遥测给所有 WS 客户端
  handleWebSocketMessage()  // 处理 WS 消息
}
```

### DroneClient (单个飞控连接)

```javascript
class DroneClient {
  constructor(socket)
    this.socket       // TCP Socket
    this.remoteAddr   // 远程地址
    this.rxCount      // 接收包计数
    this.txCount      // 发送包计数
    this.lastHeartbeat // 最后心跳时间
    this.telemetry    // 最新遥测数据
  }

  handleData()      // 处理接收的数据
  parseTelemetry()  // 解析遥测包
  sendHeartbeat()   // 发送心跳
  sendControl()     // 发送控制命令
}
```

### Protocol (协议定义)

```javascript
PacketType = {
  HEARTBEAT: 0x00,
  TELEMETRY: 0x01,
  CONTROL: 0x02,
  ...
}

FlightMode = {
  DISABLED: 0,
  STABILIZE: 1,
  ALTITUDE_HOLD: 2,
  ...
}

ControlSource = {
  NONE: 0,
  CRTP: 1,
  TCP_GCS: 2,
  SBUS: 3
}
```

### Parser (数据包处理)

```javascript
parseHeader(buffer)        // 解析包头
parseTelemetry(buffer)     // 解析遥测数据
createHeader(type, len)    // 创建包头
createControlCommand()     // 创建控制包
```

## 数据流

### 遥测数据流向

```
飞控 TCP 发送
    ↓ (二进制包)
DroneServer.tcpServer 接收
    ↓ 
DroneClient.handleData()
    ↓
parser.parseTelemetry()
    ↓
client.telemetry (缓存最新数据)
    ↓
broadcastTelemetry()
    ↓
WebSocket 广播给所有前端
    ↓
前端 WebSocket 消息事件
    ↓
websocket.onMessage() 回调
    ↓
dashboard.updateDashboard()  (更新 DOM)
visualization.updateDroneOrientation()  (更新 3D)
    ↓
用户看到实时更新
```

### 控制命令流向

```
用户点击 UI 按钮
    ↓
control.sendAction() 或 motorTest.updateMotor()
    ↓
websocket.send() 通过 WS 发送 JSON
    ↓
DroneServer 接收 WebSocket 消息
    ↓
parser.createControlCommand() 创建二进制包
    ↓
DroneClient.sendControl() 通过 TCP 发送给飞控
    ↓
飞控接收并执行命令
    ↓
飞控状态改变 → 遥测更新 → 反馈到前端
```

## 前端模块详解

### app.js (主入口)

作用：
- 模块初始化（顺序很重要！）
- 导出全局函数供 HTML onclick 使用
- 页面导航逻辑

```javascript
// 模块导入
import * as viz from './visualization.js';
import * as ws from './websocket.js';
import * as ctrl from './control.js';
import * as motor from './motorTest.js';
import * as dash from './dashboard.js';

// 页面导航
window.switchPage = (page) => { ... }
window.toggleConsole = () => { ... }

// 启动顺序
(async () => {
  viz.initThreeJS();      // 1. 初始化 3D
  ws.connectWS();         // 2. 连接 WebSocket
  ws.setTelemetryCallback(dash.updateDashboard);  // 3. 设置回调
  // ...
})();
```

### visualization.js (3D 可视化)

核心函数：
```javascript
initThreeJS()            // 初始化场景、相机、渲染器
createDroneModel()       // 创建四轴无人机 3D 模型
updateDroneOrientation(roll, pitch, yaw)  // 更新姿态角
```

### websocket.js (连接管理)

核心功能：
- 自动重连机制
- 心跳保活
- 消息处理回调
- 日志输出

```javascript
connectWS()              // 连接 WebSocket
setTelemetryCallback(fn) // 注册遥测回调
log(msg, type)          // 输出日志到控制台
```

### control.js (飞行控制)

核心函数：
```javascript
sendAction(action)              // 发送操作 (ARM/DISARM/TAKEOFF等)
setRemoteControlMode(mode)      // 设置远程控制模式
updateCtrlModeButtons()         // 更新 UI 按钮状态
```

### motorTest.js (电机测试)

核心函数：
```javascript
updateMotor(motorNum, value)    // 更新单个电机推力
stopAllMotors()                 // 停止所有电机
setAllMotors(value)             // 设置所有电机为同一推力
sequentialTest()                // 顺序测试每个电机
```

### dashboard.js (数据仪表盘)

核心函数：
```javascript
updateDashboard(data)           // 更新所有 UI 显示
updateTelemetryDisplay()        // 更新遥测数据
updateStatusIndicators()        // 更新状态指示器
updateBatteryBar()              // 更新电池显示
```

## 通信协议

### 二进制协议 (TCP)

```
┌─────────────────────────────────────────┐
│ 包头 (8 bytes)                          │
├────────┬────────┬────────┬─────┬─────┬───┤
│ Magic1 │ Magic2 │Version │Type │ Seq │Len│
│ 0xAB   │ 0xCD   │  0x01  │ 1B  │ 2B  │2B │
├─────────────────────────────────────────┤
│ 包体 (可变长度)                         │
│ (根据 Type 类型不同)                    │
└─────────────────────────────────────────┘
```

### WebSocket 协议 (JSON)

前端 → 服务器：
```json
{
  "type": "control",
  "data": {
    "cmdType": 0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "thrust": 50000
  }
}
```

服务器 → 前端（广播）：
```json
{
  "type": "telemetry",
  "client": "192.168.1.100:59516",
  "data": {
    "roll": 1.2,
    "pitch": -2.5,
    "yaw": -17.1,
    "armingState": 0,
    "flightMode": 0,
    "batteryPercent": 100,
    "timestamp": 1737449400000
  }
}
```

## 错误处理

### 后端错误处理

```javascript
// TCP 连接错误
tcpServer.on('error', (err) => {
  console.error('[ERROR] TCP 错误:', err);
});

// WebSocket 错误
ws.on('error', (err) => {
  console.error('[ERROR] WebSocket 错误:', err);
});

// 未捕获异常
process.on('uncaughtException', (err) => {
  console.error('[ERROR] 未捕获异常:', err);
});
```

### 前端错误处理

```javascript
try {
  ws.send(command);
} catch (err) {
  console.error('[ERROR] 发送失败:', err);
  ws.log('发送失败: ' + err.message, 'error');
}
```

## 扩展点

### 添加新的飞控命令

1. 在 `src/protocol.js` 中定义新的 `ControlCmdType`
2. 在 `src/parser.js` 中实现对应的 `createXxxCommand()` 函数
3. 在 `public/js/control.js` 中添加新的 `window.xxxCommand` 函数
4. 在 `public/index.html` 中添加对应的 UI 按钮

### 添加新的遥测字段

1. 在 `src/protocol.js` 中更新 `TelemetryData` 结构
2. 在 `src/parser.js` 中更新 `parseTelemetry()` 函数
3. 在 `public/js/dashboard.js` 中添加对应的显示逻辑
4. 在 `public/index.html` 中添加对应的 UI 元素

### 添加历史数据记录

```javascript
// 在 src/DroneServer.js 中
const dataHistory = [];  // 或接入数据库

// 每次收到遥测时
dataHistory.push({
  timestamp: Date.now(),
  clientAddr: client.remoteAddr,
  telemetry: data
});
```

## 性能考虑

### 前端

- Three.js 渲染帧率：60 FPS
- WebSocket 消息频率：10 Hz (100ms)
- DOM 更新防抖：最小 50ms

### 后端

- TCP 心跳间隔：100 ms
- 心跳超时：10 s
- WebSocket 广播：每收到遥测包时广播一次

### 内存管理

- 飞控客户端只缓存最新遥测数据
- WebSocket 客户端列表自动清理断开连接
- 无历史数据堆积（需要手动保存到数据库）

## 部署指南

### 开发环境

```bash
npm install
npm run dev  # 启用 --watch 热重载
```

### 生产环境

```bash
npm install --production
npm start  # 单进程启动
```

### Docker 部署

```dockerfile
FROM node:18-alpine
WORKDIR /app
COPY . .
RUN npm install --production
EXPOSE 3000 8080
CMD ["node", "server.js"]
```

### 反向代理 (Nginx)

```nginx
upstream gcs {
    server localhost:3000;
}

upstream drone_tcp {
    server localhost:8080;
}

server {
    listen 80;
    server_name yourdomain.com;

    location / {
        proxy_pass http://gcs;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host $host;
    }
}
```

## 调试技巧

### 服务器日志

所有事件都会打印到控制台：
```
[DroneClient] 新连接: 192.168.1.100:59516
[DroneClient] 接收到遥测: 44 bytes
[WebSocket] 客户端连接
[WebSocket] 广播遥测给 2 个客户端
```

### 浏览器开发工具

1. **Network 标签** - 查看 WebSocket 消息
2. **Console 标签** - 查看 JS 错误和日志
3. **Performance 标签** - 性能分析

### 命令行测试

```bash
# 测试 HTTP 接口
curl http://localhost:3000/api/status

# 使用 wscat 测试 WebSocket
npm install -g wscat
wscat -c ws://localhost:3000
```

## 故障排查

| 问题           | 排查步骤                                                    |
| -------------- | ----------------------------------------------------------- |
| 飞控连接不上   | 1. 检查防火墙 2. 检查 IP/端口 3. 查看服务器日志             |
| WebSocket 断连 | 1. 检查网络 2. 查看浏览器控制台 3. 重连机制自动处理         |
| 遥测数据不更新 | 1. 检查 WS 连接 2. 检查飞控是否发送数据 3. 检查 parser 解析 |
| 控制命令无响应 | 1. 检查飞控是否解锁 2. 检查命令格式 3. 查看飞控日志         |

---

**更新时间**: 2024年2月  
**版本**: 2.0 (模块化架构)  
**维护者**: ESP-Drone 项目团队
