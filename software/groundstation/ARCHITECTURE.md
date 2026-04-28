# ESP-Drone Ground Station — 架构设计

> 同步代码状态:已实现完整 MAVLink v2 双栈、UDP 高频遥测、UDP 广播发现、VOFA+ 转发、PID 持久化与自动下发、磁力计/电机/ESC 完整调试链。

---

## 1. 系统架构

```text
┌──────────────────────────────────────────────────────────────┐
│                     ESP32-S3 飞控固件                         │
│        (FreeRTOS · Crazyflie 框架 · MAVLink/CRTP)            │
└────────┬───────────────┬───────────────────┬─────────────────┘
         │ TCP 8080      │ UDP 8082          │ UDP 8081
         │ MAVLink/0xABCD│ ESPU 高频遥测     │ ESPD 广播发现
         ▼               ▼                   ▲
┌──────────────────────────────────────────────────────────────┐
│              DroneServer (Node.js · server.js)               │
│  ┌────────────────────────────────────────────────────────┐  │
│  │ TCP server  ──→ DroneClient × N (per-IP)               │  │
│  │ UDP telemetry server                                   │  │
│  │ Discovery broadcaster (向 192.168.x.255)               │  │
│  │ PID profile (data/pid_profile.json)                    │  │
│  │ Express HTTP + WebSocket                               │  │
│  │ VOFA+ UDP forwarder (FireWater)                        │  │
│  └────────────────────────────────────────────────────────┘  │
└─────────┬───────────────────────────────────────┬────────────┘
          │ HTTP/WS 3000                          │ UDP 8083
          ▼                                       ▼
   ┌─────────────┐                          ┌──────────┐
   │  Browser    │                          │  VOFA+   │
   │  (4 tabs)   │                          │  /其他   │
   └─────────────┘                          └──────────┘
```

---

## 2. 后端模块 (`src/`)

| 模块 | 类/导出 | 职责 |
| --- | --- | --- |
| `config.js` | 常量对象 | 端口、`PROTOCOL_MODE`、超时、缓冲上限 |
| `protocol.js` | 枚举 + 魔数 | `PacketType` / `FlightMode` / `ControlSource` / `ControlCmdType` / `DISCOVERY_MAGIC` (`ESPD`) / `UDP_TELEMETRY_MAGIC` (`ESPU`) |
| `parser.js` | 函数集 | 私有协议:`parseHeader` / `parseTelemetry` / `createHeader` / `createControlCommand` |
| `mavlink.js` | 函数集 | MAVLink v2 帧编/解码、X25 CRC、消息子集 |
| `DroneClient.js` | `class DroneClient` | 单架飞控连接抽象。维护 RX 缓冲 → 双协议帧分离 → 心跳 → `sendControl` / `sendParamSet` |
| `DroneServer.js` | `class DroneServer` | 主服务,聚合一切:TCP/HTTP/WS/UDP × 3、PID 持久化、VOFA 转发、所有 REST API |

### `DroneServer` 关键成员

```js
this.clients         // Map<remoteAddr, DroneClient>
this.clientsByIP     // Map<droneIP, DroneClient>  // 用于匹配 UDP 遥测
this.webClients      // Set<WebSocket>
this.tcpServer / httpServer / wss
this.discoverySocket / udpTelemetrySocket / vofaSocket
this.pidProfile      // Map<"group.name", float>
this.vofaClients     // Map<"ip:port", {ip, port}>
```

启动入口:`start()` → `startTCPServer()` + `startWebServer()` + `startDiscoveryBroadcast()` + `startUDPTelemetryServer()` + `startVofaServer()`。

### PID 持久化与自动下发

- 文件:`data/pid_profile.json`,结构 `{version, updatedAt, params: {"group.name": value}}`
- 启动时 `loadPidProfile()` 读入 `Map`
- 飞控 TCP 连接建立 800 ms 后自动 `applyPidProfileToClient(client)`,逐条 `sendParamSet(group, name, FLOAT, value)`
- API `/api/pid/set` 与 `/api/pid/batch` 写入即刻保存 + 下发所有在线飞控
- 允许的 group:`pid_attitude` / `pid_rate` / `velCtlPid` / `posCtlPid`(白名单)

---

## 3. 前端模块 (`public/js/`)

```text
app.js (入口)
 ├── visualization.js   Three.js 飞机模型 / OrbitControls
 ├── websocket.js       WS 连接 + 自动重连 + 心跳 + 日志
 ├── dashboard.js       仪表盘 DOM 更新
 ├── control.js         ARM/DISARM/TAKEOFF/LAND/RTL/HOVER/远程控制模式
 ├── motorTest.js       4 滑块 + 单/全/顺序模式 + 3s 超时自停
 ├── pidTuner.js        uPlot 实时曲线 + 参数表 + REST 调用
 ├── magCalib.js        Three.js 3D 点云 + 校准状态机
 └── vofa-client.js     VOFA+ 客户端管理面板
```

`app.js` 装配顺序:

```js
viz.initThreeJS();
ws.connectWS();
ws.setTelemetryCallback(dash.updateDashboard);
ws.setMotorCallback(motor.updateMotorReadings);
ws.setLogCallback(consoleLog);
pid.init();   mag.init();   vofa.init();
```

全局函数通过 `window.fnName = ...` 暴露给 HTML `onclick`(`switchPage`、`sendAction`、`updateMotor` 等)。

---

## 4. 关键数据流

### 4.1 高频遥测 (UDP 上行)

```
ESP32 → UDP 8082 (ESPU 头 + payload)
         ↓
DroneServer.startUDPTelemetryServer
  → parseTelemetry(payload)
  → findClientByIP(rinfo.address) → client.latestTelemetry
  → broadcastTelemetry(addr, data)
        ├→ 所有 WS 客户端: {type:'telemetry', client, data}
        └→ 所有 vofaClients: FireWater 文本(13 通道 + '\n')
```

### 4.2 PID 调参

```
浏览器 pidTuner.js
  → fetch('/api/pid/set', {group, name, value})
DroneServer
  → normalizePidParam(白名单/类型)
  → mergePidProfileEntries → savePidProfile(JSON 落盘)
  → applyPidParamToAllClients
       → DroneClient.sendParamSet (MAVLink PARAM_SET, FLOAT)
            ↓ 长键应用别名 (pa_*/pp_*)
飞控 remote_server.c
  → 反向解析别名 → paramGetVarId → paramSetFloat
```

### 4.3 磁力计校准

```
浏览器 → POST /api/mag/calibrate/start
  → mag.calibrate=1 (UINT8)
飞控 sensors_bmi088_spi_ms5611.c 进入采集状态
浏览器周期性接收 mag.x/y/z (LOG) → 3D 点云渲染
浏览器 → POST /api/mag/calibrate/stop
  → mag.calibrate=0,固件计算 offset/scale,自动落参数
浏览器 → POST /api/mag/param  // 可手动写 declination/yawStdDev/yawEn
```

### 4.4 控制下行

```
浏览器按钮 → control.sendAction
  → fetch('/api/control', {cmdType, ...})
DroneServer.forEachDroneClient → DroneClient.sendControl
  → 按 PROTOCOL_MODE 选择 MAVLink MANUAL_CONTROL/COMMAND_LONG
    或 legacy CONTROL 包
TCP 8080 → 飞控
```

### 4.5 自动发现

```
DroneServer.startDiscoveryBroadcast
  → 每 DISCOVERY_INTERVAL ms 向 192.168.x.255 + 255.255.255.255
    发送 [ESPD][TCP_PORT][UDP_TELEMETRY_PORT]
飞控 收到广播后建立 TCP 8080 连接 + UDP 8082 上行
```

---

## 5. WebSocket 消息

浏览器 → 服务端:统一 `{type, ...}` JSON,目前服务端主要由 REST 驱动,WS 主要用于推送。

服务端 → 浏览器:

| type | 关键字段 |
| --- | --- |
| `telemetry` | `client`, `data: {roll,pitch,yaw, gyro, acc, motorOutputs[4], battery, flightMode, armingState, controlSource, protection}` |
| `status` | `clients[]`, `webClients`, `protocolMode` |
| `log` | `level`, `msg` |

---

## 6. 协议适配层 (`DroneClient`)

接收路径:

```
TCP onData → 累积 RX buffer
  → 帧分离循环:
      if buf[0:2] == AB CD:           ── legacy
         按 (Type, Length) 取出整帧 → parser.parseXxx
      elif buf[0] in {0xFD, 0xFE}:    ── MAVLink v2/v1
         解析包头 → 取出 payload → 校验 X25 + extra CRC
         分发到 onAttitude / onSysStatus / onParamValue / ...
      else:
         丢弃 1 字节,继续重同步
  → 缓冲超过 RX_BUFFER_MAX_BYTES 时清空,防 DoS
```

发送路径(`PROTOCOL_MODE` 决定):

| 模式 | sendControl | sendParamSet |
| --- | --- | --- |
| `legacy` | 自定义 0xABCD CONTROL 包 | 不支持(仅 MAVLink 通道) |
| `mavlink` | MANUAL_CONTROL / COMMAND_LONG | PARAM_SET + 16B 别名 |
| `dual` | 优先 MAVLink | PARAM_SET |

---

## 7. 部署

### 开发

```bash
npm install
npm run dev
```

### 生产

```bash
npm install --omit=dev
NODE_ENV=production npm start
```

### Nginx 反代

```nginx
location / {
    proxy_pass http://localhost:3000;
    proxy_http_version 1.1;
    proxy_set_header Upgrade $http_upgrade;
    proxy_set_header Connection "upgrade";
    proxy_set_header Host $host;
}
```

> TCP 8080 与 3 个 UDP 端口需直连飞控所在网段,通常无法走 7 层反代。

---

## 8. 扩展点

- **新控制指令**:`protocol.js` 加 `ControlCmdType` → `parser.js` 编码 → `control.js` 调用 → 必要时固件 `crtp_commander_*` 处理。
- **新遥测字段**:固件 `remote_server.c` 加入 MAVLink 消息 → `mavlink.js` 解码 → `dashboard.js` 渲染。
- **新参数面板**:`pidTuner.js` 是模板,组别加入 `ALLOWED_PID_GROUPS` 即可复用持久化机制。
- **历史回放**:`DroneServer.broadcastTelemetry` 是单一汇聚点,接 SQLite/MQTT 即可落盘。

---

## 9. 性能与限制

| 维度 | 数值 |
| --- | --- |
| TCP 心跳超时 | 10 s (`HEARTBEAT_TIMEOUT`) |
| WS 心跳/超时 | 10 / 15 s |
| RX 缓冲上限 | 256 KB,超出清空(防恶意流) |
| 最大单包 | 8 KB |
| 浏览器遥测节流 | 10 ms(`WEB_TELEMETRY_INTERVAL`) |
| PID 自动下发延迟 | 飞控连接后 800 ms |
| 发现广播间隔 | 2 s |

---

## 10. 调试

```bash
# 健康检查
curl http://localhost:3000/api/status

# WebSocket 抓包
npx wscat -c ws://localhost:3000

# UDP 遥测旁观
node test-vofa-receiver.js 8082    # 临时接管 8082 不可用,使用 vofa-receiver 仅作为 8083 校验
```

服务端日志前缀约定:`[TCP] [HTTP] [Discovery] [UDP Telemetry] [vofa+] [PID] [API]`,便于过滤。
