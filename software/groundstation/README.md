# ESP-Drone Ground Station (QGC Lite)

> 基于 Node.js 的 ESP32-S3 飞控配套地面站。提供遥测可视化、3D 姿态、飞行控制、PID 调参、磁力计校准、电机/ESC 测试,以及向 [VOFA+](https://www.vofa.plus/) 的实时数据转发。

---

## 1. 功能概览

| 标签页 | 功能 |
| --- | --- |
| 📊 飞行监控 | 姿态/电机/电池/保护状态实时仪表盘 + Three.js 3D 姿态 + 飞行控制按钮 |
| ⚙️ 电机测试 | 单/全/顺序模式电机测试,3 秒无操作自动停止;ESC 行程校准 |
| 🎛️ PID 调参 | `pid_attitude` / `pid_rate` / `velCtlPid` / `posCtlPid` 实时曲线 + 参数表;持久化到 `data/pid_profile.json`,飞控连接后自动下发 |
| 🧭 磁力计校准 | 3D 点云采集、`offX/Y/Z` + `scaleX/Y/Z` 写回飞控 |

通信特性:

- **双协议栈**:`legacy`(自定义 0xABCD 二进制)/ `mavlink`(MAVLink v2)/ `dual`(同时兼容)
- **TCP** 接收飞控连接,**UDP** 接收高频遥测,**UDP 广播** 自动发现地面站
- **WebSocket** 推送给浏览器,**UDP/FireWater** 转发给 VOFA+
- 多飞控连接 + 多浏览器客户端

---

## 2. 快速开始

```bash
npm install
npm start          # 生产
npm run dev        # 开发(--watch 热重载)
```

服务端口(详见 [`src/config.js`](src/config.js)):

| 端口 | 协议 | 用途 |
| --- | --- | --- |
| 3000 | HTTP / WebSocket | Web 界面与浏览器双向通道 |
| 8080 | TCP | 飞控主连接(控制 + 心跳 + 参数) |
| 8081 | UDP 广播 | 地面站发现(`ESPD` 魔数) |
| 8082 | UDP | 飞控高频遥测上行(`ESPU` 魔数) |
| 8083 | UDP | VOFA+ FireWater 转发 |

打开浏览器访问 <http://localhost:3000>。

---

## 3. 飞控端配置

通过 `idf.py menuconfig` → `diy-drone config` → `wireless config`:

```
[*] Enable STA+AP Mode
    Router SSID = <你的 WiFi>
    Router Password = <密码>
[*] Enable Remote Server Connection
    Remote Server IP   = <运行地面站的电脑 IP>
    Remote Server Port = 8080
```

获取本机 IP(PowerShell):

```powershell
Get-NetIPAddress -AddressFamily IPv4 |
  Where-Object { $_.InterfaceAlias -notlike '*Loopback*' } |
  Select-Object IPAddress, InterfaceAlias
```

> 地面站启动时会向本网段广播 8081 端口,飞控可通过广播自动定位地面站(无需固定 IP)。

---

## 4. 项目结构

```text
groundstation/
├── server.js                    # 入口,实例化 DroneServer
├── package.json                 # express ^5 / ws ^8 / bonjour-service ^1
├── motor_test.js                # CLI 电机测试工具(直连飞控 TCP)
├── test-vofa-receiver.js        # VOFA+ UDP 接收测试器
├── data/
│   └── pid_profile.json         # PID 持久化(自动生成)
├── src/                         # 后端 (CommonJS)
│   ├── config.js                #   端口、协议模式、超时
│   ├── protocol.js              #   PacketType / FlightMode / ControlSource / 魔数
│   ├── parser.js                #   私有协议二进制编解码
│   ├── mavlink.js               #   MAVLink v2 编解码(最小子集)
│   ├── DroneClient.js           #   单架飞控连接(TCP + 双协议解析)
│   └── DroneServer.js           #   主服务: TCP/HTTP/WS/UDP/PID/VOFA
└── public/                      # 前端 (ES6 modules)
    ├── index.html
    ├── css/style.css
    └── js/
        ├── app.js               # 入口、模块装配、页面切换
        ├── websocket.js         # WS 连接、自动重连、心跳、日志
        ├── visualization.js     # Three.js 3D 飞机模型
        ├── dashboard.js         # 遥测仪表盘
        ├── control.js           # ARM/DISARM/TAKEOFF/LAND/RTL/HOVER
        ├── motorTest.js         # 电机测试 UI
        ├── pidTuner.js          # PID 实时曲线 + 参数表
        ├── magCalib.js          # 磁力计 3D 点云
        └── vofa-client.js       # VOFA+ 配置面板
```

---

## 5. 协议模式

[`src/config.js`](src/config.js) 中 `PROTOCOL_MODE` 决定与飞控通信的栈:

```js
PROTOCOL_MODE: 'mavlink',   // 'legacy' | 'mavlink' | 'dual'
```

- `legacy` — 仅自定义 `0xAB 0xCD` 二进制协议
- `mavlink` — 仅 MAVLink v2(默认,推荐)
- `dual` — 双栈识别;接收时按起始字节自动识别,发送时优先 MAVLink

> 切换模式后需重启服务,飞控端配置同步切换。在 `dual` 模式下,所有参数写入(电机/PID/磁力计)统一走 MAVLink 的 `PARAM_SET`,避免 CRTP 与 MAVLink 混合产生不一致。

### 参数键长度别名

MAVLink `PARAM_SET` 限制 param_id 为 16 字节,而部分参数键过长(如 `pid_attitude.roll_kp`)。系统使用以下别名,由固件 [`remote_server.c`](../firmware/components/core/crazyflie/modules/src/remote_server.c) 反向解析回原 (group, name):

| 别名前缀 | 实际组 |
| --- | --- |
| `pa_*` | `pid_attitude.*` |
| `pp_*` | `posCtlPid.*` |

其余短键直接使用。

---

## 6. REST API

| 方法 | 路径 | 功能 |
| --- | --- | --- |
| **状态** | | |
| GET | `/api/status` | 飞控连接列表 + WebSocket 客户端数 |
| **飞行控制** | | |
| POST | `/api/control` | `{cmdType, roll, pitch, yaw, thrust, mode}` |
| **电机测试** | | |
| POST | `/api/motor/set` | 单电机 `{motorId 0-3, thrust 0-65535}` |
| POST | `/api/motor/all` | 全部同推 `{thrust}` |
| POST | `/api/motor/sequential` | 顺序测试 `{thrust}` |
| POST | `/api/motor/estop` | 紧急停止(写 `motortest.estop=1`) |
| POST | `/api/esc/calibrate-first` | ESC 首次行程 `{stage: start\|finish\|abort}` |
| **磁力计** | | |
| POST | `/api/mag/calibrate/start` | `mag.calibrate=1` 开始采集 |
| POST | `/api/mag/calibrate/stop` | `mag.calibrate=0` 结束并固化 |
| POST | `/api/mag/param` | `{name, value, type}`,`name` ∈ `offX/Y/Z, scaleX/Y/Z, declination, yawStdDev, yawEn` |
| **PID** | | |
| GET | `/api/pid/profile` | 当前 PID 全量 |
| POST | `/api/pid/profile` | `{params: {"group.name": value, ...}}` 批量保存 |
| POST | `/api/pid/set` | `{group, name, value}` 单参,自动持久化 + 下发 |
| POST | `/api/pid/batch` | `{params: [{group, name, value}, ...]}` |
| **VOFA+** | | |
| POST | `/api/vofa/enable` | `{enabled: bool}` |
| GET | `/api/vofa/status` | 启停状态 + 客户端列表 |
| POST | `/api/vofa/add-client` | `{ip, port}` |
| POST | `/api/vofa/remove-client` | `{key: "ip:port"}` |

### WebSocket 消息

浏览器 → 服务端为 `{type, ...}` JSON;服务端 → 浏览器主要类型:

| type | 内容 |
| --- | --- |
| `telemetry` | `{client, data: {roll, pitch, yaw, motorOutputs[4], battery, flightMode, armingState, protection, ...}}` |
| `status` | 客户端连/断、协议模式 |
| `log` | 服务器日志推送 |

---

## 7. 工具脚本

### 电机直测 CLI

绕过地面站,直接连飞控(TCP 8080)发参数:

```bash
node motor_test.js stop
node motor_test.js sequential 30000
node motor_test.js single 0 25000
node motor_test.js all 20000
```

### VOFA+ UDP 接收器

用于在没有 VOFA+ 时验证转发链:

```bash
node test-vofa-receiver.js          # 默认端口 8083
node test-vofa-receiver.js 1347     # 指定端口
```

VOFA+ FireWater 通道顺序:`roll, pitch, yaw, gyroX, gyroY, gyroZ, accX, accY, accZ, M1, M2, M3, M4`(共 13 通道)。

---

## 8. 数据协议(私有 0xABCD)

仅 `legacy`/`dual` 模式使用,`mavlink` 模式可忽略本节。

```text
包头 (8 B):
┌──────┬──────┬─────────┬──────┬─────┬────────┐
│ 0xAB │ 0xCD │ Version │ Type │ Seq │ Length │
│  1B  │  1B  │   1B    │  1B  │ 2B  │   2B   │
└──────┴──────┴─────────┴──────┴─────┴────────┘
包体: 由 Type 决定
```

包类型见 [`src/protocol.js`](src/protocol.js):`HEARTBEAT 0x00` / `TELEMETRY 0x01` / `CONTROL 0x02` / `CRTP 0x03` / `ACK 0x04` / `CONFIG 0x05` / `LOG 0x06`。

UDP 高频遥测包前缀魔数为 `ESPU`(0x45 0x53 0x50 0x55),后接 1 字节类型 + 1 字节序号 + payload(同 TELEMETRY)。

---

## 9. 控制源优先级(飞控端)

飞控同时接收多路控制源,按优先级聚合,1 秒无数据自动降级:

| 优先级 | 源 |
| --- | --- |
| 3 | SBUS 遥控器 |
| 2 | TCP 地面站(本工程) |
| 1 | CRTP / WiFi(手机 APP / cfclient) |
| 0 | 无 |

详见固件 [`vehicle_state.c`](../firmware/components/core/crazyflie/modules/src/vehicle_state.c)。

---

## 10. 故障排除

| 现象 | 排查 |
| --- | --- |
| 飞控 TCP 连不上 | ① 防火墙放行 8080 ② 飞控/电脑同一网段 ③ 飞控 menuconfig 中 IP/Port 正确 ④ 试试 8081 广播自动发现 |
| 浏览器无遥测 | ① 左上角 WS 指示灯 ② 浏览器控制台错误 ③ 服务端日志 ④ `PROTOCOL_MODE` 与飞控一致 |
| PID 写入失败 | ① 飞控已连接 ② 参数键存在(看飞控 `param.c` LOG) ③ 长键应使用 `pa_*`/`pp_*` 别名 |
| 磁力计校准卡住 | ① 启动后必须缓慢翻滚画 8 字 30 s+ ② 检查 QMC5883P I2C 通信 |
| VOFA+ 无数据 | ① `/api/vofa/enable` 已启用 ② 客户端已 `add-client` ③ 防火墙放行 UDP 8083 |

---

## 11. 开发指南

```bash
npm start       # 生产模式
npm run dev     # 热重载
```

- **后端 (CommonJS)**:在 `src/DroneServer.js` 增加 `app.get/post('/api/...')`,在 `src/parser.js` 或 `src/mavlink.js` 增加编解码。
- **前端 (ES6 module)**:在 `public/js/` 新增模块,在 `app.js` 中 `import` 并装配,通过 `window.fnName = ...` 暴露给 HTML `onclick`。

技术栈:

| 层 | 选型 |
| --- | --- |
| 运行时 | Node.js ≥ 16 |
| HTTP | Express ^5 |
| WebSocket | ws ^8 |
| 服务发现 | bonjour-service / 自定义 UDP 广播 |
| TCP/UDP | 原生 `net` / `dgram` |
| 3D | Three.js r128 (CDN) |
| 实时图表 | uPlot 1.6 (CDN) |

---

## 12. 许可证

GPL-3.0,与 ESP-Drone 上游保持一致。
