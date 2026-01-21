/**
 * ESP-Drone 远程服务器 (Node.js)
 * ================================
 * 
 * 功能：
 * 1. TCP 服务器接收飞控遥测数据
 * 2. WebSocket 实时推送数据到前端
 * 3. HTTP 服务器提供可视化界面
 * 4. 支持发送控制指令
 */

const net = require('net');
const http = require('http');
const express = require('express');
const WebSocket = require('ws');

// ============================================================================
// 配置参数
// ============================================================================

const CONFIG = {
    TCP_PORT: 8080,        // TCP 服务器端口（接收飞控数据）
    HTTP_PORT: 3000,       // HTTP/WebSocket 端口（Web界面）
    HEARTBEAT_TIMEOUT: 10000  // 心跳超时时间(ms)
};

// ============================================================================
// 协议定义
// ============================================================================

const MAGIC = [0xAB, 0xCD];
const PROTOCOL_VERSION = 0x01;
const HEADER_SIZE = 8;

const PacketType = {
    HEARTBEAT: 0x00,
    TELEMETRY: 0x01,
    CONTROL: 0x02,
    CRTP: 0x03,
    ACK: 0x04,
    CONFIG: 0x05,
    LOG: 0x06
};

const ControlCmdType = {
    RPYT: 0x00,
    VELOCITY: 0x01,
    POSITION: 0x02,
    HOVER: 0x03,
    LAND: 0x04,
    EMERGENCY: 0x05,
    ARM: 0x06,
    DISARM: 0x07
};

const FlightMode = {
    0: 'DISABLED',
    1: 'STABILIZE',
    2: 'ALTITUDE_HOLD',
    3: 'POSITION_HOLD'
};

// ============================================================================
// 工具函数
// ============================================================================

/**
 * 解析包头
 */
function parseHeader(buffer) {
    if (buffer.length < HEADER_SIZE) return null;

    const magic0 = buffer.readUInt8(0);
    const magic1 = buffer.readUInt8(1);

    if (magic0 !== MAGIC[0] || magic1 !== MAGIC[1]) {
        return null;
    }

    return {
        magic0,
        magic1,
        version: buffer.readUInt8(2),
        pktType: buffer.readUInt8(3),
        seq: buffer.readUInt16LE(4),
        length: buffer.readUInt16LE(6)
    };
}

/**
 * 创建包头
 */
function createHeader(pktType, seq, payloadLength) {
    const buffer = Buffer.allocUnsafe(HEADER_SIZE);
    buffer.writeUInt8(MAGIC[0], 0);
    buffer.writeUInt8(MAGIC[1], 1);
    buffer.writeUInt8(PROTOCOL_VERSION, 2);
    buffer.writeUInt8(pktType, 3);
    buffer.writeUInt16LE(seq, 4);
    buffer.writeUInt16LE(payloadLength, 6);
    return buffer;
}

/**
 * 解析遥测数据
 */
function parseTelemetry(buffer) {
    // 最小长度检查（不含 timestamp 的版本是 42 字节，含 padding 和 timestamp 是 48 字节）
    if (buffer.length < 42) {
        console.log(`[WARN] Telemetry buffer too small: ${buffer.length} bytes`);
        return null;
    }

    let offset = 0;
    const telemetry = {
        roll: buffer.readInt16LE(offset) / 100.0,           // 度
        pitch: buffer.readInt16LE(offset + 2) / 100.0,      // 度
        yaw: buffer.readInt16LE(offset + 4) / 100.0,        // 度
        gyroX: buffer.readInt16LE(offset + 6) / 10.0,       // deg/s
        gyroY: buffer.readInt16LE(offset + 8) / 10.0,
        gyroZ: buffer.readInt16LE(offset + 10) / 10.0,
        accX: buffer.readInt16LE(offset + 12),              // mg
        accY: buffer.readInt16LE(offset + 14),
        accZ: buffer.readInt16LE(offset + 16),
        posX: buffer.readInt32LE(offset + 18) / 1000.0,     // m
        posY: buffer.readInt32LE(offset + 22) / 1000.0,
        posZ: buffer.readInt32LE(offset + 26) / 1000.0,
        velX: buffer.readInt16LE(offset + 30) / 1000.0,     // m/s
        velY: buffer.readInt16LE(offset + 32) / 1000.0,
        velZ: buffer.readInt16LE(offset + 34) / 1000.0,
        battVoltage: buffer.readUInt16LE(offset + 36) / 1000.0,  // V
        battPercent: buffer.readUInt8(offset + 38),         // %
        flightMode: buffer.readUInt8(offset + 39),
        isArmed: !!buffer.readUInt8(offset + 40),
        isLowBattery: !!buffer.readUInt8(offset + 41),
        // timestamp 在 offset 44 (跳过 2 字节 padding)，如果数据包足够长则读取
        timestamp: buffer.length >= 48 ? buffer.readUInt32LE(offset + 44) : Date.now()
    };

    // 添加可读的模式名称
    telemetry.flightModeName = FlightMode[telemetry.flightMode] || 'UNKNOWN';

    return telemetry;
}

/**
 * 创建控制指令
 */
function createControlCommand(cmdType, roll = 0, pitch = 0, yaw = 0, thrust = 0, mode = 0) {
    const buffer = Buffer.allocUnsafe(12);
    buffer.writeUInt8(cmdType, 0);
    buffer.writeInt16LE(Math.round(roll * 100), 1);
    buffer.writeInt16LE(Math.round(pitch * 100), 3);
    buffer.writeInt16LE(Math.round(yaw * 10), 5);
    buffer.writeUInt16LE(thrust, 7);
    buffer.writeUInt8(mode, 9);
    buffer.fill(0, 10, 12);  // padding
    return buffer;
}

// ============================================================================
// 飞控客户端类
// ============================================================================

class DroneClient {
    constructor(socket, remoteAddr, server) {
        this.socket = socket;
        this.remoteAddr = remoteAddr;
        this.server = server;
        this.rxBuffer = Buffer.alloc(0);
        this.txSeq = 0;
        this.lastHeartbeat = Date.now();
        this.connectTime = Date.now();
        this.rxCount = 0;
        this.txCount = 0;
        this.latestTelemetry = null;

        this.setupSocket();
        this.startHeartbeatCheck();

        console.log(`[${this.remoteAddr}] Drone connected`);
    }

    setupSocket() {
        this.socket.on('data', (data) => this.handleData(data));
        this.socket.on('error', (err) => this.handleError(err));
        this.socket.on('close', () => this.handleClose());
    }

    handleData(data) {
        this.rxBuffer = Buffer.concat([this.rxBuffer, data]);
        this.processBuffer();
    }

    processBuffer() {
        while (this.rxBuffer.length >= HEADER_SIZE) {
            const header = parseHeader(this.rxBuffer);

            if (!header) {
                // 同步丢失，跳过一个字节
                this.rxBuffer = this.rxBuffer.slice(1);
                continue;
            }

            const totalLen = HEADER_SIZE + header.length;
            if (this.rxBuffer.length < totalLen) {
                // 数据不完整
                break;
            }

            const payload = this.rxBuffer.slice(HEADER_SIZE, totalLen);
            this.rxBuffer = this.rxBuffer.slice(totalLen);

            this.rxCount++;
            this.handlePacket(header, payload);
        }
    }

    handlePacket(header, payload) {
        const pktType = header.pktType;

        switch (pktType) {
            case PacketType.HEARTBEAT:
                this.lastHeartbeat = Date.now();
                this.sendHeartbeat();
                break;

            case PacketType.TELEMETRY:
                const telemetry = parseTelemetry(payload);
                if (telemetry) {
                    this.latestTelemetry = telemetry;
                    this.server.onTelemetry(this, telemetry);
                }
                break;

            case PacketType.CRTP:
                this.server.onCRTP(this, payload);
                break;

            case PacketType.LOG:
                const logMsg = payload.toString('utf8');
                console.log(`[${this.remoteAddr}] LOG: ${logMsg}`);
                break;

            default:
                console.log(`[${this.remoteAddr}] Unknown packet type: ${pktType}`);
        }
    }

    sendHeartbeat() {
        const timestamp = Date.now() >>> 0;  // 使用无符号右移确保在 uint32 范围内
        const payload = Buffer.allocUnsafe(4);
        payload.writeUInt32LE(timestamp, 0);
        this.sendPacket(PacketType.HEARTBEAT, payload);
    }

    sendPacket(pktType, payload = Buffer.alloc(0)) {
        const header = createHeader(pktType, this.txSeq, payload.length);
        this.txSeq = (this.txSeq + 1) & 0xFFFF;

        try {
            this.socket.write(Buffer.concat([header, payload]));
            this.txCount++;
        } catch (err) {
            console.error(`[${this.remoteAddr}] TX error:`, err.message);
        }
    }

    sendControl(cmdType, roll, pitch, yaw, thrust, mode) {
        const payload = createControlCommand(cmdType, roll, pitch, yaw, thrust, mode);
        this.sendPacket(PacketType.CONTROL, payload);
    }

    startHeartbeatCheck() {
        this.heartbeatTimer = setInterval(() => {
            if (Date.now() - this.lastHeartbeat > CONFIG.HEARTBEAT_TIMEOUT) {
                console.log(`[${this.remoteAddr}] Heartbeat timeout`);
                this.close();
            }
        }, 3000);
    }

    handleError(err) {
        console.error(`[${this.remoteAddr}] Socket error:`, err.message);
    }

    handleClose() {
        console.log(`[${this.remoteAddr}] Drone disconnected`);
        clearInterval(this.heartbeatTimer);
        this.server.removeClient(this);
    }

    close() {
        this.socket.destroy();
    }

    getStats() {
        return {
            addr: this.remoteAddr,
            connectTime: this.connectTime,
            rxCount: this.rxCount,
            txCount: this.txCount,
            lastHeartbeat: this.lastHeartbeat,
            telemetry: this.latestTelemetry
        };
    }
}

// ============================================================================
// 飞控服务器类
// ============================================================================

class DroneServer {
    constructor() {
        this.clients = new Map();
        this.webClients = new Set();
        this.tcpServer = null;
        this.httpServer = null;
        this.wss = null;
    }

    start() {
        this.startTCPServer();
        this.startWebServer();
    }

    startTCPServer() {
        this.tcpServer = net.createServer((socket) => {
            const remoteAddr = `${socket.remoteAddress}:${socket.remotePort}`;
            const client = new DroneClient(socket, remoteAddr, this);
            this.clients.set(remoteAddr, client);
        });

        this.tcpServer.listen(CONFIG.TCP_PORT, () => {
            console.log(`✓ TCP Server listening on port ${CONFIG.TCP_PORT}`);
        });

        this.tcpServer.on('error', (err) => {
            console.error('TCP Server error:', err.message);
        });
    }

    startWebServer() {
        const app = express();

        // 静态文件
        app.use(express.static(__dirname));
        app.use(express.json());

        // API 端点
        app.get('/api/status', (req, res) => {
            const clients = Array.from(this.clients.values()).map(c => c.getStats());
            res.json({
                timestamp: Date.now(),
                clients: clients,
                webClients: this.webClients.size
            });
        });

        // 控制指令
        app.post('/api/control', (req, res) => {
            const { cmdType, roll, pitch, yaw, thrust, mode } = req.body;

            if (this.clients.size === 0) {
                return res.status(400).json({ error: 'No drone connected' });
            }

            // 发送给所有连接的飞控
            for (const client of this.clients.values()) {
                client.sendControl(
                    cmdType || ControlCmdType.RPYT,
                    roll || 0,
                    pitch || 0,
                    yaw || 0,
                    thrust || 0,
                    mode || 0
                );
            }

            res.json({ success: true });
        });

        // 主页面
        app.get('/', (req, res) => {
            res.send(this.getIndexHTML());
        });

        // HTTP 服务器
        this.httpServer = http.createServer(app);

        // WebSocket 服务器
        this.wss = new WebSocket.Server({ server: this.httpServer });
        this.wss.on('connection', (ws) => {
            console.log('WebSocket client connected');
            this.webClients.add(ws);

            ws.on('close', () => {
                console.log('WebSocket client disconnected');
                this.webClients.delete(ws);
            });

            ws.on('error', (err) => {
                console.error('WebSocket error:', err.message);
            });
        });

        this.httpServer.listen(CONFIG.HTTP_PORT, () => {
            console.log(`✓ HTTP/WebSocket Server listening on port ${CONFIG.HTTP_PORT}`);
            console.log(`  Open http://localhost:${CONFIG.HTTP_PORT} in your browser`);
        });
    }

    removeClient(client) {
        this.clients.delete(client.remoteAddr);
    }

    onTelemetry(client, telemetry) {
        // 打印到控制台
        console.log(`[${client.remoteAddr}] Telemetry: ` +
            `Roll=${telemetry.roll.toFixed(1)}° ` +
            `Pitch=${telemetry.pitch.toFixed(1)}° ` +
            `Yaw=${telemetry.yaw.toFixed(1)}° ` +
            `Mode=${telemetry.flightModeName} ` +
            `Armed=${telemetry.isArmed} ` +
            `Batt=${telemetry.battPercent}%`
        );

        // 广播到所有 WebSocket 客户端
        const data = JSON.stringify({
            type: 'telemetry',
            addr: client.remoteAddr,
            data: telemetry,
            timestamp: Date.now()
        });

        for (const ws of this.webClients) {
            if (ws.readyState === WebSocket.OPEN) {
                ws.send(data);
            }
        }
    }

    onCRTP(client, payload) {
        console.log(`[${client.remoteAddr}] CRTP packet (${payload.length} bytes)`);
    }

    getIndexHTML() {
        return `<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP-Drone 远程监控</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: #333;
            padding: 20px;
        }
        .container {
            max-width: 1400px;
            margin: 0 auto;
        }
        h1 {
            color: white;
            text-align: center;
            margin-bottom: 30px;
            font-size: 2.5em;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        .status {
            background: white;
            border-radius: 10px;
            padding: 20px;
            margin-bottom: 20px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }
        .status h2 {
            color: #667eea;
            margin-bottom: 15px;
            border-bottom: 2px solid #667eea;
            padding-bottom: 10px;
        }
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
            margin-bottom: 20px;
        }
        .card {
            background: white;
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }
        .card h3 {
            color: #667eea;
            margin-bottom: 15px;
            font-size: 1.3em;
        }
        .data-row {
            display: flex;
            justify-content: space-between;
            padding: 8px 0;
            border-bottom: 1px solid #eee;
        }
        .data-row:last-child {
            border-bottom: none;
        }
        .label {
            color: #666;
            font-weight: 500;
        }
        .value {
            font-weight: bold;
            color: #333;
        }
        .armed { color: #e74c3c; }
        .disarmed { color: #95a5a6; }
        .mode { 
            padding: 4px 12px;
            border-radius: 15px;
            font-size: 0.85em;
            background: #667eea;
            color: white;
        }
        .control-panel {
            background: white;
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }
        .btn {
            padding: 12px 24px;
            margin: 5px;
            border: none;
            border-radius: 5px;
            font-size: 1em;
            cursor: pointer;
            transition: all 0.3s;
            font-weight: 500;
        }
        .btn-primary {
            background: #667eea;
            color: white;
        }
        .btn-primary:hover {
            background: #5568d3;
            transform: translateY(-2px);
        }
        .btn-danger {
            background: #e74c3c;
            color: white;
        }
        .btn-danger:hover {
            background: #c0392b;
        }
        .btn-success {
            background: #27ae60;
            color: white;
        }
        .btn-success:hover {
            background: #229954;
        }
        .connection-status {
            display: inline-block;
            width: 12px;
            height: 12px;
            border-radius: 50%;
            margin-right: 8px;
        }
        .connected { background: #27ae60; }
        .disconnected { background: #e74c3c; }
        #console {
            background: #1e1e1e;
            color: #d4d4d4;
            padding: 15px;
            border-radius: 5px;
            font-family: 'Courier New', monospace;
            font-size: 0.9em;
            max-height: 300px;
            overflow-y: auto;
            margin-top: 20px;
        }
        .attitude-display {
            text-align: center;
            padding: 20px;
        }
        .attitude-value {
            font-size: 2.5em;
            font-weight: bold;
            color: #667eea;
            margin: 10px 0;
        }
        .progress-bar {
            background: #ecf0f1;
            border-radius: 10px;
            height: 25px;
            overflow: hidden;
            margin: 10px 0;
        }
        .progress-fill {
            height: 100%;
            background: linear-gradient(90deg, #667eea 0%, #764ba2 100%);
            transition: width 0.3s;
            display: flex;
            align-items: center;
            justify-content: center;
            color: white;
            font-weight: bold;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🚁 ESP-Drone 远程监控系统</h1>
        
        <div class="status">
            <h2>
                <span class="connection-status" id="ws-status"></span>
                连接状态
            </h2>
            <div class="data-row">
                <span class="label">服务器状态:</span>
                <span class="value" id="server-status">等待连接...</span>
            </div>
            <div class="data-row">
                <span class="label">飞控连接:</span>
                <span class="value" id="drone-count">0</span>
            </div>
            <div class="data-row">
                <span class="label">数据更新时间:</span>
                <span class="value" id="last-update">--</span>
            </div>
        </div>

        <div class="grid">
            <div class="card">
                <h3>姿态角</h3>
                <div class="attitude-display">
                    <div>
                        <div class="label">Roll (横滚)</div>
                        <div class="attitude-value" id="roll">--°</div>
                    </div>
                    <div>
                        <div class="label">Pitch (俯仰)</div>
                        <div class="attitude-value" id="pitch">--°</div>
                    </div>
                    <div>
                        <div class="label">Yaw (航向)</div>
                        <div class="attitude-value" id="yaw">--°</div>
                    </div>
                </div>
            </div>

            <div class="card">
                <h3>飞行状态</h3>
                <div class="data-row">
                    <span class="label">飞行模式:</span>
                    <span class="mode" id="flight-mode">--</span>
                </div>
                <div class="data-row">
                    <span class="label">解锁状态:</span>
                    <span class="value" id="armed-status">--</span>
                </div>
                <div class="data-row">
                    <span class="label">位置 (X/Y/Z):</span>
                    <span class="value" id="position">-- / -- / --</span>
                </div>
                <div class="data-row">
                    <span class="label">速度 (X/Y/Z):</span>
                    <span class="value" id="velocity">-- / -- / --</span>
                </div>
            </div>

            <div class="card">
                <h3>电池状态</h3>
                <div class="data-row">
                    <span class="label">电压:</span>
                    <span class="value" id="battery-voltage">-- V</span>
                </div>
                <div class="data-row">
                    <span class="label">电量:</span>
                </div>
                <div class="progress-bar">
                    <div class="progress-fill" id="battery-bar" style="width: 0%">0%</div>
                </div>
                <div class="data-row">
                    <span class="label">低电量警告:</span>
                    <span class="value" id="low-battery">--</span>
                </div>
            </div>

            <div class="card">
                <h3>传感器数据</h3>
                <div class="data-row">
                    <span class="label">陀螺仪 (X/Y/Z):</span>
                    <span class="value" id="gyro">-- / -- / --</span>
                </div>
                <div class="data-row">
                    <span class="label">加速度 (X/Y/Z):</span>
                    <span class="value" id="acc">-- / -- / --</span>
                </div>
            </div>
        </div>

        <div class="control-panel">
            <h3 style="color: #667eea; margin-bottom: 15px;">快捷控制</h3>
            <button class="btn btn-success" onclick="sendArm()">🔓 解锁</button>
            <button class="btn btn-danger" onclick="sendDisarm()">🔒 上锁</button>
            <button class="btn btn-primary" onclick="sendHover()">🛑 悬停</button>
            <button class="btn btn-danger" onclick="sendEmergency()">⚠️ 紧急停止</button>
            <button class="btn btn-primary" onclick="sendLand()">🛬 降落</button>
        </div>

        <div id="console"></div>
    </div>

    <script>
        let ws = null;
        const consoleDiv = document.getElementById('console');
        const maxConsoleLines = 50;

        function log(msg, type = 'info') {
            const timestamp = new Date().toLocaleTimeString();
            const line = document.createElement('div');
            line.textContent = \`[\${timestamp}] \${msg}\`;
            if (type === 'error') line.style.color = '#e74c3c';
            if (type === 'success') line.style.color = '#27ae60';
            consoleDiv.appendChild(line);
            
            // 限制控制台行数
            while (consoleDiv.children.length > maxConsoleLines) {
                consoleDiv.removeChild(consoleDiv.firstChild);
            }
            
            consoleDiv.scrollTop = consoleDiv.scrollHeight;
        }

        function connectWebSocket() {
            ws = new WebSocket(\`ws://\${window.location.host}\`);
            
            ws.onopen = () => {
                log('WebSocket 连接成功', 'success');
                document.getElementById('ws-status').className = 'connection-status connected';
                document.getElementById('server-status').textContent = '在线';
            };
            
            ws.onclose = () => {
                log('WebSocket 连接断开', 'error');
                document.getElementById('ws-status').className = 'connection-status disconnected';
                document.getElementById('server-status').textContent = '离线';
                setTimeout(connectWebSocket, 3000);
            };
            
            ws.onerror = (error) => {
                log('WebSocket 错误: ' + error, 'error');
            };
            
            ws.onmessage = (event) => {
                try {
                    const msg = JSON.parse(event.data);
                    if (msg.type === 'telemetry') {
                        updateTelemetry(msg.data);
                    }
                } catch (e) {
                    log('解析消息失败: ' + e.message, 'error');
                }
            };
        }

        function updateTelemetry(data) {
            document.getElementById('roll').textContent = data.roll.toFixed(1) + '°';
            document.getElementById('pitch').textContent = data.pitch.toFixed(1) + '°';
            document.getElementById('yaw').textContent = data.yaw.toFixed(1) + '°';
            
            document.getElementById('flight-mode').textContent = data.flightModeName;
            
            const armedSpan = document.getElementById('armed-status');
            armedSpan.textContent = data.isArmed ? '已解锁' : '已上锁';
            armedSpan.className = data.isArmed ? 'value armed' : 'value disarmed';
            
            document.getElementById('position').textContent = 
                \`\${data.posX.toFixed(2)} / \${data.posY.toFixed(2)} / \${data.posZ.toFixed(2)} m\`;
            
            document.getElementById('velocity').textContent = 
                \`\${data.velX.toFixed(2)} / \${data.velY.toFixed(2)} / \${data.velZ.toFixed(2)} m/s\`;
            
            document.getElementById('battery-voltage').textContent = data.battVoltage.toFixed(2) + ' V';
            
            const battBar = document.getElementById('battery-bar');
            battBar.style.width = data.battPercent + '%';
            battBar.textContent = data.battPercent + '%';
            
            document.getElementById('low-battery').textContent = data.isLowBattery ? '⚠️ 是' : '否';
            
            document.getElementById('gyro').textContent = 
                \`\${data.gyroX.toFixed(1)} / \${data.gyroY.toFixed(1)} / \${data.gyroZ.toFixed(1)} °/s\`;
            
            document.getElementById('acc').textContent = 
                \`\${data.accX} / \${data.accY} / \${data.accZ} mg\`;
            
            document.getElementById('last-update').textContent = new Date().toLocaleTimeString();
            document.getElementById('drone-count').textContent = '1';
        }

        async function sendControl(cmdType, roll = 0, pitch = 0, yaw = 0, thrust = 0, mode = 0) {
            try {
                const response = await fetch('/api/control', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ cmdType, roll, pitch, yaw, thrust, mode })
                });
                
                const result = await response.json();
                if (result.success) {
                    log('控制指令发送成功', 'success');
                } else {
                    log('控制指令失败: ' + result.error, 'error');
                }
            } catch (e) {
                log('发送失败: ' + e.message, 'error');
            }
        }

        function sendArm() { sendControl(6); }  // ARM
        function sendDisarm() { sendControl(7); }  // DISARM
        function sendHover() { sendControl(3); }  // HOVER
        function sendEmergency() { sendControl(5); }  // EMERGENCY
        function sendLand() { sendControl(4); }  // LAND

        // 启动连接
        connectWebSocket();
        log('系统启动，等待飞控连接...');
    </script>
</body>
</html>`;
    }
}

// ============================================================================
// 启动服务器
// ============================================================================

const server = new DroneServer();
server.start();

console.log('');
console.log('════════════════════════════════════════════════════════════');
console.log('  ESP-Drone 远程服务器已启动');
console.log('════════════════════════════════════════════════════════════');
console.log(`  TCP 端口:  ${CONFIG.TCP_PORT} (飞控连接)`);
console.log(`  Web 端口:  ${CONFIG.HTTP_PORT} (浏览器访问)`);
console.log(`  监控界面:  http://localhost:${CONFIG.HTTP_PORT}`);
console.log('════════════════════════════════════════════════════════════');
console.log('');

// 优雅退出
process.on('SIGINT', () => {
    console.log('\n正在关闭服务器...');
    process.exit(0);
});
