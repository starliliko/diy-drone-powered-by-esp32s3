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
    <!-- 引入 Three.js -->
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <style>
        :root {
            --bg-color: #0f172a;
            --card-bg: #1e293b;
            --text-main: #f1f5f9;
            --text-muted: #94a3b8;
            --accent: #38bdf8;
            --success: #34d399;
            --danger: #f87171;
            --warning: #fbbf24;
            --card-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.5);
        }
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
            background-color: var(--bg-color);
            color: var(--text-main);
            padding: 24px;
            line-height: 1.5;
        }
        .container {
            max-width: 1400px;
            margin: 0 auto;
        }
        h1 {
            color: var(--text-main);
            text-align: center;
            margin-bottom: 32px;
            font-size: 2rem;
            font-weight: 300;
            letter-spacing: 1px;
        }
        h1 span { color: var(--accent); font-weight: 600; }
        
        .status {
            background: var(--card-bg);
            border-radius: 12px;
            padding: 24px;
            margin-bottom: 24px;
            box-shadow: var(--card-shadow);
            border: 1px solid rgba(255,255,255,0.05);
        }
        .status h2 {
            color: var(--accent);
            margin-bottom: 15px;
            border-bottom: 1px solid rgba(255,255,255,0.1);
            padding-bottom: 15px;
            font-size: 1.2rem;
        }
        
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 24px;
            margin-bottom: 24px;
        }
        .card {
            background: var(--card-bg);
            border-radius: 12px;
            padding: 24px;
            box-shadow: var(--card-shadow);
            border: 1px solid rgba(255,255,255,0.05);
        }
        .card h3 {
            color: var(--accent);
            margin-bottom: 20px;
            font-size: 1.1rem;
            text-transform: uppercase;
            letter-spacing: 1px;
            font-weight: 600;
        }
        
        /* 3D 模型容器 */
        #model-container {
            width: 100%;
            height: 250px;
            margin-bottom: 15px;
            border-radius: 8px;
            overflow: hidden;
            background: #000;
            position: relative;
        }
        
        .data-row {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 10px 0;
            border-bottom: 1px solid rgba(255,255,255,0.05);
        }
        .data-row:last-child { border-bottom: none; }
        
        .label { color: var(--text-muted); font-size: 0.9em; }
        .value { 
            font-family: 'Consolas', monospace; 
            font-weight: bold;
            color: var(--text-main);
        }
        
        .armed { color: var(--danger); animation: pulse 2s infinite; }
        .disarmed { color: var(--success); }
        .mode { 
            padding: 4px 12px;
            border-radius: 99px;
            font-size: 0.8rem;
            font-weight: 600;
            background: rgba(56, 189, 248, 0.15);
            color: var(--accent);
            border: 1px solid rgba(56, 189, 248, 0.3);
        }
        
        .control-panel {
            background: var(--card-bg);
            border-radius: 12px;
            padding: 24px;
            box-shadow: var(--card-shadow);
            border: 1px solid rgba(255,255,255,0.05);
        }
        
        .btn {
            padding: 12px 20px;
            margin: 6px;
            border: none;
            border-radius: 8px;
            font-size: 0.95rem;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.2s;
            color: #fff;
            box-shadow: 0 2px 4px rgba(0,0,0,0.2);
        }
        .btn:hover { transform: translateY(-2px); filter: brightness(1.1); }
        .btn:active { transform: translateY(0); }
        
        .btn-primary { background: var(--accent); color: #0f172a; }
        .btn-danger { background: var(--danger); color: #fff; }
        .btn-success { background: var(--success); color: #064e3b; }
        
        .connection-status {
            display: inline-block;
            width: 10px;
            height: 10px;
            border-radius: 50%;
            margin-right: 8px;
            box-shadow: 0 0 8px currentColor;
        }
        .connected { background: var(--success); color: var(--success); }
        .disconnected { background: var(--danger); color: var(--danger); }
        
        #console {
            background: #000;
            color: #22d3ee;
            padding: 20px;
            border-radius: 12px;
            font-family: 'Consolas', monospace;
            font-size: 0.85rem;
            max-height: 300px;
            overflow-y: auto;
            margin-top: 24px;
            border: 1px solid #333;
        }
        
        .attitude-display {
            display: flex;
            justify-content: space-around;
            text-align: center;
            padding: 10px 0;
            gap: 10px;
        }
        .attitude-value {
            font-family: 'Consolas', monospace;
            font-size: 1.8rem;
            font-weight: bold;
            color: var(--text-main);
            margin: 5px 0;
        }
        
        .progress-bar {
            background: rgba(255,255,255,0.1);
            border-radius: 8px;
            height: 24px;
            overflow: hidden;
            margin: 12px 0;
        }
        .progress-fill {
            height: 100%;
            background: linear-gradient(90deg, var(--accent) 0%, #2563eb 100%);
            transition: width 0.3s;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 0.8rem;
            font-weight: bold;
            text-shadow: 0 1px 2px rgba(0,0,0,0.5);
            color: #fff;
        }
        
        @keyframes pulse { 0% { opacity: 1; } 50% { opacity: 0.6; } 100% { opacity: 1; } }
    </style>
</head>
<body>
    <div class="container">
        <h1>🚁 ESP-Drone <span>远程监控系统</span></h1>
        
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
                <div id="model-container"></div>
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
            <h3 style="margin-bottom: 20px;">快捷控制</h3>
            <button class="btn btn-success" onclick="sendArm()">🔓 解锁</button>
            <button class="btn btn-danger" onclick="sendDisarm()">🔒 上锁</button>
            <button class="btn btn-primary" onclick="sendHover()">🛑 悬停</button>
            <button class="btn btn-danger" onclick="sendEmergency()">⚠️ 紧急停止</button>
            <button class="btn btn-primary" onclick="sendLand()">🛬 降落</button>
        </div>

        <div id="console"></div>
    </div>

    <script>
        // Three.js 变量
        let scene, camera, renderer, drone;
        
        function initThreeJS() {
            const container = document.getElementById('model-container');
            const width = container.clientWidth;
            const height = container.clientHeight;
            
            // 场景
            scene = new THREE.Scene();
            scene.background = new THREE.Color(0x1e293b); // card-bg
            
            // 相机 (透视)
            camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
            camera.position.set(0, 2, 3);
            camera.lookAt(0, 0, 0);
            
            // 渲染器
            renderer = new THREE.WebGLRenderer({ antialias: true });
            renderer.setSize(width, height);
            container.appendChild(renderer.domElement);
            
            // 灯光
            const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
            scene.add(ambientLight);
            
            const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
            dirLight.position.set(2, 5, 3);
            scene.add(dirLight);
            
            // --- 构建精细简约模型 (Sleek High-Fidelity) ---
            drone = new THREE.Group();

            // 材质定义
            const bodyMat = new THREE.MeshPhongMaterial({ color: 0x0f172a }); // Very Dark
            const frameMat = new THREE.MeshPhongMaterial({ color: 0x334155 }); // Slate Frame
            const motorMat = new THREE.MeshPhongMaterial({ color: 0x94a3b8, shininess: 80 }); // Metallic
            const propMat = new THREE.MeshPhongMaterial({ color: 0xe2e8f0, transparent: true, opacity: 0.8 });
            const accentMat = new THREE.MeshPhongMaterial({ color: 0x0ea5e9, emissive: 0x0c4a6e }); // Cyan Glow

            // 1. 机身 (纤薄)
            const body = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.06, 0.7), bodyMat);
            drone.add(body);
            
            // 2. 顶板 (装饰)
            const top = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.01, 0.4), accentMat);
            top.position.y = 0.036;
            drone.add(top);

            // 3. 电池 (底部)
            const batt = new THREE.Mesh(new THREE.BoxGeometry(0.35, 0.12, 0.6), frameMat);
            batt.position.y = -0.09;
            drone.add(batt);
            
            // 4. 机臂 (X型碳纤维杆)
            const armGeo = new THREE.BoxGeometry(2.1, 0.04, 0.12); // Long thin strip
            const arm1 = new THREE.Mesh(armGeo, frameMat);
            arm1.rotation.y = Math.PI / 4;
            drone.add(arm1);
            
            const arm2 = new THREE.Mesh(armGeo, frameMat);
            arm2.rotation.y = -Math.PI / 4;
            drone.add(arm2);

            // 5. 电机与桨叶 (无旋转动画)
            const motorGeo = new THREE.CylinderGeometry(0.14, 0.14, 0.12, 32);
            
            // 桨叶几何: 中心座 + 两片桨叶
            const hubGeo = new THREE.CylinderGeometry(0.04, 0.04, 0.05, 16);
            const bladeGeo = new THREE.BoxGeometry(1.4, 0.01, 0.12);

            const positions = [
                { x: -0.75, z: -0.75, cw: true },
                { x: 0.75, z: -0.75, cw: false },
                { x: 0.75, z: 0.75, cw: true }, 
                { x: -0.75, z: 0.75, cw: false }
            ];

            positions.forEach(pos => {
                const group = new THREE.Group();
                group.position.set(pos.x, 0.02, pos.z);

                // 电机
                const motor = new THREE.Mesh(motorGeo, motorMat);
                motor.position.y = 0.06;
                group.add(motor);

                // 桨帽
                const hub = new THREE.Mesh(hubGeo, motorMat); // Silver hub
                hub.position.y = 0.13;
                group.add(hub);

                // 桨叶 (白透)
                const prop = new THREE.Mesh(bladeGeo, propMat);
                prop.position.y = 0.13;
                group.add(prop);

                drone.add(group);
            });

            // 6. 头部摄像机/指示
            const cam = new THREE.Mesh(new THREE.BoxGeometry(0.15, 0.15, 0.1), accentMat);
            cam.position.set(0, 0, -0.36);
            drone.add(cam);

            scene.add(drone);
            
            animate();
            
            // 窗口大小适配
            window.addEventListener('resize', () => {
                const newWidth = container.clientWidth;
                const newHeight = container.clientHeight;
                renderer.setSize(newWidth, newHeight);
                camera.aspect = newWidth / newHeight;
                camera.updateProjectionMatrix();
            });
        }
        
        function animate() {
            requestAnimationFrame(animate);
            renderer.render(scene, camera);
        }

        // 调用初始化
        window.addEventListener('load', initThreeJS);

        let ws = null;
        const consoleDiv = document.getElementById('console');
        const maxConsoleLines = 50;

        function log(msg, type = 'info') {
            const timestamp = new Date().toLocaleTimeString();
            const line = document.createElement('div');
            line.textContent = \`[\${timestamp}] \${msg}\`;
            if (type === 'error') line.style.color = '#f87171'; // Red-400
            if (type === 'success') line.style.color = '#34d399'; // Emerald-400
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
                document.getElementById('ws-status').classList.remove('disconnected');
                document.getElementById('ws-status').classList.add('connected');
                document.getElementById('server-status').textContent = '在线';
                document.getElementById('server-status').style.color = 'var(--success)';
            };
            
            ws.onclose = () => {
                log('WebSocket 连接断开', 'error');
                document.getElementById('ws-status').classList.remove('connected');
                document.getElementById('ws-status').classList.add('disconnected');
                document.getElementById('server-status').textContent = '离线';
                document.getElementById('server-status').style.color = 'var(--danger)';
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
            
            // 更新 3D 模型姿态
            if (drone) {
                const toRad = Math.PI / 180;
                // 坐标映射: ThreeJS (Y-up, -Z forward) vs Drone (NED)
                // Pitch: 绕 X 轴
                // Yaw: 绕 Y 轴
                // Roll: 绕 Z 轴 (负号修正方向)
                drone.rotation.order = 'YXZ'; 
                drone.rotation.x = data.pitch * toRad;
                drone.rotation.y = -data.yaw * toRad; 
                drone.rotation.z = -data.roll * toRad;
            }
            
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
            
            // Adjust battery color based on percentage
            if (data.battPercent < 20) {
              battBar.style.background = 'var(--danger)';
            } else if (data.battPercent < 50) {
              battBar.style.background = 'var(--warning)';
            } else {
              battBar.style.background = 'linear-gradient(90deg, var(--accent) 0%, #2563eb 100%)';
            }
            
            document.getElementById('low-battery').textContent = data.isLowBattery ? '⚠️ 是' : '否';
            if (data.isLowBattery) document.getElementById('low-battery').style.color = 'var(--danger)';
            else document.getElementById('low-battery').style.color = 'var(--text-main)';
            
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
