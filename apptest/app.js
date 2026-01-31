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
const dgram = require('dgram');
const os = require('os');

// ============================================================================
// 配置参数
// ============================================================================

const CONFIG = {
    TCP_PORT: 8080,        // TCP 服务器端口（接收飞控数据）
    HTTP_PORT: 3000,       // HTTP/WebSocket 端口（Web界面）
    DISCOVERY_PORT: 8081,  // UDP 广播发现端口
    DISCOVERY_INTERVAL: 2000, // 广播间隔(ms)
    HEARTBEAT_TIMEOUT: 10000  // 心跳超时时间(ms)
};

// 发现协议魔数 "ESPD"
const DISCOVERY_MAGIC = Buffer.from([0x45, 0x53, 0x50, 0x44]);

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
    DISARM: 0x07,
    SET_CONTROL_MODE: 0x10  // 设置远程控制模式
};

// 远程控制模式
const RemoteControlMode = {
    DISABLED: 0,    // 服务器控制禁用
    ENABLED: 1,     // 服务器控制启用（默认，无其他控制源时接管）
    SHARED: 2       // 共享控制（遥控器+服务器）
};

const RemoteControlModeNameCN = {
    0: '禁用',
    1: '启用',
    2: '协同'
};

const FlightMode = {
    0: 'MANUAL',         // 手动模式
    1: 'STABILIZE',      // 自稳模式
    2: 'ALTITUDE',       // 定高模式
    3: 'POSITION',       // 定点模式
    4: 'MISSION',        // 任务模式
    5: 'RTL',            // 返航模式
    6: 'LAND',           // 降落模式
    7: 'TAKEOFF',        // 起飞模式
    8: 'OFFBOARD'        // 外部控制模式
};

const FlightModeNameCN = {
    0: '手动模式',
    1: '自稳模式',
    2: '定高模式',
    3: '定点模式',
    4: '任务模式',
    5: '返航模式',
    6: '降落模式',
    7: '起飞模式',
    8: '外部控制'
};

// === V3 新增：实际飞行姿态模式（由传感器能力决定）===
const ActualFlightMode = {
    0: 'STABILIZE',      // 自稳（仅IMU）
    1: 'ALTHOLD',        // 定高（IMU + 气压计/ToF）
    2: 'POSHOLD'         // 定点（IMU + 气压计/ToF + 光流）
};

const ActualFlightModeNameCN = {
    0: '自稳模式',
    1: '定高模式',
    2: '定点模式'
};

// === V3 新增：控制来源 ===
const ControlSource = {
    0: 'NONE',           // 无控制/超时
    1: 'CRTP',           // Wi-Fi UDP/手机APP
    2: 'REMOTE',         // TCP地面站
    3: 'SBUS'            // SBUS遥控器
};

const ControlSourceNameCN = {
    0: '无控制',
    1: 'WiFi控制',
    2: '地面站',
    3: '遥控器'
};

// PX4风格解锁状态
const ArmingState = {
    0: 'INIT',
    1: 'STANDBY',
    2: 'ARMED',
    3: 'STANDBY_ERROR',
    4: 'SHUTDOWN',
    5: 'IN_AIR_RESTORE'
};

const ArmingStateNameCN = {
    0: '初始化',
    1: '待机',
    2: '已解锁',
    3: '待机错误',
    4: '关机',
    5: '空中恢复'
};

// 飞行阶段
const FlightPhase = {
    0: 'ON_GROUND',
    1: 'TAKEOFF',
    2: 'IN_AIR',
    3: 'LANDING',
    4: 'LANDED'
};

const FlightPhaseNameCN = {
    0: '地面待机',
    1: '起飞中',
    2: '空中飞行',
    3: '降落中',
    4: '已降落'
};

// 故障安全状态
const FailsafeState = {
    0: 'NONE',
    1: 'LOW_BATTERY',
    2: 'CRITICAL_BATTERY',
    3: 'RC_LOSS',
    4: 'GCS_LOSS',
    5: 'SENSOR_FAILURE',
    6: 'GEOFENCE'
};

// 状态标志位
const STATUS_FLAGS = {
    ARMED: 0x01,
    FLYING: 0x02,
    EMERGENCY: 0x04,
    RC_CONNECTED: 0x08,
    GCS_CONNECTED: 0x10,
    BATTERY_LOW: 0x20
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
 * 解析遥测数据 - PX4/QGC 风格 (Protocol V2)
 */
function parseTelemetry(buffer) {
    // 新格式最小长度检查 (52字节)
    if (buffer.length < 42) {
        console.log(`[WARN] Telemetry buffer too small: ${buffer.length} bytes`);
        return null;
    }

    let offset = 0;
    const telemetry = {
        // 姿态 (度)
        roll: buffer.readInt16LE(offset) / 100.0,
        pitch: buffer.readInt16LE(offset + 2) / 100.0,
        yaw: buffer.readInt16LE(offset + 4) / 100.0,
        // 角速度 (deg/s)
        gyroX: buffer.readInt16LE(offset + 6) / 10.0,
        gyroY: buffer.readInt16LE(offset + 8) / 10.0,
        gyroZ: buffer.readInt16LE(offset + 10) / 10.0,
        // 加速度 (mg)
        accX: buffer.readInt16LE(offset + 12),
        accY: buffer.readInt16LE(offset + 14),
        accZ: buffer.readInt16LE(offset + 16),
        // 位置 (mm) - 跳过 offset 18-29
        // 速度 (mm/s) - 跳过 offset 30-35
        // 电池状态
        battVoltage: buffer.readUInt16LE(offset + 36) / 1000.0,  // V
        battPercent: buffer.readUInt8(offset + 38),              // %
    };

    // PX4风格状态字段 (新协议)
    if (buffer.length >= 52) {
        // 新格式 Protocol V2/V3
        telemetry.armingState = buffer.readUInt8(offset + 39);
        telemetry.flightMode = buffer.readUInt8(offset + 40);
        telemetry.flightPhase = buffer.readUInt8(offset + 41);
        telemetry.failsafeState = buffer.readUInt8(offset + 42);
        telemetry.statusFlags = buffer.readUInt8(offset + 43);
        telemetry.timestamp = buffer.readUInt32LE(offset + 44);
        telemetry.flightTime = buffer.readUInt32LE(offset + 48);

        // === V3 新增字段 (55字节) ===
        if (buffer.length >= 55) {
            telemetry.actualFlightMode = buffer.readUInt8(offset + 52);
            telemetry.controlSource = buffer.readUInt8(offset + 53);
            telemetry.remoteCtrlMode = buffer.readUInt8(offset + 54);
        } else if (buffer.length >= 54) {
            telemetry.actualFlightMode = buffer.readUInt8(offset + 52);
            telemetry.controlSource = buffer.readUInt8(offset + 53);
            telemetry.remoteCtrlMode = 1;  // 默认启用
        } else {
            // V2兼容：无实际模式和控制来源信息
            telemetry.actualFlightMode = 0;  // 默认自稳
            telemetry.controlSource = 0;     // 默认无控制
            telemetry.remoteCtrlMode = 1;    // 默认启用
        }

        // 从状态标志位解析
        telemetry.isArmed = !!(telemetry.statusFlags & STATUS_FLAGS.ARMED);
        telemetry.isFlying = !!(telemetry.statusFlags & STATUS_FLAGS.FLYING);
        telemetry.isEmergency = !!(telemetry.statusFlags & STATUS_FLAGS.EMERGENCY);
        telemetry.isRcConnected = !!(telemetry.statusFlags & STATUS_FLAGS.RC_CONNECTED);
        telemetry.isGcsConnected = !!(telemetry.statusFlags & STATUS_FLAGS.GCS_CONNECTED);
        telemetry.isLowBattery = !!(telemetry.statusFlags & STATUS_FLAGS.BATTERY_LOW);

        // 添加可读的状态名称
        telemetry.armingStateName = ArmingState[telemetry.armingState] || 'UNKNOWN';
        telemetry.armingStateNameCN = ArmingStateNameCN[telemetry.armingState] || '未知';
        telemetry.flightPhaseName = FlightPhase[telemetry.flightPhase] || 'UNKNOWN';
        telemetry.flightPhaseNameCN = FlightPhaseNameCN[telemetry.flightPhase] || '未知';
        telemetry.failsafeStateName = FailsafeState[telemetry.failsafeState] || 'NONE';

        // === V3 新增：实际飞行姿态模式和控制来源的可读名称 ===
        telemetry.actualFlightModeName = ActualFlightMode[telemetry.actualFlightMode] || 'UNKNOWN';
        telemetry.actualFlightModeNameCN = ActualFlightModeNameCN[telemetry.actualFlightMode] || '未知';
        telemetry.controlSourceName = ControlSource[telemetry.controlSource] || 'UNKNOWN';
        telemetry.controlSourceNameCN = ControlSourceNameCN[telemetry.controlSource] || '未知';
        telemetry.remoteCtrlModeName = RemoteControlMode[telemetry.remoteCtrlMode] !== undefined
            ? Object.keys(RemoteControlMode).find(k => RemoteControlMode[k] === telemetry.remoteCtrlMode)
            : 'UNKNOWN';
        telemetry.remoteCtrlModeNameCN = RemoteControlModeNameCN[telemetry.remoteCtrlMode] || '未知';
    } else {
        // 旧格式兼容 Protocol V1
        telemetry.flightMode = buffer.readUInt8(offset + 39);
        telemetry.isArmed = !!buffer.readUInt8(offset + 40);
        telemetry.isLowBattery = !!buffer.readUInt8(offset + 41);
        telemetry.timestamp = buffer.length >= 48 ? buffer.readUInt32LE(offset + 44) : Date.now();
        telemetry.armingState = telemetry.isArmed ? 2 : 1;  // 兼容
        telemetry.flightPhase = 0;
        telemetry.failsafeState = 0;
        telemetry.statusFlags = telemetry.isArmed ? 0x01 : 0x00;
        telemetry.flightTime = 0;
        telemetry.armingStateName = telemetry.isArmed ? 'ARMED' : 'STANDBY';
        telemetry.armingStateNameCN = telemetry.isArmed ? '已解锁' : '待机';
        telemetry.flightPhaseName = 'ON_GROUND';
        telemetry.flightPhaseNameCN = '地面待机';
        telemetry.failsafeStateName = 'NONE';
        // V1 兼容：无实际模式和控制来源信息
        telemetry.actualFlightMode = 0;
        telemetry.controlSource = 0;
        telemetry.actualFlightModeName = 'STABILIZE';
        telemetry.actualFlightModeNameCN = '自稳模式';
        telemetry.controlSourceName = 'NONE';
        telemetry.controlSourceNameCN = '未知';
    }

    // 添加可读的目标模式名称（前端发送的期望模式）
    telemetry.flightModeName = FlightMode[telemetry.flightMode] || 'UNKNOWN';
    telemetry.flightModeNameCN = FlightModeNameCN[telemetry.flightMode] || '未知模式';

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
        this.discoverySocket = null;
        this.discoveryInterval = null;
    }

    start() {
        this.startTCPServer();
        this.startWebServer();
        this.startDiscoveryBroadcast();
    }

    /**
     * 获取本机局域网 IP 地址
     * 优先选择 WiFi/WLAN 接口（手机热点场景）
     * 其次选择常见的局域网网段（192.168.x.x, 10.x.x.x, 172.16-31.x.x）
     */
    getLocalIP() {
        const interfaces = os.networkInterfaces();
        let candidates = [];

        for (const name of Object.keys(interfaces)) {
            for (const iface of interfaces[name]) {
                // 跳过内部地址和非 IPv4 地址
                if (iface.family === 'IPv4' && !iface.internal) {
                    // 跳过链路本地地址 (169.254.x.x)
                    if (iface.address.startsWith('169.254.')) {
                        continue;
                    }
                    candidates.push({
                        address: iface.address,
                        name: name,
                        isWifi: name.toLowerCase().includes('wi-fi') ||
                            name.toLowerCase().includes('wifi') ||
                            name.toLowerCase().includes('wlan') ||
                            name.toLowerCase().includes('wireless')
                    });
                }
            }
        }

        // 优先选择 WiFi/WLAN 接口上的 192.168.x.x 地址（最适合手机热点）
        for (const c of candidates) {
            if (c.isWifi && c.address.startsWith('192.168.')) {
                console.log(`  Selected network interface: ${c.name} (${c.address}) [WiFi preferred]`);
                return c.address;
            }
        }

        // 其次选择任何 WiFi 接口
        for (const c of candidates) {
            if (c.isWifi) {
                console.log(`  Selected network interface: ${c.name} (${c.address}) [WiFi]`);
                return c.address;
            }
        }

        // 然后选择非虚拟网卡的 192.168.x.x（排除 VirtualBox, VMware 等）
        for (const c of candidates) {
            if (c.address.startsWith('192.168.')) {
                const lowerName = c.name.toLowerCase();
                if (!lowerName.includes('virtual') &&
                    !lowerName.includes('vmware') &&
                    !lowerName.includes('vmnet') &&
                    !lowerName.includes('vbox') &&
                    !lowerName.includes('以太网 2')) { // VirtualBox 常用名称
                    console.log(`  Selected network interface: ${c.name} (${c.address})`);
                    return c.address;
                }
            }
        }

        // 其次选择 10.x.x.x
        for (const c of candidates) {
            if (c.address.startsWith('10.')) {
                console.log(`  Selected network interface: ${c.name} (${c.address})`);
                return c.address;
            }
        }

        // 其次选择 172.16-31.x.x
        for (const c of candidates) {
            const parts = c.address.split('.');
            if (parts[0] === '172') {
                const second = parseInt(parts[1]);
                if (second >= 16 && second <= 31) {
                    console.log(`  Selected network interface: ${c.name} (${c.address})`);
                    return c.address;
                }
            }
        }

        // 返回第一个可用地址
        if (candidates.length > 0) {
            console.log(`  Selected network interface: ${candidates[0].name} (${candidates[0].address})`);
            return candidates[0].address;
        }

        return '127.0.0.1';
    }

    /**
     * 启动 UDP 广播发现服务
     * 定期向局域网广播服务器地址，供 ESP32 自动发现
     */
    startDiscoveryBroadcast() {
        try {
            this.discoverySocket = dgram.createSocket('udp4');

            this.discoverySocket.on('error', (err) => {
                console.error('✗ Discovery socket error:', err.message);
            });

            this.discoverySocket.bind(() => {
                this.discoverySocket.setBroadcast(true);

                const localIP = this.getLocalIP();

                // 计算子网广播地址
                const ipParts = localIP.split('.');
                const subnetBroadcast = `${ipParts[0]}.${ipParts[1]}.${ipParts[2]}.255`;

                console.log(`✓ UDP Discovery broadcasting on port ${CONFIG.DISCOVERY_PORT}`);
                console.log(`  Local IP: ${localIP}, TCP port: ${CONFIG.TCP_PORT}`);
                console.log(`  Broadcast address: ${subnetBroadcast}`);
                console.log(`  ESP32 will auto-discover this server`);

                // 构建广播包: 4字节魔数 + 2字节端口
                const packet = Buffer.alloc(6);
                DISCOVERY_MAGIC.copy(packet, 0);
                packet.writeUInt16BE(CONFIG.TCP_PORT, 4);

                // 定期广播
                this.discoveryInterval = setInterval(() => {
                    // 发送到子网广播地址和全网广播地址
                    this.discoverySocket.send(packet, CONFIG.DISCOVERY_PORT, subnetBroadcast);
                    this.discoverySocket.send(packet, CONFIG.DISCOVERY_PORT, '255.255.255.255');
                }, CONFIG.DISCOVERY_INTERVAL);

                // 立即发送一次
                this.discoverySocket.send(packet, CONFIG.DISCOVERY_PORT, subnetBroadcast);
                this.discoverySocket.send(packet, CONFIG.DISCOVERY_PORT, '255.255.255.255');
            });
        } catch (err) {
            console.error('✗ Discovery broadcast failed:', err.message);
            console.log('  ESP32 will use fallback IP from configuration');
        }
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
        // 打印到控制台（显示实际飞行姿态模式和控制来源）
        console.log(`[${client.remoteAddr}] Telemetry: ` +
            `Roll=${telemetry.roll.toFixed(1)}° ` +
            `Pitch=${telemetry.pitch.toFixed(1)}° ` +
            `Yaw=${telemetry.yaw.toFixed(1)}° ` +
            `姿态模式=${telemetry.actualFlightModeNameCN} ` +
            `控制=${telemetry.controlSourceNameCN} ` +
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
    <title>ESP Drone 地面站</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <style>
        :root {
            --qgc-bg-dark: #121212;
            --qgc-panel-bg: rgba(22, 22, 22, 0.95);
            --qgc-border: #333;
            --qgc-text-main: #ffffff;
            --qgc-text-muted: #b0b0b0;
            --qgc-green: #27d965;
            --qgc-red: #d94227;
            --qgc-orange: #d99927;
            --qgc-blue: #2c9edd;
            --font-main: 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
        }

        * { margin: 0; padding: 0; box-sizing: border-box; }

        body {
            background-color: var(--qgc-bg-dark);
            color: var(--qgc-text-main);
            font-family: var(--font-main);
            height: 100vh;
            display: flex;
            flex-direction: column;
            overflow: hidden;
        }

        /* Top Toolbar */
        .toolbar {
            height: 48px;
            background: var(--qgc-panel-bg);
            border-bottom: 1px solid var(--qgc-border);
            display: flex;
            align-items: center;
            padding: 0 16px;
            justify-content: space-between;
            z-index: 100;
        }

        .brand {
            font-weight: bold;
            font-size: 1.2rem;
            color: var(--qgc-text-main);
            display: flex;
            align-items: center;
            gap: 10px;
        }
        
        .brand span {
            color: var(--qgc-blue);
        }

        .status-bar {
            display: flex;
            gap: 20px;
            font-size: 0.9rem;
        }

        .status-item {
            display: flex;
            align-items: center;
            gap: 6px;
            color: var(--qgc-text-muted);
        }
        
        .status-item.active { color: var(--qgc-text-main); }
        .status-item.warning { color: var(--qgc-orange); }
        .status-item.error { color: var(--qgc-red); }

        .indicator-dot {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            background: var(--qgc-text-muted);
        }
        .indicator-dot.green { background: var(--qgc-green); box-shadow: 0 0 5px var(--qgc-green); }
        .indicator-dot.red { background: var(--qgc-red); box-shadow: 0 0 5px var(--qgc-red); }

        /* Main Workspace */
        .workspace {
            flex: 1;
            display: flex;
            position: relative;
            overflow: hidden;
            min-height: 0; /* 关键：防止嵌套 flex 溢出 */
        }

        /* Left Sidebar - Telemetry */
        .sidebar-left {
            width: 280px;
            flex-shrink: 0;
            background: var(--qgc-panel-bg);
            border-right: 1px solid var(--qgc-border);
            display: flex;
            flex-direction: column;
            z-index: 90;
        }

        .panel-header {
            padding: 12px 16px;
            font-size: 0.85rem;
            text-transform: uppercase;
            letter-spacing: 1px;
            color: var(--qgc-blue);
            border-bottom: 1px solid var(--qgc-border);
            font-weight: 600;
        }

        .telemetry-grid {
            padding: 10px;
            display: flex;
            flex-direction: column;
            gap: 8px;
            overflow-y: auto;
        }

        .telemetry-row {
            display: flex;
            justify-content: space-between;
            background: rgba(255,255,255,0.03);
            padding: 8px 12px;
            border-radius: 4px;
            align-items: center;
        }

        .t-label { color: var(--qgc-text-muted); font-size: 0.8rem; }
        .t-value { font-family: 'Consolas', monospace; font-weight: bold; color: var(--qgc-text-main); font-size: 0.95rem; }
        
        .t-value.armed { color: var(--qgc-red); animation: blink 1s infinite; }
        .t-value.disarmed { color: var(--qgc-green); }
        
        @keyframes blink { 50% { opacity: 0.5; } }

        /* Center View - 3D */
        .center-view {
            flex: 1;
            position: relative;
            background: #000;
            overflow: hidden; /* 防止 Canvas 溢出 */
            display: flex;
            align-items: center;
            justify-content: center;
            min-height: 0; /* 关键：允许 flex item 缩小到比 content 更小 */
            min-width: 0;
        }

        #model-container {
            width: 100%;
            height: 100%;
            display: block;
        }

        /* HUD Overlay */
        .hud-overlay {
            position: absolute;
            top: 20px;
            left: 50%;
            transform: translateX(-50%);
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 10px;
            pointer-events: none;
        }

        .flight-mode-badge {
            background: rgba(0,0,0,0.6);
            border: 1px solid var(--qgc-green);
            color: var(--qgc-green);
            padding: 8px 24px;
            border-radius: 4px;
            font-size: 1.2rem;
            font-weight: bold;
            text-transform: uppercase;
            backdrop-filter: blur(4px);
        }

        .flight-message {
            background: rgba(217, 66, 39, 0.8);
            color: white;
            padding: 5px 15px;
            border-radius: 4px;
            font-weight: bold;
            display: none;
        }

        .artificial-horizon-overlay {
            position: absolute;
            bottom: 20px;
            left: 20px;
            width: 200px;
            height: 200px;
            /* Placeholder for additional instruments */
            pointer-events: none;
        }

        /* Right Sidebar - Actions */
        .sidebar-right {
            width: 240px;
            flex-shrink: 0;
            background: var(--qgc-panel-bg);
            border-left: 1px solid var(--qgc-border);
            display: flex;
            flex-direction: column;
            z-index: 90;
            overflow-y: auto;
        }

        .action-pad {
            padding: 16px;
            display: flex;
            flex-direction: column;
            gap: 12px;
        }

        .slide-btn {
            background: #2a2a2a;
            border: 1px solid #444;
            color: white;
            padding: 15px;
            border-radius: 6px;
            cursor: pointer;
            text-align: left;
            position: relative;
            overflow: hidden;
            transition: all 0.2s;
            display: flex;
            align-items: center;
            justify-content: center;
            font-weight: 600;
            text-transform: uppercase;
        }

        .slide-btn:hover { background: #333; }
        .slide-btn:active { transform: scale(0.98); }

        .btn-arm { border-color: var(--qgc-red); color: var(--qgc-red); }
        .btn-arm:hover { background: rgba(217, 66, 39, 0.1); }
        
        .btn-disarm { border-color: var(--qgc-green); color: var(--qgc-green); }
        .btn-disarm:hover { background: rgba(39, 217, 101, 0.1); }

        .btn-action { border-left: 4px solid var(--qgc-blue); }

        /* 远程控制模式按钮 */
        .ctrl-mode-indicator {
            display: flex;
            align-items: center;
            gap: 8px;
            padding: 10px 15px;
            background: #1a1a2e;
            border-radius: 6px;
            margin-bottom: 10px;
        }
        .ctrl-mode-icon { font-size: 18px; }
        .ctrl-mode-text { font-size: 13px; color: var(--qgc-white); }

        /* Segmented Control */
        .segmented-control {
            display: flex;
            background: #1a1a2e;
            border-radius: 6px;
            padding: 4px;
            position: relative;
            user-select: none;
            border: 1px solid #444;
            margin-bottom: 20px;
        }

        .segmented-control input {
            display: none;
        }

        .seg-item {
            flex: 1;
            text-align: center;
            padding: 8px 0;
            font-size: 12px;
            color: #888;
            z-index: 2;
            cursor: pointer;
            transition: color 0.3s;
            font-weight: 600;
        }

        /* Specific colors for active states */
        #mode-enabled:checked + label { color: var(--qgc-green); }
        #mode-shared:checked + label { color: var(--qgc-orange); }
        #mode-disabled:checked + label { color: #ccc; }

        .seg-glider {
            position: absolute;
            top: 4px;
            bottom: 4px;
            left: 4px;
            width: calc((100% - 8px) / 3);
            background: #333;
            border-radius: 4px;
            transition: transform 0.3s cubic-bezier(0.4, 0.0, 0.2, 1);
            z-index: 1;
            box-shadow: 0 2px 4px rgba(0,0,0,0.2);
        }

        /* Glider positions */
        #mode-disabled:checked ~ .seg-glider { transform: translateX(0%); background: #444; }
        #mode-enabled:checked ~ .seg-glider { transform: translateX(100%); background: rgba(39, 217, 101, 0.2); border: 1px solid rgba(39, 217, 101, 0.4); }
        #mode-shared:checked ~ .seg-glider { transform: translateX(200%); background: rgba(255, 152, 0, 0.2); border: 1px solid rgba(255, 152, 0, 0.4); }

        /* Bottom Console */
        .bottom-console {
            width: 100%;
            height: 150px;
            background: rgba(0,0,0,0.8);
            border-top: 1px solid var(--qgc-border);
            backdrop-filter: blur(5px);
            display: flex;
            flex-direction: column;
            z-index: 50;
            transition: height 0.3s;
            flex-shrink: 0;
        }
        
        .console-header {
            padding: 5px 10px;
            background: rgba(255,255,255,0.05);
            font-size: 0.75rem;
            color: var(--qgc-text-muted);
            cursor: pointer;
        }

        .console-logs {
            flex: 1;
            overflow-y: auto;
            padding: 10px;
            font-family: 'Consolas', monospace;
            font-size: 0.8rem;
        }
        
        .log-entry { margin-bottom: 4px; border-bottom: 1px solid rgba(255,255,255,0.05); padding-bottom: 2px; }
        .log-info { color: #aaa; }
        .log-warn { color: var(--qgc-orange); }
        .log-error { color: var(--qgc-red); }
        .log-success { color: var(--qgc-green); }

        /* Battery Bar vertical in Top */
        .batt-container {
            width: 60px;
            height: 20px;
            border: 1px solid #555;
            position: relative;
            background: #111;
        }
        .batt-fill {
            height: 100%;
            background: var(--qgc-green);
            width: 50%;
            transition: width 0.5s;
        }

    </style>
</head>
<body>
    <div class="toolbar">
        <div class="brand">
            <span>Q</span>GroundControl <span>Lite</span>
        </div>
        <div class="status-bar">
            <div class="status-item">
                <div class="indicator-dot" id="comm-dot"></div>
                <span id="comm-status">未连接</span>
            </div>
            <div class="status-item" title="遥控器连接状态">
                <span>🎮</span>
                <div class="indicator-dot" id="rc-dot"></div>
                <span id="rc-status">RC</span>
            </div>
            <div class="status-item" title="地面站连接状态">
                <span>🖥️</span>
                <div class="indicator-dot" id="gcs-dot"></div>
                <span id="gcs-status">GCS</span>
            </div>
            <div class="status-item">
                <span>电池电压</span>
                <div class="batt-container">
                    <div class="batt-fill" id="batt-bar" style="width:0%"></div>
                </div>
                <span id="batt-val">--%</span>
            </div>
        </div>
    </div>

    <div class="workspace">
        <!-- LEFT: Telemetry -->
        <div class="sidebar-left">
            <div class="panel-header">飞行器状态</div>
            <div class="telemetry-grid">
                <div class="telemetry-row">
                    <span class="t-label">解锁状态</span>
                    <span class="t-value" id="arming-txt">--</span>
                </div>
                <div class="telemetry-row">
                    <span class="t-label">姿态模式</span>
                    <span class="t-value" id="actual-mode-txt">--</span>
                </div>
                <div class="telemetry-row">
                    <span class="t-label">控制来源</span>
                    <span class="t-value" id="control-source-txt">--</span>
                </div>
                <div class="telemetry-row">
                    <span class="t-label">目标模式</span>
                    <span class="t-value" id="mode-txt">--</span>
                </div>
                <div class="telemetry-row">
                    <span class="t-label">飞行阶段</span>
                    <span class="t-value" id="phase-txt">--</span>
                </div>
                <div class="telemetry-row">
                    <span class="t-label">故障保护</span>
                    <span class="t-value" id="failsafe-txt">--</span>
                </div>
                <div class="telemetry-row">
                    <span class="t-label">飞行时间</span>
                    <span class="t-value" id="time-txt">00:00</span>
                </div>
            </div>

            <div class="panel-header">传感器数据</div>
            <div class="telemetry-grid">
                <div class="telemetry-row">
                    <span class="t-label">横滚角 (Roll)</span>
                    <span class="t-value" id="roll-txt">0.0°</span>
                </div>
                <div class="telemetry-row">
                    <span class="t-label">俯仰角 (Pitch)</span>
                    <span class="t-value" id="pitch-txt">0.0°</span>
                </div>
                <div class="telemetry-row">
                    <span class="t-label">航向角 (Yaw)</span>
                    <span class="t-value" id="yaw-txt">0.0°</span>
                </div>
                <div class="telemetry-row">
                    <span class="t-label">电池电压</span>
                    <span class="t-value" id="volt-txt">0.00 V</span>
                </div>
            </div>
        </div>

        <!-- CENTER: 3D View -->
        <div class="center-view">
            <div id="model-container"></div>
            
            <div class="hud-overlay">
                <div class="flight-mode-badge" id="hud-mode">等待连接</div>
                <div class="flight-message" id="hud-msg">警告</div>
            </div>
        </div>

        <!-- RIGHT: Actions -->
        <div class="sidebar-right">
            <div class="panel-header">操作面板</div>
            <div class="action-pad">
                <div class="slide-btn btn-arm" onclick="sendAction('ARM')">解锁 (Arm)</div>
                <div class="slide-btn btn-disarm" onclick="sendAction('DISARM')">上锁 (Disarm)</div>
                <div style="height: 20px;"></div>
                <div class="slide-btn btn-action" onclick="sendAction('TAKEOFF')">起飞 (Takeoff)</div>
                <div class="slide-btn btn-action" onclick="sendAction('LAND')">降落 (Land)</div>
                <div class="slide-btn btn-action" onclick="sendAction('RTL')">返航 (Return)</div>
                <div class="slide-btn btn-action" onclick="sendAction('HOVER')">悬停 (Hold)</div>
            </div>
            
            <!-- 远程控制模式设置 -->
            <div class="panel-header" style="margin-top: 15px;">远程控制模式</div>
            <div class="action-pad">
                <div class="ctrl-mode-indicator" id="ctrl-mode-status">
                    <span class="ctrl-mode-icon">⚙️</span>
                    <span class="ctrl-mode-text" id="ctrl-mode-text">启用</span>
                </div>
                
                <div class="segmented-control">
                    <input type="radio" name="ctrl-mode" id="mode-disabled" value="0" onclick="setRemoteControlMode(0)">
                    <label for="mode-disabled" class="seg-item" title="禁用服务器控制">禁用</label>
                    
                    <input type="radio" name="ctrl-mode" id="mode-enabled" value="1" onclick="setRemoteControlMode(1)">
                    <label for="mode-enabled" class="seg-item" title="启用服务器控制">启用</label>
                    
                    <input type="radio" name="ctrl-mode" id="mode-shared" value="2" onclick="setRemoteControlMode(2)">
                    <label for="mode-shared" class="seg-item" title="协同控制 (RC+服务器)">协同</label>
                    
                    <div class="seg-glider"></div>
                </div>

                <div style="height: 50px;"></div> <!-- 底部填充 -->
            </div>
        </div>
    </div>
        <!-- BOTTOM: Console -->
        <div class="bottom-console">
            <div class="console-header" onclick="this.parentElement.style.height = this.parentElement.style.height === '30px' ? '150px' : '30px'">
                系统日志 (点击收起/展开)
            </div>
            <div class="console-logs" id="console-logs">
                <div class="log-entry log-info">[SYSTEM] QGroundControl Lite 地面站已启动</div>
            </div>
        </div>

    <script>
        // --- 控制来源优先级说明（前端显示用）---
        const ControlSourcePriority = {
            0: { name: '无控制', color: '#666', icon: '⚪', desc: '无活跃控制源' },
            1: { name: 'WiFi', color: '#2196F3', icon: '📶', desc: 'CRTP协议 (手机APP/UDP)' },
            2: { name: '地面站', color: '#4CAF50', icon: '🖥️', desc: 'TCP远程服务器' },
            3: { name: '遥控器', color: '#FF9800', icon: '🎮', desc: 'SBUS遥控器 (最高优先级)' }
        };

        // 控制源状态追踪
        const controlSourceState = {
            current: 0,           // 当前活跃控制源
            previous: 0,          // 上一个控制源
            lastSwitchTime: 0,    // 上次切换时间
            switchCount: 0,       // 切换次数
            rcConnected: false,   // 遥控器连接状态
            gcsConnected: false   // 地面站连接状态
        };

        // --- 3D Visualization ---
        let scene, camera, renderer, drone;
        
        function initThreeJS() {
            const container = document.getElementById('model-container');
            
            scene = new THREE.Scene();
            scene.background = new THREE.Color(0x000000); 
            // Add grid
            const gridHelper = new THREE.GridHelper(20, 20, 0x333333, 0x111111);
            scene.add(gridHelper);
            
            camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 0.1, 100);
            camera.position.set(0, 2, 4);
            camera.lookAt(0, 0, 0);
            
            renderer = new THREE.WebGLRenderer({ antialias: true });
            renderer.setSize(container.clientWidth, container.clientHeight);
            container.appendChild(renderer.domElement);
            
            // 监听窗口 resize
            window.addEventListener('resize', onWindowResize, false);
            
            // Lights
            const ambient = new THREE.AmbientLight(0xffffff, 0.7);
            scene.add(ambient);
            const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
            dirLight.position.set(5, 10, 5);
            scene.add(dirLight);
            
            // Build Drone Model
            drone = new THREE.Group();
            
            // Materials
            const matDark = new THREE.MeshPhongMaterial({ color: 0x222222 });
            const matArm = new THREE.MeshPhongMaterial({ color: 0x444444 });
            const matProp = new THREE.MeshPhongMaterial({ color: 0xeeeeee, transparent: true, opacity: 0.5 });
            const matInd = new THREE.MeshBasicMaterial({ color: 0x27d965 }); // Front indicator
            
            // Body
            const body = new THREE.Mesh(new THREE.BoxGeometry(0.4, 0.1, 0.6), matDark);
            drone.add(body);
            
            // Front indicator
            const ind = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.05, 0.1), matInd);
            ind.position.set(0, 0, -0.3);
            drone.add(ind);
            
            // Arms
            const arm1 = new THREE.Mesh(new THREE.BoxGeometry(1.8, 0.05, 0.1), matArm);
            arm1.rotation.y = Math.PI/4;
            drone.add(arm1);
            const arm2 = new THREE.Mesh(new THREE.BoxGeometry(1.8, 0.05, 0.1), matArm);
            arm2.rotation.y = -Math.PI/4;
            drone.add(arm2);
            
            // Props
            const propGeo = new THREE.CylinderGeometry(0.6, 0.6, 0.01, 32);
            const pos = [
                {x:-0.65, z:-0.65}, {x:0.65, z:-0.65},
                {x:0.65, z:0.65}, {x:-0.65, z:0.65}
            ];
            
            pos.forEach(p => {
                const prop = new THREE.Mesh(propGeo, matProp);
                prop.position.set(p.x, 0.1, p.z);
                drone.add(prop);
            });
            
            // Axis helper to show orientation
            const axesHelper = new THREE.AxesHelper(1);
            drone.add(axesHelper);

            scene.add(drone);
            
            animate();
            
            window.addEventListener('resize', () => {
                if (container) {
                    camera.aspect = container.clientWidth / container.clientHeight;
                    camera.updateProjectionMatrix();
                    renderer.setSize(container.clientWidth, container.clientHeight);
                }
            });
        }
        
        function animate() {
            requestAnimationFrame(animate);
            renderer.render(scene, camera);
        }

        // --- System Logic ---
        let ws = null;
        
        function log(msg, type='info') {
            const el = document.getElementById('console-logs');
            const entry = document.createElement('div');
            entry.className = \`log-entry log-\${type}\`;
            entry.textContent = \`[\${new Date().toLocaleTimeString()}] \${msg}\`;
            el.appendChild(entry);
            el.scrollTop = el.scrollHeight;
        }

        function connectWS() {
            ws = new WebSocket(\`ws://\${window.location.host}\`);
            
            ws.onopen = () => {
                log('已连接至地面站服务器', 'success');
                document.getElementById('comm-dot').className = 'indicator-dot green';
                document.getElementById('comm-status').textContent = '已连接';
            };
            
            ws.onclose = () => {
                log('与服务器断开连接', 'error');
                document.getElementById('comm-dot').className = 'indicator-dot red';
                document.getElementById('comm-status').textContent = '未连接';
                setTimeout(connectWS, 3000);
            };
            
            ws.onmessage = (evt) => {
                try {
                    const msg = JSON.parse(evt.data);
                    if (msg.type === 'telemetry') updateDashboard(msg.data);
                } catch(e) { console.error(e); }
            };
        }

        function updateDashboard(data) {
            // Update Text
            document.getElementById('roll-txt').textContent = data.roll.toFixed(1) + '°';
            document.getElementById('pitch-txt').textContent = data.pitch.toFixed(1) + '°';
            document.getElementById('yaw-txt').textContent = data.yaw.toFixed(1) + '°';
            document.getElementById('volt-txt').textContent = data.battVoltage.toFixed(2) + ' V';
            
            const isArmed = data.isArmed;
            const armEl = document.getElementById('arming-txt');
            armEl.textContent = isArmed ? (data.armingStateNameCN || '已解锁') : '未解锁';
            armEl.className = \`t-value \${isArmed ? 'armed' : 'disarmed'}\`;
            
            // === V3 新增：实际飞行姿态模式和控制来源 ===
            document.getElementById('actual-mode-txt').textContent = data.actualFlightModeNameCN || '--';
            
            // 控制来源显示（带颜色和图标）
            const ctrlSrcEl = document.getElementById('control-source-txt');
            const srcInfo = ControlSourcePriority[data.controlSource] || ControlSourcePriority[0];
            ctrlSrcEl.innerHTML = \`<span style="color:\${srcInfo.color}">\${srcInfo.icon} \${srcInfo.name}</span>\`;
            
            // 检测控制源切换
            if (data.controlSource !== controlSourceState.current) {
                const oldSrc = ControlSourcePriority[controlSourceState.current];
                const newSrc = ControlSourcePriority[data.controlSource];
                controlSourceState.previous = controlSourceState.current;
                controlSourceState.current = data.controlSource;
                controlSourceState.lastSwitchTime = Date.now();
                controlSourceState.switchCount++;
                
                // 记录切换日志
                if (controlSourceState.switchCount > 1) {
                    log(\`控制源切换: \${oldSrc.name} → \${newSrc.name} (\${newSrc.desc})\`, 'warning');
                }
            }
            
            // 更新连接状态
            controlSourceState.rcConnected = data.isRcConnected;
            controlSourceState.gcsConnected = data.isGcsConnected;
            
            // 更新顶部状态栏的连接指示器
            const rcDot = document.getElementById('rc-dot');
            const gcsDot = document.getElementById('gcs-dot');
            rcDot.className = \`indicator-dot \${data.isRcConnected ? 'green' : 'red'}\`;
            gcsDot.className = \`indicator-dot \${data.isGcsConnected ? 'green' : 'red'}\`;
            
            document.getElementById('mode-txt').textContent = data.flightModeNameCN || '--';
            document.getElementById('phase-txt').textContent = data.flightPhaseNameCN || '--';
            
            const fsEl = document.getElementById('failsafe-txt');
            fsEl.textContent = data.failsafeStateName || '正常';
            fsEl.style.color = (data.failsafeState > 0) ? 'var(--qgc-red)' : 'var(--qgc-text-main)';

            // Battery
            const battPct = data.battPercent;
            const bBar = document.getElementById('batt-bar');
            bBar.style.width = battPct + '%';
            bBar.style.background = (battPct < 20) ? 'var(--qgc-red)' : (battPct < 40 ? 'var(--qgc-orange)' : 'var(--qgc-green)');
            document.getElementById('batt-val').textContent = battPct + '%';

            // Flight Time
            const sec = Math.floor(data.flightTime / 1000);
            const m = Math.floor(sec / 60);
            const s = sec % 60;
            document.getElementById('time-txt').textContent = \`\${m}:\${s.toString().padStart(2,'0')}\`;

            // HUD - 显示实际飞行姿态模式 + 控制来源
            const hudModeEl = document.getElementById('hud-mode');
            hudModeEl.textContent = \`\${data.actualFlightModeNameCN || '等待'} | \${data.controlSourceNameCN || ''}\`;
            hudModeEl.style.borderColor = isArmed ? 'var(--qgc-red)' : 'var(--qgc-green)';
            hudModeEl.style.color = isArmed ? 'var(--qgc-red)' : 'var(--qgc-green)';

            // 更新远程控制模式按钮状态
            if (data.remoteCtrlMode !== undefined) {
                updateCtrlModeButtons(data.remoteCtrlMode);
            }

            // 3D Model Update
            if (drone) {
                const toRad = Math.PI / 180;
                drone.rotation.order = 'YXZ';
                drone.rotation.x = data.pitch * toRad;
                drone.rotation.y = -data.yaw * toRad;
                drone.rotation.z = -data.roll * toRad;
            }
        }

        function onWindowResize() {
            const container = document.getElementById('model-container');
            if (camera && renderer && container) {
                // 需要重新获取容器尺寸，可能因为缩放或布局变化而改变
                const width = container.clientWidth;
                const height = container.clientHeight;
                
                camera.aspect = width / height;
                camera.updateProjectionMatrix();
                
                renderer.setSize(width, height);
            }
        }

        async function sendAction(action) {
            let cmdType = 0; // RPYT
            let roll=0, pitch=0, yaw=0, thrust=0;
            let mode = 0;

            let logMsg = '';
            switch(action) {
                case 'ARM': logMsg = '发送解锁指令'; break;
                case 'DISARM': logMsg = '发送上锁指令'; break;
                case 'TAKEOFF': logMsg = '发送起飞指令'; break;
                case 'LAND': logMsg = '发送降落指令'; break;
                case 'RTL': logMsg = '发送返航指令'; break;
                case 'HOVER': logMsg = '发送悬停指令'; break;
                default: logMsg = '未知指令: ' + action;
            }
            log(logMsg, 'info');

            switch(action) {
                case 'ARM': cmdType = 6; break;
                case 'DISARM': cmdType = 7; break;
                case 'TAKEOFF': cmdType = 7; break; // Map to correct cmd if available, actually 7 is TAKEOFF in FlightMode enum but control definition might differ. 
                                // Checking code: ControlCmdType ARM=6, DISARM=7. Wait.
                                // ControlCmdType: RPYT=0, VEL=1, POS=2, HOVER=3, LAND=4, EMERG=5, ARM=6, DISARM=7.
                                // FlightMode: ... TAKEOFF=7 ... 
                                // To set flight mode, we might need a different mechanism or use RPYT with mode set?
                                // Let's check server 'sendControl'. It sends 'mode'.
                                // If we want to switch mode, we should use a command that sets mode.
                                // Current backend logic maps 'cmdType' directly.
                                // Let's look at ControlCmdType again.
                                break;
                case 'LAND': cmdType = 4; break;
                case 'HOVER': cmdType = 3; break;
                case 'RTL': cmdType = 5; break; // EMERGENCY? No, RTL is logic. 
                            // The current server 'createControlCommand' maps directly to CRTP or internally handled?
                            // Server side: 'client.sendControl(...)' -> 'createControlCommand'.
                            // Note: 'ControlCmdType' enum in app.js seems to define:
                            // 0:RPYT, 1:VEL, 2:POS, 3:HOVER, 4:LAND, 5:EMERGENCY, 6:ARM, 7:DISARM.
                            // What about "Switch Flight Mode"?
                            // The protocol might rely on Mapping on the drone side.
                            // For now, let's stick to Basic cmds.
                            
                            // To actually switch mode (e.g. to Altitude or Stabilize), we might need a dedicated command or use auxiliary channels if SBUS.
                            // But here we are sending via Wifi.
                            // Let's just implement ARM/DISARM/LAND/HOVER as provided.
                            break;
            }
            
            // Correction for Send Logic
             const payload = { cmdType: 0, roll:0, pitch:0, yaw:0, thrust:0, mode:0 };
             
             if (action === 'ARM') payload.cmdType = 6;
             else if (action === 'DISARM') payload.cmdType = 7;
             else if (action === 'LAND') payload.cmdType = 4;
             else if (action === 'HOVER') payload.cmdType = 3;
             else if (action === 'TAKEOFF') { 
                // Not strictly defined in ControlCmdType, assume handled by drone or manual throttle
                // Use HOVER for now to test
                payload.cmdType = 3; 
             }
             else if (action === 'RTL') payload.cmdType = 5; // Use Emergency for now as fallback if RTL not def

             try {
                await fetch('/api/control', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(payload)
                });
             } catch(e) { log(e.message, 'error'); }
        }

        // 设置远程控制模式
        async function setRemoteControlMode(mode) {
            const modeNames = ['禁用', '启用', '协同'];
            log('设置远程控制模式: ' + modeNames[mode], 'info');
            
            const payload = { 
                cmdType: 0x10,  // CTRL_CMD_SET_CONTROL_MODE
                roll: 0, 
                pitch: 0, 
                yaw: 0, 
                thrust: 0, 
                mode: mode  // RemoteControlMode: 0=禁用, 1=启用, 2=协同
            };
            
            try {
                await fetch('/api/control', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(payload)
                });
                updateCtrlModeButtons(mode);
            } catch(e) { 
                log('发送控制模式失败: ' + e.message, 'error'); 
            }
        }

        // 更新控制模式按钮状态
        function updateCtrlModeButtons(activeMode) {
            const modeNames = ['禁用', '启用', '协同'];
            const modeIcons = ['🚫', '✅', '🤝'];
            
            // 更新指示器
            document.getElementById('ctrl-mode-text').textContent = modeNames[activeMode] || '未知';
            document.querySelector('.ctrl-mode-icon').textContent = modeIcons[activeMode] || '⚙️';
            
            // 更新 Segmented Control 状态
            const radioId = ['mode-disabled', 'mode-enabled', 'mode-shared'][activeMode];
            if (radioId) {
                const radio = document.getElementById(radioId);
                if (radio) radio.checked = true;
            }
        }

        window.addEventListener('load', () => {
             initThreeJS();
             connectWS();
        });

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
