/**
 * ESP-Drone 服务器核心
 * ==========================
 * TCP/HTTP/WebSocket 服务器管理
 */

const net = require('net');
const http = require('http');
const path = require('path');
const express = require('express');
const WebSocket = require('ws');
const dgram = require('dgram');
const os = require('os');

const CONFIG = require('./config');
const { DISCOVERY_MAGIC, ControlCmdType } = require('./protocol');
const DroneClient = require('./DroneClient');

// CRTP 参数写入（按名称）常量
const CRTP_PORT_PARAM = 0x02;
const CRTP_CH_MISC = 0x03;
const CRTP_HEADER_PARAM_MISC = (CRTP_PORT_PARAM << 4) | CRTP_CH_MISC;
const MISC_SETBYNAME = 0x00;

// Param 类型常量（与 param.h 保持一致）
const PARAM_UINT8 = 0x08;  // PARAM_1BYTE | PARAM_TYPE_INT | PARAM_UNSIGNED
const PARAM_UINT16 = 0x09; // PARAM_2BYTES | PARAM_TYPE_INT | PARAM_UNSIGNED

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

    buildParamSetByNamePacket(group, name, type, value, valueSize) {
        const groupBuf = Buffer.from(group, 'utf8');
        const nameBuf = Buffer.from(name, 'utf8');

        const totalLen = 1 + 1 + groupBuf.length + 1 + nameBuf.length + 1 + 1 + valueSize;
        if (totalLen > 31) {
            throw new Error('CRTP param packet too long');
        }

        const buf = Buffer.alloc(totalLen);
        let offset = 0;

        // CRTP header
        buf[offset++] = CRTP_HEADER_PARAM_MISC;
        // MISC_SETBYNAME
        buf[offset++] = MISC_SETBYNAME;

        // group\0
        groupBuf.copy(buf, offset);
        offset += groupBuf.length;
        buf[offset++] = 0x00;

        // name\0
        nameBuf.copy(buf, offset);
        offset += nameBuf.length;
        buf[offset++] = 0x00;

        // type
        buf[offset++] = type;

        // value (little-endian)
        if (valueSize === 1) {
            buf.writeUInt8(value & 0xFF, offset);
        } else if (valueSize === 2) {
            buf.writeUInt16LE(value & 0xFFFF, offset);
        } else {
            throw new Error('Unsupported param size');
        }

        return buf;
    }

    start() {
        this.startTCPServer();
        this.startWebServer();
        this.startDiscoveryBroadcast();
    }

    /**
     * 获取本机局域网 IP 地址
     */
    getLocalIP() {
        const interfaces = os.networkInterfaces();
        let candidates = [];

        for (const name of Object.keys(interfaces)) {
            for (const iface of interfaces[name]) {
                if (iface.family === 'IPv4' && !iface.internal) {
                    if (iface.address.startsWith('169.254.')) continue;
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

        // 优先选择 WiFi/WLAN 接口上的 192.168.x.x 地址
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

        // 然后选择非虚拟网卡的 192.168.x.x
        for (const c of candidates) {
            if (c.address.startsWith('192.168.')) {
                const lowerName = c.name.toLowerCase();
                if (!lowerName.includes('virtual') && !lowerName.includes('vmware') &&
                    !lowerName.includes('vmnet') && !lowerName.includes('vbox')) {
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

        // 返回第一个可用地址
        if (candidates.length > 0) {
            console.log(`  Selected network interface: ${candidates[0].name} (${candidates[0].address})`);
            return candidates[0].address;
        }

        return '127.0.0.1';
    }

    /**
     * 启动 UDP 广播发现服务
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
                const ipParts = localIP.split('.');
                const subnetBroadcast = `${ipParts[0]}.${ipParts[1]}.${ipParts[2]}.255`;

                console.log(`✓ UDP Discovery broadcasting on port ${CONFIG.DISCOVERY_PORT}`);
                console.log(`  Local IP: ${localIP}, TCP port: ${CONFIG.TCP_PORT}`);
                console.log(`  Broadcast address: ${subnetBroadcast}`);

                const packet = Buffer.alloc(6);
                DISCOVERY_MAGIC.copy(packet, 0);
                packet.writeUInt16BE(CONFIG.TCP_PORT, 4);

                this.discoveryInterval = setInterval(() => {
                    this.discoverySocket.send(packet, CONFIG.DISCOVERY_PORT, subnetBroadcast);
                    this.discoverySocket.send(packet, CONFIG.DISCOVERY_PORT, '255.255.255.255');
                }, CONFIG.DISCOVERY_INTERVAL);

                this.discoverySocket.send(packet, CONFIG.DISCOVERY_PORT, subnetBroadcast);
                this.discoverySocket.send(packet, CONFIG.DISCOVERY_PORT, '255.255.255.255');
            });
        } catch (err) {
            console.error('✗ Discovery broadcast failed:', err.message);
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
        const publicPath = path.join(__dirname, '..', 'public');

        // 静态文件服务
        app.use(express.static(publicPath));
        app.use(express.json());

        // API 端点
        this.setupAPIRoutes(app);

        // 主页面
        app.get('/', (req, res) => {
            res.sendFile(path.join(publicPath, 'index.html'));
        });

        // HTTP 服务器
        this.httpServer = http.createServer(app);

        // WebSocket 服务器
        this.wss = new WebSocket.Server({ server: this.httpServer });
        this.setupWebSocket();

        this.httpServer.listen(CONFIG.HTTP_PORT, () => {
            console.log(`✓ HTTP/WebSocket Server listening on port ${CONFIG.HTTP_PORT}`);
            console.log(`  Open http://localhost:${CONFIG.HTTP_PORT} in your browser`);
        });
    }

    setupAPIRoutes(app) {
        // 状态查询
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

            console.log(`[API] /api/control - cmdType: ${cmdType}, mode: ${mode}`);

            if (this.clients.size === 0) {
                return res.status(400).json({ error: 'No drone connected' });
            }

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

        // 电机测试 API
        app.post('/api/motor/set', (req, res) => {
            const { motorId, thrust } = req.body;
            console.log(`[API] /api/motor/set - motorId: ${motorId}, thrust: ${thrust}`);

            if (this.clients.size === 0) {
                return res.status(400).json({ error: 'No drone connected' });
            }

            const safeMotorId = Math.max(0, Math.min(3, Number(motorId) || 0));
            const safeThrust = Math.max(0, Math.min(65535, Number(thrust) || 0));

            console.log(`[API] Sending motor command: id=${safeMotorId}, thrust=${safeThrust}`);

            for (const client of this.clients.values()) {
                client.handleMotorCommand(safeMotorId, safeThrust);

                // motortest.id (uint8)
                const pktId = this.buildParamSetByNamePacket('motortest', 'id', PARAM_UINT8, safeMotorId, 1);
                client.sendCRTP(pktId);

                // motortest.thrust (uint16)
                const pktThrust = this.buildParamSetByNamePacket('motortest', 'thrust', PARAM_UINT16, safeThrust, 2);
                client.sendCRTP(pktThrust);

                // motortest.mode = 2 (单电机测试)
                const pktMode = this.buildParamSetByNamePacket('motortest', 'mode', PARAM_UINT8, 2, 1);
                client.sendCRTP(pktMode);
            }
            res.json({ success: true });
        });

        app.post('/api/motor/estop', (req, res) => {
            if (this.clients.size === 0) {
                return res.status(400).json({ error: 'No drone connected' });
            }

            for (const client of this.clients.values()) {
                // motortest.estop = 1
                client.sendCRTP(this.buildParamSetByNamePacket('motortest', 'estop', PARAM_UINT8, 1, 1));
                // motortest.mode = 0
                client.sendCRTP(this.buildParamSetByNamePacket('motortest', 'mode', PARAM_UINT8, 0, 1));
            }
            res.json({ success: true });
        });

        // 全部电机同时设置 (mode=3)
        app.post('/api/motor/all', (req, res) => {
            const { thrust } = req.body || {};
            if (this.clients.size === 0) {
                return res.status(400).json({ error: 'No drone connected' });
            }

            const safeThrust = Math.max(0, Math.min(65535, Number(thrust) || 0));

            for (const client of this.clients.values()) {
                // 设置推力
                client.sendCRTP(this.buildParamSetByNamePacket('motortest', 'thrust', PARAM_UINT16, safeThrust, 2));
                // motortest.mode = 3 (全部电机同时测试)
                client.sendCRTP(this.buildParamSetByNamePacket('motortest', 'mode', PARAM_UINT8, 3, 1));
            }
            res.json({ success: true });
        });

        app.post('/api/motor/sequential', (req, res) => {
            const { thrust } = req.body || {};
            if (this.clients.size === 0) {
                return res.status(400).json({ error: 'No drone connected' });
            }

            const safeThrust = Math.max(0, Math.min(65535, Number(thrust) || 0));

            for (const client of this.clients.values()) {
                // 可选：设置推力值（当前固件顺序测试使用固定值）
                if (safeThrust > 0) {
                    client.sendCRTP(this.buildParamSetByNamePacket('motortest', 'thrust', PARAM_UINT16, safeThrust, 2));
                }
                // motortest.mode = 1
                client.sendCRTP(this.buildParamSetByNamePacket('motortest', 'mode', PARAM_UINT8, 1, 1));
            }
            res.json({ success: true });
        });
    }

    setupWebSocket() {
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
    }

    removeClient(client) {
        this.clients.delete(client.remoteAddr);
    }

    onTelemetry(client, telemetry) {
        // 限制日志输出频率（每秒一次）
        const now = Date.now();
        if (!client.lastTelemetryLog || now - client.lastTelemetryLog >= 1000) {
            client.lastTelemetryLog = now;
            console.log(`[${client.remoteAddr}] Telemetry: ` +
                `Roll=${telemetry.roll.toFixed(1)}° ` +
                `Pitch=${telemetry.pitch.toFixed(1)}° ` +
                `Yaw=${telemetry.yaw.toFixed(1)}° ` +
                `姿态模式=${telemetry.actualFlightModeNameCN} ` +
                `控制=${telemetry.controlSourceNameCN} ` +
                `Armed=${telemetry.isArmed} ` +
                `Batt=${telemetry.battPercent}%`
            );
        }

        // 添加电机输出数据（优先使用遥测数据）
        if (!telemetry.motorOutputs || telemetry.motorOutputs.length !== 4) {
            telemetry.motorOutputs = [
                client.motorOutputs[0] || 0,
                client.motorOutputs[1] || 0,
                client.motorOutputs[2] || 0,
                client.motorOutputs[3] || 0
            ];
        }

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

    stop() {
        if (this.discoveryInterval) {
            clearInterval(this.discoveryInterval);
        }
        if (this.discoverySocket) {
            this.discoverySocket.close();
        }
        if (this.tcpServer) {
            this.tcpServer.close();
        }
        if (this.httpServer) {
            this.httpServer.close();
        }
        console.log('Server stopped');
    }
}

module.exports = DroneServer;
