/**
 * ESP-Drone 服务器核心
 * ==========================
 * TCP/HTTP/WebSocket 服务器管理
 */

const net = require('net');
const http = require('http');
const fs = require('fs');
const path = require('path');
const express = require('express');
const WebSocket = require('ws');
const dgram = require('dgram');
const os = require('os');

const CONFIG = require('./config');
const { DISCOVERY_MAGIC, UDP_TELEMETRY_MAGIC, UDP_TELEMETRY_HEADER_SIZE, ControlCmdType } = require('./protocol');
const { parseTelemetry } = require('./parser');
const DroneClient = require('./DroneClient');

// CRTP 参数写入（按名称）常量
const CRTP_PORT_PARAM = 0x02;
const CRTP_CH_MISC = 0x03;
const CRTP_HEADER_PARAM_MISC = (CRTP_PORT_PARAM << 4) | CRTP_CH_MISC;
const MISC_SETBYNAME = 0x00;

// Param 类型常量（与 param.h 保持一致）
const PARAM_UINT8 = 0x08;  // PARAM_1BYTE | PARAM_TYPE_INT | PARAM_UNSIGNED
const PARAM_UINT16 = 0x09; // PARAM_2BYTES | PARAM_TYPE_INT | PARAM_UNSIGNED
const PARAM_FLOAT = 0x06;  // PARAM_4BYTES | PARAM_TYPE_FLOAT
const ALLOWED_PID_GROUPS = new Set(['pid_attitude', 'pid_rate', 'velCtlPid', 'posCtlPid']);
const PID_PROFILE_VERSION = 1;

class DroneServer {
    constructor() {
        this.clients = new Map();          // key: remoteAddr (ip:port)
        this.clientsByIP = new Map();      // key: droneIP, 用于 UDP 包匹配
        this.webClients = new Set();
        this.tcpServer = null;
        this.httpServer = null;
        this.wss = null;
        this.discoverySocket = null;
        this.discoveryInterval = null;
        this.wsPingTimer = null;
        this.pidProfile = new Map();       // key: "group.name", value: float
        this.pidAutoApplyDelayMs = 800;
        this.pidProfilePath = path.join(__dirname, '..', 'data', 'pid_profile.json');
        this.udpTelemetrySocket = null;
        this.vofaSocket = null;            // vofa+ UDP socket
        this.vofaEnabled = true;           // vofa+ 默认启用
        this.vofaClients = new Map();      // vofa+ 客户端 Map<key, {ip, port}>
        this.loadPidProfile();
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
        } else if (valueSize === 4 && type === PARAM_FLOAT) {
            buf.writeFloatLE(value, offset);
        } else if (valueSize === 4) {
            buf.writeUInt32LE(value & 0xFFFFFFFF, offset);
        } else {
            throw new Error('Unsupported param size');
        }

        return buf;
    }

    normalizePidParam(group, name, value) {
        if (!ALLOWED_PID_GROUPS.has(group)) {
            return { ok: false, error: `Invalid param group: ${group}` };
        }
        if (!name || typeof name !== 'string') {
            return { ok: false, error: 'Invalid param name' };
        }

        const floatValue = parseFloat(value);
        if (!Number.isFinite(floatValue)) {
            return { ok: false, error: `Invalid float value: ${value}` };
        }

        return { ok: true, group, name, value: floatValue };
    }

    getPidProfileObject() {
        const out = {};
        for (const [key, value] of this.pidProfile.entries()) {
            out[key] = value;
        }
        return out;
    }

    getPidProfileEntries() {
        const out = [];
        for (const [key, value] of this.pidProfile.entries()) {
            const dot = key.indexOf('.');
            if (dot <= 0) continue;
            out.push({ group: key.slice(0, dot), name: key.slice(dot + 1), value });
        }
        out.sort((a, b) => `${a.group}.${a.name}`.localeCompare(`${b.group}.${b.name}`));
        return out;
    }

    loadPidProfile() {
        try {
            if (!fs.existsSync(this.pidProfilePath)) return;

            const raw = fs.readFileSync(this.pidProfilePath, 'utf8');
            const parsed = JSON.parse(raw);
            const nextMap = new Map();

            if (parsed && parsed.params && typeof parsed.params === 'object' && !Array.isArray(parsed.params)) {
                for (const key of Object.keys(parsed.params)) {
                    const dot = key.indexOf('.');
                    if (dot <= 0) continue;
                    const normalized = this.normalizePidParam(key.slice(0, dot), key.slice(dot + 1), parsed.params[key]);
                    if (!normalized.ok) continue;
                    nextMap.set(`${normalized.group}.${normalized.name}`, normalized.value);
                }
            }

            this.pidProfile = nextMap;
            console.log(`[PID] Loaded profile: ${this.pidProfile.size} params`);
        } catch (err) {
            console.error('[PID] Failed to load profile:', err.message);
        }
    }

    savePidProfile() {
        try {
            const dir = path.dirname(this.pidProfilePath);
            fs.mkdirSync(dir, { recursive: true });
            const payload = {
                version: PID_PROFILE_VERSION,
                updatedAt: new Date().toISOString(),
                params: this.getPidProfileObject()
            };
            fs.writeFileSync(this.pidProfilePath, JSON.stringify(payload, null, 2), 'utf8');
        } catch (err) {
            console.error('[PID] Failed to save profile:', err.message);
        }
    }

    mergePidProfileEntries(entries) {
        let merged = 0;
        for (const item of entries) {
            const normalized = this.normalizePidParam(item.group, item.name, item.value);
            if (!normalized.ok) continue;
            this.pidProfile.set(`${normalized.group}.${normalized.name}`, normalized.value);
            merged++;
        }
        if (merged > 0) this.savePidProfile();
        return merged;
    }

    applyPidProfileToClient(client) {
        const entries = this.getPidProfileEntries();
        if (!client || entries.length === 0) return 0;

        let applied = 0;
        for (const p of entries) {
            try {
                const pkt = this.buildParamSetByNamePacket(p.group, p.name, PARAM_FLOAT, p.value, 4);
                client.sendCRTP(pkt);
                applied++;
            } catch (err) {
                console.error(`[PID] Apply failed (${p.group}.${p.name}): ${err.message}`);
            }
        }

        console.log(`[PID] Auto-applied ${applied}/${entries.length} params to ${client.remoteAddr}`);
        return applied;
    }


    start() {
        this.startTCPServer();
        this.startWebServer();
        this.startDiscoveryBroadcast();
        this.startUDPTelemetryServer();
        this.startVofaServer();
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

            this.discoverySocket.on('message', (msg, rinfo) => {
                if (!msg || msg.length < 6) return;

                // 检查魔数
                if (!DISCOVERY_MAGIC.equals(msg.subarray(0, 4))) return;

                const reqPort = msg.readUInt16BE(4);

                // TCP端口=0 视为发现请求，单播回应
                // 回应格式: [4字节 ESPD] + [2字节 TCP端口] + [2字节 UDP遥测端口]
                if (reqPort === 0) {
                    const resp = Buffer.alloc(8);
                    DISCOVERY_MAGIC.copy(resp, 0);
                    resp.writeUInt16BE(CONFIG.TCP_PORT, 4);
                    resp.writeUInt16BE(CONFIG.UDP_TELEMETRY_PORT, 6);

                    this.discoverySocket.send(resp, rinfo.port, rinfo.address, (err) => {
                        if (err) {
                            console.error('✗ Discovery response failed:', err.message);
                        }
                    });
                }
            });

            this.discoverySocket.bind(CONFIG.DISCOVERY_PORT, () => {
                this.discoverySocket.setBroadcast(true);

                const localIP = this.getLocalIP();
                const ipParts = localIP.split('.');
                const subnetBroadcast = `${ipParts[0]}.${ipParts[1]}.${ipParts[2]}.255`;

                console.log(`✓ UDP Discovery broadcasting on port ${CONFIG.DISCOVERY_PORT}`);
                console.log(`  Local IP: ${localIP}, TCP port: ${CONFIG.TCP_PORT}, UDP telemetry port: ${CONFIG.UDP_TELEMETRY_PORT}`);
                console.log(`  Broadcast address: ${subnetBroadcast}`);

                // 广播包 8 字节: [4字节 ESPD] + [2字节 TCP端口] + [2字节 UDP遥测端口]
                const packet = Buffer.alloc(8);
                DISCOVERY_MAGIC.copy(packet, 0);
                packet.writeUInt16BE(CONFIG.TCP_PORT, 4);
                packet.writeUInt16BE(CONFIG.UDP_TELEMETRY_PORT, 6);

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
            // 同时按 IP 建立索引，供 UDP 包匹配
            this.clientsByIP.set(client.droneIP, client);

            setTimeout(() => {
                if (this.clients.has(remoteAddr)) {
                    this.applyPidProfileToClient(client);
                }
            }, this.pidAutoApplyDelayMs);
        });

        this.tcpServer.listen(CONFIG.TCP_PORT, () => {
            console.log(`✓ TCP Server listening on port ${CONFIG.TCP_PORT}`);
        });

        this.tcpServer.on('error', (err) => {
            console.error('TCP Server error:', err.message);
        });
    }

    /**
     * 根据 IP 地址匹配已连接的飞控客户端
     */
    findClientByIP(ip) {
        const normalizedIP = ip.replace(/^::ffff:/, '');
        return this.clientsByIP.get(normalizedIP) || null;
    }

    /**
     * 启动 UDP 遥测服务器
     * 飞控向此端口发送高频遥测包（姿态角/电机输出/陀螺仪等）
     * UDP 包格式: [4字节 ESPU 魔数] + [1字节类型] + [1字节序号] + [payload]
     */
    /**
     * 启动 vofa+ UDP 转发服务
     * 接收 vofa+ 客户端注册请求，将遥测数据通过 FireWater 协议转发
     */
    startVofaServer() {
        try {
            this.vofaSocket = dgram.createSocket('udp4');

            this.vofaSocket.on('error', (err) => {
                console.error('✗ vofa+ UDP socket error:', err.message);
            });

            this.vofaSocket.bind(CONFIG.VOFA_UDP_PORT, () => {
                console.log(`✓ vofa+ Server listening on port ${CONFIG.VOFA_UDP_PORT}`);
                console.log(`  FireWater 协议，13 个浮点数通道`);
                console.log(`  通道: roll,pitch,yaw,gyroX,gyroY,gyroZ,accX,accY,accZ,M1,M2,M3,M4`);
            });
        } catch (err) {
            console.error('✗ vofa+ server failed:', err.message);
        }
    }

    startUDPTelemetryServer() {
        try {
            this.udpTelemetrySocket = dgram.createSocket('udp4');

            this.udpTelemetrySocket.on('error', (err) => {
                console.error('✗ UDP Telemetry socket error:', err.message);
            });

            this.udpTelemetrySocket.on('message', (msg, rinfo) => {
                try {
                    // 校验魔数
                    if (msg.length < UDP_TELEMETRY_HEADER_SIZE) return;
                    if (!msg.subarray(0, 4).equals(UDP_TELEMETRY_MAGIC)) return;

                    const pktType = msg.readUInt8(4);
                    // 目前只处理遥测包 (type=0x01)
                    if (pktType !== 0x01) return;

                    const payload = msg.subarray(UDP_TELEMETRY_HEADER_SIZE);
                    const telemetry = parseTelemetry(payload);
                    if (!telemetry) return;

                    // 匹配对应的 TCP 客户端，更新其遥测缓存
                    const client = this.findClientByIP(rinfo.address);
                    if (client) {
                        client.latestTelemetry = telemetry;
                        if (telemetry.motorOutputs && telemetry.motorOutputs.length === 4) {
                            client.motorOutputs = telemetry.motorOutputs;
                        }
                    }

                    // 转发到所有 WebSocket 客户端
                    this.broadcastTelemetry(
                        client ? client.remoteAddr : rinfo.address,
                        telemetry
                    );
                } catch (err) {
                    // 静默丢弃假包
                }
            });

            this.udpTelemetrySocket.bind(CONFIG.UDP_TELEMETRY_PORT, () => {
                console.log(`✓ UDP Telemetry Server listening on port ${CONFIG.UDP_TELEMETRY_PORT}`);
                console.log(`  飞控向此端口发送遥测包（姿态角/电机/陀螺仪），释放 TCP 带宽`);
            });
        } catch (err) {
            console.error('✗ UDP Telemetry server failed:', err.message);
        }
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

        // ESC 首次行程校准（触发参数 escCalib.request=1）
        app.post('/api/esc/calibrate-first', (req, res) => {
            if (this.clients.size === 0) {
                return res.status(400).json({ error: 'No drone connected' });
            }

            try {
                for (const client of this.clients.values()) {
                    // 固件收到后会重启，并在 PWM 初始化后立即执行最大->最小校准序列
                    client.sendCRTP(this.buildParamSetByNamePacket('escCalib', 'request', PARAM_UINT8, 1, 1));
                }
                res.json({ success: true, action: 'esc_first_calibration_triggered' });
            } catch (err) {
                console.error('[ESC] Trigger first calibration error:', err.message);
                res.status(500).json({ error: err.message });
            }
        });

        // ========== vofa+ 配置 API ==========
        app.post('/api/vofa/enable', (req, res) => {
            const { enabled } = req.body || {};
            this.vofaEnabled = enabled === true;
            console.log(`[API] vofa+ ${this.vofaEnabled ? 'enabled' : 'disabled'}`);
            res.json({ success: true, enabled: this.vofaEnabled });
        });

        app.get('/api/vofa/status', (req, res) => {
            const clients = [];
            for (const [key, val] of this.vofaClients) {
                clients.push(val);
            }
            res.json({
                enabled: this.vofaEnabled,
                clients,
                clientCount: this.vofaClients.size
            });
        });

        app.post('/api/vofa/add-client', (req, res) => {
            const { ip, port } = req.body || {};
            if (!ip || typeof ip !== 'string') {
                return res.status(400).json({ error: 'Invalid IP' });
            }
            const clientPort = parseInt(port) || 1347;
            const key = `${ip}:${clientPort}`;
            this.vofaClients.set(key, { ip, port: clientPort });
            console.log(`[API] Added vofa+ client: ${key} (total: ${this.vofaClients.size})`);
            res.json({ success: true, clientCount: this.vofaClients.size });
        });

        app.post('/api/vofa/remove-client', (req, res) => {
            const { key } = req.body || {};
            if (!key || typeof key !== 'string') {
                return res.status(400).json({ error: 'Invalid key' });
            }
            this.vofaClients.delete(key);
            console.log(`[API] Removed vofa+ client: ${key} (remaining: ${this.vofaClients.size})`);
            res.json({ success: true, clientCount: this.vofaClients.size });
        });

        // 设置单个 PID 参数 (float)
        app.get('/api/pid/profile', (req, res) => {
            res.json({
                success: true,
                version: PID_PROFILE_VERSION,
                count: this.pidProfile.size,
                params: this.getPidProfileObject()
            });
        });

        app.post('/api/pid/profile', (req, res) => {
            const { params } = req.body || {};
            if (!params || typeof params !== 'object' || Array.isArray(params)) {
                return res.status(400).json({ error: 'Invalid params object' });
            }

            const entries = [];
            for (const key of Object.keys(params)) {
                const dot = key.indexOf('.');
                if (dot <= 0) continue;
                entries.push({
                    group: key.slice(0, dot),
                    name: key.slice(dot + 1),
                    value: params[key]
                });
            }

            const accepted = this.mergePidProfileEntries(entries);
            res.json({ success: true, saved: accepted, count: this.pidProfile.size });
        });

        app.post('/api/pid/set', (req, res) => {
            const { group, name, value } = req.body || {};
            console.log(`[API] /api/pid/set - ${group}.${name} = ${value}`);

            const normalized = this.normalizePidParam(group, name, value);
            if (!normalized.ok) {
                return res.status(400).json({ error: normalized.error });
            }

            this.mergePidProfileEntries([normalized]);

            let appliedClients = 0;
            const results = [];
            for (const client of this.clients.values()) {
                try {
                    const pkt = this.buildParamSetByNamePacket(
                        normalized.group,
                        normalized.name,
                        PARAM_FLOAT,
                        normalized.value,
                        4
                    );
                    client.sendCRTP(pkt);
                    appliedClients++;
                    results.push({ client: client.remoteAddr, success: true });
                } catch (err) {
                    results.push({ client: client.remoteAddr, error: err.message });
                }
            }

            res.json({
                success: true,
                group: normalized.group,
                name: normalized.name,
                value: normalized.value,
                saved: true,
                appliedClients,
                queued: this.clients.size === 0,
                results
            });
        });

        app.post('/api/pid/batch', (req, res) => {
            const { params } = req.body || {};
            console.log(`[API] /api/pid/batch - ${params ? params.length : 0} params`);

            if (!Array.isArray(params) || params.length === 0) {
                return res.status(400).json({ error: 'No params provided' });
            }

            const normalizedParams = [];
            const results = [];

            for (const p of params) {
                const normalized = this.normalizePidParam(p.group, p.name, p.value);
                if (!normalized.ok) {
                    results.push({ group: p.group, name: p.name, value: p.value, error: normalized.error });
                    continue;
                }
                normalizedParams.push(normalized);
            }

            this.mergePidProfileEntries(normalizedParams);

            for (const p of normalizedParams) {
                if (this.clients.size === 0) {
                    results.push({ group: p.group, name: p.name, value: p.value, success: true, queued: true });
                    continue;
                }

                let applyOk = true;
                let applyErr = '';
                for (const client of this.clients.values()) {
                    try {
                        const pkt = this.buildParamSetByNamePacket(p.group, p.name, PARAM_FLOAT, p.value, 4);
                        client.sendCRTP(pkt);
                    } catch (err) {
                        applyOk = false;
                        applyErr = err.message;
                    }
                }

                if (applyOk) {
                    results.push({ group: p.group, name: p.name, value: p.value, success: true });
                } else {
                    results.push({ group: p.group, name: p.name, value: p.value, error: applyErr || 'Apply failed' });
                }
            }

            res.json({ success: true, savedCount: normalizedParams.length, results });
        });
    }

    setupWebSocket() {
        this.wss.on('connection', (ws) => {
            console.log('WebSocket client connected');
            ws.isAlive = true;
            ws.lastSeenAt = Date.now();
            this.webClients.add(ws);

            ws.on('pong', () => {
                ws.isAlive = true;
                ws.lastSeenAt = Date.now();
            });

            ws.on('message', (raw) => {
                try {
                    const msg = JSON.parse(raw.toString());
                    if (msg && msg.type === 'ping') {
                        ws.isAlive = true;
                        ws.lastSeenAt = Date.now();
                        if (ws.readyState === WebSocket.OPEN) {
                            ws.send(JSON.stringify({ type: 'pong', timestamp: Date.now() }));
                        }
                    }
                } catch (_) {
                    // 忽略非JSON消息
                }
            });

            ws.on('close', (code, reasonBuffer) => {
                const reason = reasonBuffer ? reasonBuffer.toString() : '';
                console.log(
                    `WebSocket client disconnected | code=${code}` +
                    `${reason ? ` | reason=${reason}` : ''}`
                );
                this.webClients.delete(ws);
            });

            ws.on('error', (err) => {
                console.error('WebSocket error:', err.message);
            });
        });

        // 服务端保活：定时ping，超时终止僵尸连接
        if (this.wsPingTimer) {
            clearInterval(this.wsPingTimer);
        }
        this.wsPingTimer = setInterval(() => {
            for (const ws of this.webClients) {
                if (ws.readyState !== WebSocket.OPEN) {
                    this.webClients.delete(ws);
                    continue;
                }

                const sinceLastSeen = Date.now() - (ws.lastSeenAt || 0);
                if (sinceLastSeen > (CONFIG.WS_PING_TIMEOUT || 15000)) {
                    this.webClients.delete(ws);
                    ws.terminate();
                    continue;
                }

                ws.isAlive = false;
                ws.ping();
            }
        }, CONFIG.WS_PING_INTERVAL || 10000);
    }

    removeClient(client, reason = null) {
        this.clients.delete(client.remoteAddr);
        this.clientsByIP.delete(client.droneIP);

        if (reason) {
            console.log(
                `[${client.remoteAddr}] Client removed | reason=${reason.code}` +
                `${reason.detail ? ` | detail=${reason.detail}` : ''}`
            );
        } else {
            console.log(`[${client.remoteAddr}] Client removed | reason=unknown`);
        }

        // 通知前端有飞控断开及原因
        const evt = JSON.stringify({
            type: 'drone_disconnected',
            addr: client.remoteAddr,
            reason: reason || { code: 'unknown', detail: '' },
            timestamp: Date.now()
        });

        for (const ws of this.webClients) {
            if (ws.readyState === WebSocket.OPEN) {
                ws.send(evt);
            }
        }
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

        this.broadcastTelemetry(client.remoteAddr, telemetry);
    }

    /**
     * 将遥测数据广播到所有 WebSocket 客户端
     * TCP 路径和 UDP 路径共用此方法
     */
    broadcastTelemetry(addr, telemetry) {
        const data = JSON.stringify({
            type: 'telemetry',
            addr,
            data: telemetry,
            timestamp: Date.now()
        });

        for (const ws of this.webClients) {
            if (ws.readyState === WebSocket.OPEN) {
                ws.send(data);
            }
        }

        // 同时转发到 vofa+ (FireWater 协议)
        if (this.vofaEnabled && this.vofaSocket && this.vofaClients.size > 0) {
            this.sendToVofaClients(telemetry);
        }
    }

    /**
     * FireWater 协议转发遥测数据到 vofa+
     * 格式: channel1:value1,channel2:value2,channel3:value3\n
     */
    sendToVofaClients(telemetry) {
        if (!telemetry) return;

        // 构建 FireWater 格式字符串（逗号分隔浮点数）
        const values = [
            telemetry.roll || 0,
            telemetry.pitch || 0,
            telemetry.yaw || 0,
            telemetry.gyroX || 0,
            telemetry.gyroY || 0,
            telemetry.gyroZ || 0,
            telemetry.accX || 0,
            telemetry.accY || 0,
            telemetry.accZ || 0,
            (telemetry.motorOutputs && telemetry.motorOutputs[0]) || 0,
            (telemetry.motorOutputs && telemetry.motorOutputs[1]) || 0,
            (telemetry.motorOutputs && telemetry.motorOutputs[2]) || 0,
            (telemetry.motorOutputs && telemetry.motorOutputs[3]) || 0
        ];

        // FireWater 格式: 13 个浮点数，逗号分隔，最后加换行
        const payload = values.map(v => v.toFixed(2)).join(',') + '\n';

        const buf = Buffer.from(payload);
        for (const [key, client] of this.vofaClients) {
            this.vofaSocket.send(buf, 0, buf.length, client.port, client.ip, (err) => {
                if (err && err.code !== 'ENETUNREACH') {
                    console.error(`[vofa+] Send error to ${key}:`, err.message);
                }
            });
        }
    }

    onCRTP(client, payload) {
        console.log(`[${client.remoteAddr}] CRTP packet (${payload.length} bytes)`);
    }

    stop() {
        if (this.discoveryInterval) {
            clearInterval(this.discoveryInterval);
        }
        if (this.wsPingTimer) {
            clearInterval(this.wsPingTimer);
        }
        if (this.discoverySocket) {
            this.discoverySocket.close();
        }
        if (this.udpTelemetrySocket) {
            this.udpTelemetrySocket.close();
        }
        if (this.vofaSocket) {
            this.vofaSocket.close();
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
