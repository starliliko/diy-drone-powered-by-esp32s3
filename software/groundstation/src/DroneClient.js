/**
 * ESP-Drone 飞控客户端类
 * ==========================
 * 处理与单个飞控的TCP连接
 */

const CONFIG = require('./config');
const { PacketType, HEADER_SIZE } = require('./protocol');
const { parseHeader, createHeader, parseTelemetry, createControlCommand } = require('./parser');

const RX_BUFFER_MAX_BYTES = Math.max(HEADER_SIZE * 2, Number(CONFIG.RX_BUFFER_MAX_BYTES) || (256 * 1024));
const MAX_PACKET_SIZE = Math.max(HEADER_SIZE, Number(CONFIG.MAX_PACKET_SIZE) || (8 * 1024));

class DroneClient {
    constructor(socket, remoteAddr, server) {
        this.socket = socket;
        this.remoteAddr = remoteAddr;
        this.droneIP = socket.remoteAddress
            ? socket.remoteAddress.replace(/^::ffff:/, '')
            : remoteAddr.split(':')[0];
        this.server = server;
        this.rxBuffer = Buffer.alloc(0);
        this.txSeq = 0;
        this.lastHeartbeat = Date.now();
        this.lastPacketAt = Date.now();
        this.connectTime = Date.now();
        this.rxCount = 0;
        this.txCount = 0;
        this.latestTelemetry = null;
        this.disconnectReason = null;
        // 电机输出状态 (四个电机，初始值为0)
        this.motorOutputs = [0, 0, 0, 0];

        this.setupSocket();
        this.startHeartbeatCheck();

        console.log(`[${this.remoteAddr}] Drone connected`);
    }

    setupSocket() {
        // 提升弱网络下链路稳定性
        this.socket.setKeepAlive(true, 3000);
        this.socket.setNoDelay(true);

        this.socket.on('data', (data) => this.handleData(data));
        this.socket.on('error', (err) => this.handleError(err));
        this.socket.on('end', () => {
            this.setDisconnectReason('peer_half_close', 'Peer sent FIN');
        });
        this.socket.on('close', () => this.handleClose());
    }

    setDisconnectReason(code, detail = '') {
        if (!this.disconnectReason) {
            this.disconnectReason = {
                code,
                detail,
                at: Date.now()
            };
        }
    }

    handleData(data) {
        // 任意收到数据都视为链路活跃
        this.lastPacketAt = Date.now();
        const nextLength = this.rxBuffer.length + data.length;
        if (nextLength > RX_BUFFER_MAX_BYTES) {
            this.setDisconnectReason(
                'rx_buffer_overflow',
                `Buffered ${nextLength} bytes (limit ${RX_BUFFER_MAX_BYTES})`
            );
            console.warn(`[${this.remoteAddr}] RX buffer overflow: ${nextLength} bytes`);
            this.close('rx_buffer_overflow');
            return;
        }

        this.rxBuffer = Buffer.concat([this.rxBuffer, data], nextLength);
        this.processBuffer();
    }

    processBuffer() {
        while (this.rxBuffer.length >= HEADER_SIZE) {
            const header = parseHeader(this.rxBuffer);

            if (!header) {
                this.rxBuffer = this.rxBuffer.slice(1);
                continue;
            }

            if (header.length > MAX_PACKET_SIZE) {
                this.setDisconnectReason(
                    'packet_too_large',
                    `Packet payload ${header.length} exceeds limit ${MAX_PACKET_SIZE}`
                );
                console.warn(`[${this.remoteAddr}] Oversized packet: ${header.length} bytes`);
                this.close('packet_too_large');
                return;
            }

            const totalLen = HEADER_SIZE + header.length;
            if (this.rxBuffer.length < totalLen) {
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
        this.lastPacketAt = Date.now();

        switch (pktType) {
            case PacketType.HEARTBEAT:
                this.lastHeartbeat = Date.now();
                this.sendHeartbeat();
                break;

            case PacketType.TELEMETRY:
                const telemetry = parseTelemetry(payload);
                if (telemetry) {
                    this.latestTelemetry = telemetry;
                    // 从 telemetry 中提取电机输出 (V3.1 新增字段)
                    if (telemetry.motorOutputs && telemetry.motorOutputs.length === 4) {
                        this.motorOutputs = telemetry.motorOutputs;
                    }
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
        const timestamp = Date.now() >>> 0;
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

    sendCRTP(data) {
        console.log(`[CRTP TX] Sending ${data.length} bytes: ${data.toString('hex')}`);
        this.sendPacket(PacketType.CRTP, data);
    }

    // 处理电机控制命令，更新本地缓存
    handleMotorCommand(motorId, thrust) {
        if (motorId >= 0 && motorId < 4) {
            this.motorOutputs[motorId] = thrust;
        }
    }

    startHeartbeatCheck() {
        this.heartbeatTimer = setInterval(() => {
            // 只要有任意有效数据包，都不应判定掉线
            if (Date.now() - this.lastPacketAt > CONFIG.HEARTBEAT_TIMEOUT) {
                const idleMs = Date.now() - this.lastPacketAt;
                this.setDisconnectReason('link_timeout', `No packet for ${idleMs}ms (threshold ${CONFIG.HEARTBEAT_TIMEOUT}ms)`);
                console.log(`[${this.remoteAddr}] Link timeout (>${CONFIG.HEARTBEAT_TIMEOUT}ms, last packet ${idleMs}ms ago)`);
                this.close('link_timeout');
            }
        }, CONFIG.HEARTBEAT_CHECK_INTERVAL || 3000);
    }

    handleError(err) {
        this.setDisconnectReason('socket_error', err.message || 'unknown socket error');
        console.error(`[${this.remoteAddr}] Socket error:`, err.message);
    }

    handleClose() {
        if (!this.disconnectReason) {
            this.setDisconnectReason('peer_close', 'Connection closed by peer');
        }

        const reason = this.disconnectReason;
        console.log(
            `[${this.remoteAddr}] Drone disconnected | reason=${reason.code}` +
            `${reason.detail ? ` | detail=${reason.detail}` : ''}`
        );

        clearInterval(this.heartbeatTimer);
        this.server.removeClient(this, reason);
    }

    close(reasonCode = 'server_close', detail = '') {
        this.setDisconnectReason(reasonCode, detail);
        this.socket.destroy();
    }

    getStats() {
        return {
            addr: this.remoteAddr,
            connectTime: this.connectTime,
            rxCount: this.rxCount,
            txCount: this.txCount,
            lastHeartbeat: this.lastHeartbeat,
            lastPacketAt: this.lastPacketAt,
            disconnectReason: this.disconnectReason,
            telemetry: this.latestTelemetry,
            motorOutputs: this.motorOutputs
        };
    }
}

module.exports = DroneClient;
