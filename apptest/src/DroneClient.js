/**
 * ESP-Drone 飞控客户端类
 * ==========================
 * 处理与单个飞控的TCP连接
 */

const CONFIG = require('./config');
const { PacketType, HEADER_SIZE } = require('./protocol');
const { parseHeader, createHeader, parseTelemetry, createControlCommand } = require('./parser');

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
        // 电机输出状态 (四个电机，初始值为0)
        this.motorOutputs = [0, 0, 0, 0];

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
                this.rxBuffer = this.rxBuffer.slice(1);
                continue;
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
            telemetry: this.latestTelemetry,
            motorOutputs: this.motorOutputs
        };
    }
}

module.exports = DroneClient;
