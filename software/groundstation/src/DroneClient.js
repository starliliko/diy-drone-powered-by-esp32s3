/**
 * ESP-Drone 飞控客户端类
 * ==========================
 * 处理与单个飞控的TCP连接
 */

const CONFIG = require('./config');
const { PacketType, HEADER_SIZE } = require('./protocol');
const { parseHeader, createHeader, parseTelemetry, createControlCommand } = require('./parser');
const {
    parsePackets: parseMavlinkPackets,
    encodeManualControl,
    encodeCommandLong,
    encodeParamSet,
    decodeHeartbeat,
    decodeSysStatus,
    decodeAttitude,
    decodeLocalPositionNed,
    decodeServoOutputRaw,
    decodeHighresImu,
    decodeCommandAck,
    MAV_CMD_NAV_LAND,
    MAV_CMD_COMPONENT_ARM_DISARM
} = require('./mavlink');

const RX_BUFFER_MAX_BYTES = Math.max(HEADER_SIZE * 2, Number(CONFIG.RX_BUFFER_MAX_BYTES) || (256 * 1024));
const MAX_PACKET_SIZE = Math.max(HEADER_SIZE, Number(CONFIG.MAX_PACKET_SIZE) || (8 * 1024));
const PROTOCOL_MODE = String(CONFIG.PROTOCOL_MODE || 'legacy').toLowerCase();
const LEGACY_ENABLED = PROTOCOL_MODE === 'legacy' || PROTOCOL_MODE === 'dual';
const MAVLINK_ENABLED = PROTOCOL_MODE === 'mavlink' || PROTOCOL_MODE === 'dual';

const MAV_PARAM_TYPE = {
    UINT8: 1,
    INT8: 2,
    UINT16: 3,
    INT16: 4,
    UINT32: 5,
    INT32: 6,
    REAL32: 9
};

const PARAM_TYPE_TO_MAV = {
    0x08: MAV_PARAM_TYPE.UINT8,
    0x00: MAV_PARAM_TYPE.INT8,
    0x09: MAV_PARAM_TYPE.UINT16,
    0x01: MAV_PARAM_TYPE.INT16,
    0x0a: MAV_PARAM_TYPE.UINT32,
    0x02: MAV_PARAM_TYPE.INT32,
    0x06: MAV_PARAM_TYPE.REAL32
};

// MAVLink PARAM_SET param_id is fixed 16 bytes, so long keys need aliases.
const PARAM_ID_ALIASES = {
    'pid_attitude.roll_kp': 'pa_r_kp',
    'pid_attitude.roll_ki': 'pa_r_ki',
    'pid_attitude.roll_kd': 'pa_r_kd',
    'pid_attitude.pitch_kp': 'pa_p_kp',
    'pid_attitude.pitch_ki': 'pa_p_ki',
    'pid_attitude.pitch_kd': 'pa_p_kd',
    'pid_attitude.yaw_kp': 'pa_y_kp',
    'pid_attitude.yaw_ki': 'pa_y_ki',
    'pid_attitude.yaw_kd': 'pa_y_kd',
    'posCtlPid.thrustBase': 'pp_th_base',
    'posCtlPid.thrustMin': 'pp_th_min',
    'posCtlPid.rpLimit': 'pp_rp_lim',
    'posCtlPid.xyVelMax': 'pp_xy_vmax',
    'posCtlPid.zVelMax': 'pp_z_vmax'
};

function clamp(v, min, max) {
    return Math.max(min, Math.min(max, v));
}

function radToDeg(v) {
    return (v * 180) / Math.PI;
}

function resolveMavParamId(group, name) {
    const full = `${group}.${name}`;
    if (full.length <= 16) {                                                                                
        return full;
    }

    const alias = PARAM_ID_ALIASES[full];
    if (!alias) {
        throw new Error(`MAVLink param_id too long and no alias defined for ${full}`);
    }
    return alias;
}

function mapParamTypeToMav(paramType) {
    return PARAM_TYPE_TO_MAV[paramType] || MAV_PARAM_TYPE.REAL32;
}

class DroneClient {
    constructor(socket, remoteAddr, server) {
        this.socket = socket;
        this.remoteAddr = remoteAddr;
        this.droneIP = socket.remoteAddress
            ? socket.remoteAddress.replace(/^::ffff:/, '')
            : remoteAddr.split(':')[0];
        this.server = server;
        this.rxBuffer = Buffer.alloc(0);
        this.mavRxBuffer = Buffer.alloc(0);
        this.txSeq = 0;
        this.mavTxSeq = 0;
        this.lastHeartbeat = Date.now();
        this.lastPacketAt = Date.now();
        this.connectTime = Date.now();
        this.rxCount = 0;
        this.txCount = 0;
        this.latestTelemetry = null;
        this.disconnectReason = null;
        this.activeProtocol = 'unknown';
        this.targetSystem = 1;
        this.targetComponent = 1;
        this.mavState = {
            roll: 0,
            pitch: 0,
            yaw: 0,
            gyroX: 0,
            gyroY: 0,
            gyroZ: 0,
            accX: 0,
            accY: 0,
            accZ: 0,
            estVelX: 0,
            estVelY: 0,
            estVelZ: 0,
            estAltitude: 0,
            battVoltage: 0,
            battPercent: 0,
            isArmed: false,
            flightTime: 0
        };
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

        if (LEGACY_ENABLED) {
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

        if (MAVLINK_ENABLED) {
            const nextLength = this.mavRxBuffer.length + data.length;
            if (nextLength > RX_BUFFER_MAX_BYTES) {
                this.setDisconnectReason(
                    'mav_rx_buffer_overflow',
                    `Buffered ${nextLength} bytes (limit ${RX_BUFFER_MAX_BYTES})`
                );
                console.warn(`[${this.remoteAddr}] MAVLink RX buffer overflow: ${nextLength} bytes`);
                this.close('mav_rx_buffer_overflow');
                return;
            }

            this.mavRxBuffer = Buffer.concat([this.mavRxBuffer, data], nextLength);
            this.processMavlinkBuffer();
        }
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

    processMavlinkBuffer() {
        const parsed = parseMavlinkPackets(this.mavRxBuffer);
        this.mavRxBuffer = Buffer.from(parsed.remaining);

        for (const msg of parsed.messages) {
            this.rxCount++;
            this.handleMavlinkMessage(msg);
        }
    }

    handlePacket(header, payload) {
        const pktType = header.pktType;
        this.lastPacketAt = Date.now();
        this.activeProtocol = 'legacy';

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

    emitMavTelemetry() {
        const t = {
            roll: this.mavState.roll,
            pitch: this.mavState.pitch,
            yaw: this.mavState.yaw,
            gyroX: this.mavState.gyroX,
            gyroY: this.mavState.gyroY,
            gyroZ: this.mavState.gyroZ,
            accX: this.mavState.accX,
            accY: this.mavState.accY,
            accZ: this.mavState.accZ,
            battVoltage: this.mavState.battVoltage,
            battPercent: this.mavState.battPercent,
            isArmed: this.mavState.isArmed,
            armingState: this.mavState.isArmed ? 2 : 1,
            armingStateName: this.mavState.isArmed ? 'ARMED' : 'STANDBY',
            armingStateNameCN: this.mavState.isArmed ? '已解锁' : '待机',
            flightMode: 1,
            flightModeName: 'STABILIZE',
            flightModeNameCN: '自稳模式',
            actualFlightMode: 0,
            actualFlightModeName: 'STABILIZE',
            actualFlightModeNameCN: '自稳模式',
            controlSource: 2,
            controlSourceName: 'REMOTE',
            controlSourceNameCN: '地面站',
            remoteCtrlMode: 1,
            remoteCtrlModeName: 'ENABLED',
            remoteCtrlModeNameCN: '启用',
            flightPhase: this.mavState.isArmed ? 2 : 0,
            flightPhaseName: this.mavState.isArmed ? 'IN_AIR' : 'ON_GROUND',
            flightPhaseNameCN: this.mavState.isArmed ? '空中飞行' : '地面待机',
            failsafeState: 0,
            failsafeStateName: 'NONE',
            statusFlags: this.mavState.isArmed ? 0x01 : 0x00,
            isFlying: this.mavState.isArmed,
            isEmergency: false,
            isRcConnected: false,
            isGcsConnected: true,
            isLowBattery: this.mavState.battPercent > 0 && this.mavState.battPercent < 20,
            isTumbled: false,
            isArmThrottleBlocked: false,
            timestamp: Date.now(),
            flightTime: this.mavState.flightTime,
            estAltitude: this.mavState.estAltitude,
            baroAltitude: 0,
            tofDistance: 0,
            estVelX: this.mavState.estVelX,
            estVelY: this.mavState.estVelY,
            estVelZ: this.mavState.estVelZ,
            bodyVelX: 0,
            bodyVelY: 0,
            magX: 0,
            magY: 0,
            magZ: 0,
            magHeading: 0,
            motorOutputs: this.motorOutputs
        };

        this.latestTelemetry = t;
        this.server.onTelemetry(this, t);
    }

    handleMavlinkMessage(msg) {
        this.activeProtocol = 'mavlink';
        this.lastPacketAt = Date.now();
        this.lastHeartbeat = Date.now();
        this.targetSystem = msg.sysId || this.targetSystem;
        this.targetComponent = msg.compId || this.targetComponent;

        switch (msg.msgId) {
            case 0: {
                const hb = decodeHeartbeat(msg.payload);
                if (hb) {
                    this.mavState.isArmed = (hb.baseMode & 0x80) !== 0;
                    this.emitMavTelemetry();
                }
                break;
            }
            case 1: {
                const ss = decodeSysStatus(msg.payload);
                if (ss) {
                    this.mavState.battVoltage = ss.voltageBatteryMv > 0 ? ss.voltageBatteryMv / 1000.0 : 0;
                    this.mavState.battPercent = ss.batteryRemaining >= 0 ? ss.batteryRemaining : 0;
                    this.emitMavTelemetry();
                }
                break;
            }
            case 30: {
                const att = decodeAttitude(msg.payload);
                if (att) {
                    this.mavState.roll = radToDeg(att.roll);
                    this.mavState.pitch = radToDeg(att.pitch);
                    this.mavState.yaw = radToDeg(att.yaw);
                    this.mavState.gyroX = radToDeg(att.rollspeed);
                    this.mavState.gyroY = radToDeg(att.pitchspeed);
                    this.mavState.gyroZ = radToDeg(att.yawspeed);
                    this.mavState.flightTime = att.timeBootMs >>> 0;
                    this.emitMavTelemetry();
                }
                break;
            }
            case 32: {
                const pos = decodeLocalPositionNed(msg.payload);
                if (pos) {
                    this.mavState.estAltitude = -pos.z;
                    this.mavState.estVelX = pos.vx;
                    this.mavState.estVelY = pos.vy;
                    this.mavState.estVelZ = pos.vz;
                    this.mavState.flightTime = pos.timeBootMs >>> 0;
                    this.emitMavTelemetry();
                }
                break;
            }
            case 36: {
                const servo = decodeServoOutputRaw(msg.payload);
                if (servo) {
                    this.motorOutputs = [
                        servo.servo1Raw,
                        servo.servo2Raw,
                        servo.servo3Raw,
                        servo.servo4Raw
                    ];
                    this.emitMavTelemetry();
                }
                break;
            }
            case 105: {
                const imu = decodeHighresImu(msg.payload);
                if (imu) {
                    this.mavState.accX = imu.xacc * 1000.0 / 9.80665;
                    this.mavState.accY = imu.yacc * 1000.0 / 9.80665;
                    this.mavState.accZ = imu.zacc * 1000.0 / 9.80665;
                    this.emitMavTelemetry();
                }
                break;
            }
            case 77: {
                const ack = decodeCommandAck(msg.payload);
                if (ack) {
                    console.log(`[${this.remoteAddr}] MAVLink ACK cmd=${ack.command} result=${ack.result}`);
                }
                break;
            }
            default:
                break;
        }
    }

    sendHeartbeat() {
        if (!LEGACY_ENABLED) {
            return;
        }
        const timestamp = Date.now() >>> 0;
        const payload = Buffer.allocUnsafe(4);
        payload.writeUInt32LE(timestamp, 0);
        this.sendPacket(PacketType.HEARTBEAT, payload);
    }

    sendPacket(pktType, payload = Buffer.alloc(0)) {
        if (!LEGACY_ENABLED) {
            return;
        }
        const header = createHeader(pktType, this.txSeq, payload.length);
        this.txSeq = (this.txSeq + 1) & 0xFFFF;

        try {
            this.socket.write(Buffer.concat([header, payload]));
            this.txCount++;
        } catch (err) {
            console.error(`[${this.remoteAddr}] TX error:`, err.message);
        }
    }

    sendMavlinkPacket(packet) {
        try {
            this.socket.write(packet);
            this.txCount++;
        } catch (err) {
            console.error(`[${this.remoteAddr}] MAVLink TX error:`, err.message);
        }
    }

    sendControlMavlink(cmdType, roll, pitch, yaw, thrust) {
        switch (cmdType) {
            case 0x00: {
                const x = clamp(Math.round((pitch || 0) * 100), -1000, 1000);
                const y = clamp(Math.round((roll || 0) * 100), -1000, 1000);
                const z = clamp(Math.round(((thrust || 0) / 65535) * 1000), 0, 1000);
                const r = clamp(Math.round((yaw || 0) * 10), -1000, 1000);
                const pkt = encodeManualControl(
                    this.mavTxSeq++,
                    255,
                    190,
                    this.targetSystem,
                    x,
                    y,
                    z,
                    r,
                    0
                );
                this.sendMavlinkPacket(pkt);
                break;
            }
            case 0x06: {
                const pkt = encodeCommandLong(
                    this.mavTxSeq++,
                    255,
                    190,
                    this.targetSystem,
                    this.targetComponent,
                    MAV_CMD_COMPONENT_ARM_DISARM,
                    1,
                    0
                );
                this.sendMavlinkPacket(pkt);
                break;
            }
            case 0x07:
            case 0x05: {
                const pkt = encodeCommandLong(
                    this.mavTxSeq++,
                    255,
                    190,
                    this.targetSystem,
                    this.targetComponent,
                    MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    cmdType === 0x05 ? 21196 : 0
                );
                this.sendMavlinkPacket(pkt);
                break;
            }
            case 0x04: {
                const pkt = encodeCommandLong(
                    this.mavTxSeq++,
                    255,
                    190,
                    this.targetSystem,
                    this.targetComponent,
                    MAV_CMD_NAV_LAND
                );
                this.sendMavlinkPacket(pkt);
                break;
            }
            default: {
                // 未映射命令先退化为手动控制包，避免静默丢弃。
                const pkt = encodeManualControl(
                    this.mavTxSeq++,
                    255,
                    190,
                    this.targetSystem,
                    0,
                    0,
                    0,
                    0,
                    0
                );
                this.sendMavlinkPacket(pkt);
                break;
            }
        }
    }

    sendControl(cmdType, roll, pitch, yaw, thrust, mode) {
        if (this.activeProtocol === 'mavlink' || (MAVLINK_ENABLED && !LEGACY_ENABLED)) {
            this.sendControlMavlink(cmdType, roll, pitch, yaw, thrust, mode);
            return;
        }

        const payload = createControlCommand(cmdType, roll, pitch, yaw, thrust, mode);
        this.sendPacket(PacketType.CONTROL, payload);
    }

    sendCRTP(data) {
        console.log(`[CRTP TX] Sending ${data.length} bytes: ${data.toString('hex')}`);
        this.sendPacket(PacketType.CRTP, data);
    }

    sendParamSet(group, name, paramType, value) {
        const paramId = resolveMavParamId(group, name);
        const mavParamType = mapParamTypeToMav(paramType);

        if (this.activeProtocol === 'mavlink' || (MAVLINK_ENABLED && !LEGACY_ENABLED)) {
            const pkt = encodeParamSet(
                this.mavTxSeq++,
                255,
                190,
                this.targetSystem,
                this.targetComponent,
                paramId,
                Number(value),
                mavParamType
            );
            this.sendMavlinkPacket(pkt);
            return;
        }

        throw new Error('CRTP parameter channel is disabled; switch protocol to mavlink/dual');
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
