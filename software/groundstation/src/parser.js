/**
 * ESP-Drone 数据包解析器
 * ==========================
 * 处理协议数据包的解析和创�?
 */

const {
    MAGIC,
    PROTOCOL_VERSION,
    HEADER_SIZE,
    RemoteControlMode,
    RemoteControlModeNameCN,
    FlightMode,
    FlightModeNameCN,
    ActualFlightMode,
    ActualFlightModeNameCN,
    ControlSource,
    ControlSourceNameCN,
    ArmingState,
    ArmingStateNameCN,
    FlightPhase,
    FlightPhaseNameCN,
    FailsafeState,
    STATUS_FLAGS
} = require('./protocol');
const CONFIG = require('./config');

const DEBUG_TELEMETRY_PARSER = CONFIG.DEBUG_TELEMETRY_PARSER === true;
const DEBUG_TELEMETRY_MOTORS = CONFIG.DEBUG_TELEMETRY_MOTORS === true;

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
 * 解析遥测数据 - PX4/QGC 风格 (Protocol V2/V3)
 */
function parseTelemetry(buffer) {
    // 调试：打印收到的数据包大小（�?00个包打印一次）
    if (!parseTelemetry.debugCount) parseTelemetry.debugCount = 0;
    if (DEBUG_TELEMETRY_PARSER && ++parseTelemetry.debugCount >= 100) {
        console.log(`[DEBUG] Telemetry buffer size: ${buffer.length} bytes`);
        parseTelemetry.debugCount = 0;
    }

    if (buffer.length < 42) {
        console.log(`[WARN] Telemetry buffer too small: ${buffer.length} bytes`);
        return null;
    }

    let offset = 0;
    const telemetry = {
        // 姿�?(�?
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
        // 电池状�?
        battVoltage: buffer.readUInt16LE(offset + 36) / 1000.0,
        battPercent: buffer.readUInt8(offset + 38),
    };

    // PX4风格状态字�?(新协�?
    if (buffer.length >= 52) {
        telemetry.armingState = buffer.readUInt8(offset + 39);
        telemetry.flightMode = buffer.readUInt8(offset + 40);
        telemetry.flightPhase = buffer.readUInt8(offset + 41);
        telemetry.failsafeState = buffer.readUInt8(offset + 42);
        telemetry.statusFlags = buffer.readUInt8(offset + 43);
        telemetry.timestamp = buffer.readUInt32LE(offset + 44);
        telemetry.flightTime = buffer.readUInt32LE(offset + 48);

        // V3 新增字段 (55字节)
        if (buffer.length >= 55) {
            telemetry.actualFlightMode = buffer.readUInt8(offset + 52);
            telemetry.controlSource = buffer.readUInt8(offset + 53);
            telemetry.remoteCtrlMode = buffer.readUInt8(offset + 54);
        } else if (buffer.length >= 54) {
            telemetry.actualFlightMode = buffer.readUInt8(offset + 52);
            telemetry.controlSource = buffer.readUInt8(offset + 53);
            telemetry.remoteCtrlMode = 1;
        } else {
            telemetry.actualFlightMode = 0;
            telemetry.controlSource = 0;
            telemetry.remoteCtrlMode = 1;
        }

        // V3.1 新增电机输出 (63字节)
        if (buffer.length >= 63) {
            telemetry.motorOutputs = [
                buffer.readUInt16LE(offset + 55),
                buffer.readUInt16LE(offset + 57),
                buffer.readUInt16LE(offset + 59),
                buffer.readUInt16LE(offset + 61)
            ];
            // Debug: 打印电机输出
            if (DEBUG_TELEMETRY_MOTORS && telemetry.motorOutputs.some(m => m > 0)) {
                console.log(`[MOTOR] M1=${telemetry.motorOutputs[0]} M2=${telemetry.motorOutputs[1]} M3=${telemetry.motorOutputs[2]} M4=${telemetry.motorOutputs[3]}`);
            }
        } else {
            if (DEBUG_TELEMETRY_PARSER) {
                console.log(`[WARN] Telemetry buffer too small for motor data: ${buffer.length} bytes, need 63`);
            }
        }

        // V3.2 新增高度和速度数据 (81字节)
        if (buffer.length >= 81) {
            // 估计器高�?(mm -> m)
            telemetry.estAltitude = buffer.readInt32LE(offset + 63) / 1000.0;
            // 气压计高�?(mm -> m)
            telemetry.baroAltitude = buffer.readInt32LE(offset + 67) / 1000.0;
            // ToF距离 (mm -> m)
            telemetry.tofDistance = buffer.readInt32LE(offset + 71) / 1000.0;
            // 估计器X速度 - 世界坐标�?(mm/s -> m/s)
            telemetry.estVelX = buffer.readInt16LE(offset + 75) / 1000.0;
            // 估计器Y速度 - 世界坐标�?(mm/s -> m/s)
            telemetry.estVelY = buffer.readInt16LE(offset + 77) / 1000.0;
            // 估计器Z速度 - 世界坐标�?(mm/s -> m/s)
            telemetry.estVelZ = buffer.readInt16LE(offset + 79) / 1000.0;
        } else {
            if (DEBUG_TELEMETRY_PARSER) {
                console.log(`[WARN] Telemetry buffer too small for altitude/velocity data: ${buffer.length} bytes (need 81)`);
            }
            telemetry.estAltitude = 0;
            telemetry.baroAltitude = 0;
            telemetry.tofDistance = 0;
            telemetry.estVelX = 0;
            telemetry.estVelY = 0;
            telemetry.estVelZ = 0;
        }

        // V3.3 新增机体坐标系速度 (85字节)
        if (buffer.length >= 85) {
            // 机体X速度 (前进�? (mm/s -> m/s)
            telemetry.bodyVelX = buffer.readInt16LE(offset + 81) / 1000.0;
            // 机体Y速度 (向左�? (mm/s -> m/s)
            telemetry.bodyVelY = buffer.readInt16LE(offset + 83) / 1000.0;
            // 调试: �?00包打印一次body velocity
            if (DEBUG_TELEMETRY_PARSER && parseTelemetry.debugCount === 0) {
                console.log(`[DEBUG] Body vel raw bytes at 81-84: ${buffer.readInt16LE(offset + 81)}, ${buffer.readInt16LE(offset + 83)} mm/s`);
                console.log(`[DEBUG] Body vel: X=${telemetry.bodyVelX.toFixed(3)}, Y=${telemetry.bodyVelY.toFixed(3)} m/s`);
                console.log(`[DEBUG] World vel: X=${telemetry.estVelX.toFixed(3)}, Y=${telemetry.estVelY.toFixed(3)} m/s`);
            }
        } else {
            telemetry.bodyVelX = 0;
            telemetry.bodyVelY = 0;
            if (DEBUG_TELEMETRY_PARSER) {
                console.log(`[WARN] Buffer too small for body velocity: ${buffer.length} bytes (need 85)`);
            }
        }

        // 从状态标志位解析
        telemetry.isArmed = !!(telemetry.statusFlags & STATUS_FLAGS.ARMED);
        telemetry.isFlying = !!(telemetry.statusFlags & STATUS_FLAGS.FLYING);
        telemetry.isEmergency = !!(telemetry.statusFlags & STATUS_FLAGS.EMERGENCY);
        telemetry.isRcConnected = !!(telemetry.statusFlags & STATUS_FLAGS.RC_CONNECTED);
        telemetry.isGcsConnected = !!(telemetry.statusFlags & STATUS_FLAGS.GCS_CONNECTED);
        telemetry.isLowBattery = !!(telemetry.statusFlags & STATUS_FLAGS.BATTERY_LOW);
        telemetry.isTumbled = !!(telemetry.statusFlags & STATUS_FLAGS.TUMBLED);
        telemetry.isArmThrottleBlocked = !!(telemetry.statusFlags & STATUS_FLAGS.ARM_THROTTLE_BLOCK);

        // 添加可读的状态名�?
        telemetry.armingStateName = ArmingState[telemetry.armingState] || 'UNKNOWN';
        telemetry.armingStateNameCN = ArmingStateNameCN[telemetry.armingState] || '未知';
        telemetry.flightPhaseName = FlightPhase[telemetry.flightPhase] || 'UNKNOWN';
        telemetry.flightPhaseNameCN = FlightPhaseNameCN[telemetry.flightPhase] || '未知';
        telemetry.failsafeStateName = FailsafeState[telemetry.failsafeState] || 'NONE';

        // V3 新增：实际飞行姿态模式和控制来源的可读名�?
        telemetry.actualFlightModeName = ActualFlightMode[telemetry.actualFlightMode] || 'UNKNOWN';
        telemetry.actualFlightModeNameCN = ActualFlightModeNameCN[telemetry.actualFlightMode] || '未知';
        telemetry.controlSourceName = ControlSource[telemetry.controlSource] || 'UNKNOWN';
        telemetry.controlSourceNameCN = ControlSourceNameCN[telemetry.controlSource] || '未知';
        telemetry.remoteCtrlModeName = RemoteControlMode[telemetry.remoteCtrlMode] !== undefined
            ? Object.keys(RemoteControlMode).find(k => RemoteControlMode[k] === telemetry.remoteCtrlMode)
            : 'UNKNOWN';
        telemetry.remoteCtrlModeNameCN = RemoteControlModeNameCN[telemetry.remoteCtrlMode] || '未知';
    } else {
        // 旧格式兼�?Protocol V1
        telemetry.flightMode = buffer.readUInt8(offset + 39);
        telemetry.isArmed = !!buffer.readUInt8(offset + 40);
        telemetry.isLowBattery = !!buffer.readUInt8(offset + 41);
        telemetry.timestamp = buffer.length >= 48 ? buffer.readUInt32LE(offset + 44) : Date.now();
        telemetry.armingState = telemetry.isArmed ? 2 : 1;
        telemetry.flightPhase = 0;
        telemetry.failsafeState = 0;
        telemetry.statusFlags = telemetry.isArmed ? 0x01 : 0x00;
        telemetry.flightTime = 0;
        telemetry.armingStateName = telemetry.isArmed ? 'ARMED' : 'STANDBY';
        telemetry.armingStateNameCN = telemetry.isArmed ? '�ѽ���' : '����';
        telemetry.flightPhaseName = 'ON_GROUND';
        telemetry.flightPhaseNameCN = '地面待机';
        telemetry.failsafeStateName = 'NONE';
        telemetry.actualFlightMode = 0;
        telemetry.controlSource = 0;
        telemetry.actualFlightModeName = 'STABILIZE';
        telemetry.actualFlightModeNameCN = '自稳模式';
        telemetry.controlSourceName = 'NONE';
        telemetry.controlSourceNameCN = '未知';
        telemetry.isTumbled = false;
        telemetry.isArmThrottleBlocked = false;
    }

    // 添加可读的目标模式名�?
    telemetry.flightModeName = FlightMode[telemetry.flightMode] || 'UNKNOWN';
    telemetry.flightModeNameCN = FlightModeNameCN[telemetry.flightMode] || '未知模式';

    return telemetry;
}

/**
 * 创建控制指令
 */
function createControlCommand(cmdType, roll = 0, pitch = 0, yaw = 0, thrust = 0, mode = 0) {
    // RemoteControlCmd 结构体大�? 1 + 2 + 2 + 2 + 2 + 1 + 3 = 13 字节
    const buffer = Buffer.allocUnsafe(13);
    buffer.writeUInt8(cmdType, 0);
    buffer.writeInt16LE(Math.round(roll * 100), 1);
    buffer.writeInt16LE(Math.round(pitch * 100), 3);
    buffer.writeInt16LE(Math.round(yaw * 10), 5);
    buffer.writeUInt16LE(thrust, 7);
    buffer.writeUInt8(mode, 9);
    buffer.fill(0, 10, 13);  // 3 字节 reserved
    return buffer;
}

module.exports = {
    parseHeader,
    createHeader,
    parseTelemetry,
    createControlCommand
};
