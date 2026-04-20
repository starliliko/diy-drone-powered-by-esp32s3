/**
 * Minimal MAVLink v2/v1 codec used by groundstation.
 *
 * This module intentionally implements only the subset required by
 * current flight-control telemetry and command flow.
 */

const MAVLINK_STX_V1 = 0xfe;
const MAVLINK_STX_V2 = 0xfd;

const MAV_MODE_FLAG_SAFETY_ARMED = 0x80;
const MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 0x01;

const MAV_CMD_NAV_LAND = 21;
const MAV_CMD_COMPONENT_ARM_DISARM = 400;

const MAV_RESULT = {
    ACCEPTED: 0,
    TEMPORARILY_REJECTED: 1,
    DENIED: 2,
    UNSUPPORTED: 3,
    FAILED: 4,
    IN_PROGRESS: 5
};

const EXTRA_CRC = {
    0: 50,    // HEARTBEAT
    1: 124,   // SYS_STATUS
    30: 39,   // ATTITUDE
    32: 185,  // LOCAL_POSITION_NED
    69: 243,  // MANUAL_CONTROL
    76: 152,  // COMMAND_LONG
    77: 143,  // COMMAND_ACK
    105: 93   // HIGHRES_IMU
};

function crcX25Accumulate(byte, crc) {
    let tmp = byte ^ (crc & 0xff);
    tmp = (tmp ^ (tmp << 4)) & 0xff;
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff;
}

function crcX25Buffer(buffer, start, end, extra) {
    let crc = 0xffff;
    for (let i = start; i < end; i++) {
        crc = crcX25Accumulate(buffer[i], crc);
    }
    crc = crcX25Accumulate(extra & 0xff, crc);
    return crc;
}

function tryParseOnePacket(buffer, offset) {
    if (offset >= buffer.length) {
        return null;
    }

    const stx = buffer[offset];
    const isV2 = stx === MAVLINK_STX_V2;
    const isV1 = stx === MAVLINK_STX_V1;

    if (!isV1 && !isV2) {
        return { consumed: 1 };
    }

    const headerLen = isV2 ? 10 : 6;
    if (offset + headerLen > buffer.length) {
        return null;
    }

    const payloadLen = buffer[offset + 1];
    const incompatFlags = isV2 ? buffer[offset + 2] : 0;
    const hasSignature = isV2 && ((incompatFlags & 0x01) !== 0);
    const crcLen = 2;
    const signatureLen = hasSignature ? 13 : 0;
    const totalLen = headerLen + payloadLen + crcLen + signatureLen;

    if (offset + totalLen > buffer.length) {
        return null;
    }

    const seq = isV2 ? buffer[offset + 4] : buffer[offset + 2];
    const sysId = isV2 ? buffer[offset + 5] : buffer[offset + 3];
    const compId = isV2 ? buffer[offset + 6] : buffer[offset + 4];
    const msgId = isV2
        ? (buffer[offset + 7] | (buffer[offset + 8] << 8) | (buffer[offset + 9] << 16))
        : buffer[offset + 5];

    const extra = EXTRA_CRC[msgId];
    if (extra === undefined) {
        return { consumed: totalLen };
    }

    const payloadStart = offset + headerLen;
    const payloadEnd = payloadStart + payloadLen;

    const crcStart = offset + 1;
    const crcEnd = payloadEnd;
    const expectedCrc = crcX25Buffer(buffer, crcStart, crcEnd, extra);
    const packetCrc = buffer[payloadEnd] | (buffer[payloadEnd + 1] << 8);

    if (expectedCrc !== packetCrc) {
        return { consumed: 1 };
    }

    return {
        consumed: totalLen,
        message: {
            version: isV2 ? 2 : 1,
            seq,
            sysId,
            compId,
            msgId,
            payload: buffer.subarray(payloadStart, payloadEnd)
        }
    };
}

function parsePackets(buffer) {
    const messages = [];
    let offset = 0;

    while (offset < buffer.length) {
        const parsed = tryParseOnePacket(buffer, offset);
        if (!parsed) {
            break;
        }

        offset += parsed.consumed;
        if (parsed.message) {
            messages.push(parsed.message);
        }
    }

    return {
        messages,
        remaining: buffer.subarray(offset)
    };
}

function encodeMavlinkV2Packet(msgId, payload, seq, sysId, compId) {
    const extra = EXTRA_CRC[msgId];
    if (extra === undefined) {
        throw new Error(`Missing MAVLink extra CRC for msgId=${msgId}`);
    }

    const payloadLen = payload.length;
    const packet = Buffer.allocUnsafe(10 + payloadLen + 2);

    packet[0] = MAVLINK_STX_V2;
    packet[1] = payloadLen;
    packet[2] = 0x00; // incompat flags
    packet[3] = 0x00; // compat flags
    packet[4] = seq & 0xff;
    packet[5] = sysId & 0xff;
    packet[6] = compId & 0xff;
    packet[7] = msgId & 0xff;
    packet[8] = (msgId >> 8) & 0xff;
    packet[9] = (msgId >> 16) & 0xff;

    payload.copy(packet, 10);

    const crc = crcX25Buffer(packet, 1, 10 + payloadLen, extra);
    packet[10 + payloadLen] = crc & 0xff;
    packet[10 + payloadLen + 1] = (crc >> 8) & 0xff;

    return packet;
}

function encodeManualControl(seq, sysId, compId, target, x, y, z, r, buttons = 0) {
    const payload = Buffer.allocUnsafe(11);
    payload.writeInt16LE(x, 0);
    payload.writeInt16LE(y, 2);
    payload.writeInt16LE(z, 4);
    payload.writeInt16LE(r, 6);
    payload.writeUInt16LE(buttons, 8);
    payload.writeUInt8(target & 0xff, 10);
    return encodeMavlinkV2Packet(69, payload, seq, sysId, compId);
}

function encodeCommandLong(seq, sysId, compId, targetSystem, targetComponent, command, p1 = 0, p2 = 0, p3 = 0, p4 = 0, p5 = 0, p6 = 0, p7 = 0, confirmation = 0) {
    const payload = Buffer.allocUnsafe(33);
    payload.writeFloatLE(p1, 0);
    payload.writeFloatLE(p2, 4);
    payload.writeFloatLE(p3, 8);
    payload.writeFloatLE(p4, 12);
    payload.writeFloatLE(p5, 16);
    payload.writeFloatLE(p6, 20);
    payload.writeFloatLE(p7, 24);
    payload.writeUInt16LE(command & 0xffff, 28);
    payload.writeUInt8(targetSystem & 0xff, 30);
    payload.writeUInt8(targetComponent & 0xff, 31);
    payload.writeUInt8(confirmation & 0xff, 32);
    return encodeMavlinkV2Packet(76, payload, seq, sysId, compId);
}

function decodeHeartbeat(payload) {
    if (payload.length < 9) return null;
    return {
        customMode: payload.readUInt32LE(0),
        type: payload.readUInt8(4),
        autopilot: payload.readUInt8(5),
        baseMode: payload.readUInt8(6),
        systemStatus: payload.readUInt8(7),
        mavlinkVersion: payload.readUInt8(8)
    };
}

function decodeSysStatus(payload) {
    if (payload.length < 31) return null;
    return {
        voltageBatteryMv: payload.readUInt16LE(14),
        batteryRemaining: payload.readInt8(30)
    };
}

function decodeAttitude(payload) {
    if (payload.length < 28) return null;
    return {
        timeBootMs: payload.readUInt32LE(0),
        roll: payload.readFloatLE(4),
        pitch: payload.readFloatLE(8),
        yaw: payload.readFloatLE(12),
        rollspeed: payload.readFloatLE(16),
        pitchspeed: payload.readFloatLE(20),
        yawspeed: payload.readFloatLE(24)
    };
}

function decodeLocalPositionNed(payload) {
    if (payload.length < 28) return null;
    return {
        timeBootMs: payload.readUInt32LE(0),
        x: payload.readFloatLE(4),
        y: payload.readFloatLE(8),
        z: payload.readFloatLE(12),
        vx: payload.readFloatLE(16),
        vy: payload.readFloatLE(20),
        vz: payload.readFloatLE(24)
    };
}

function decodeHighresImu(payload) {
    if (payload.length < 62) return null;
    return {
        xacc: payload.readFloatLE(8),
        yacc: payload.readFloatLE(12),
        zacc: payload.readFloatLE(16),
        xgyro: payload.readFloatLE(20),
        ygyro: payload.readFloatLE(24),
        zgyro: payload.readFloatLE(28)
    };
}

function decodeCommandAck(payload) {
    if (payload.length < 3) return null;
    return {
        command: payload.readUInt16LE(0),
        result: payload.readUInt8(2),
        accepted: payload.readUInt8(2) === MAV_RESULT.ACCEPTED
    };
}

module.exports = {
    MAVLINK_STX_V1,
    MAVLINK_STX_V2,
    MAV_MODE_FLAG_SAFETY_ARMED,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    MAV_CMD_NAV_LAND,
    MAV_CMD_COMPONENT_ARM_DISARM,
    MAV_RESULT,
    parsePackets,
    encodeManualControl,
    encodeCommandLong,
    decodeHeartbeat,
    decodeSysStatus,
    decodeAttitude,
    decodeLocalPositionNed,
    decodeHighresImu,
    decodeCommandAck
};
