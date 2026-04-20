/**
 * ESP-Drone config
 */

module.exports = {
    // 通信协议模式:
    // - legacy: 仅使用当前自定义协议
    // - mavlink: 仅使用 MAVLink
    // - dual: 同时兼容两种协议（迁移推荐）
    PROTOCOL_MODE: 'dual',
    TCP_PORT: 8080,
    HTTP_PORT: 3000,
    DISCOVERY_PORT: 8081,
    UDP_TELEMETRY_PORT: 8082,
    VOFA_UDP_PORT: 8083,
    WEB_TELEMETRY_INTERVAL: 10,
    DEBUG_TELEMETRY_PARSER: false,
    DEBUG_TELEMETRY_MOTORS: false,
    DISCOVERY_INTERVAL: 2000,
    HEARTBEAT_TIMEOUT: 10000,
    HEARTBEAT_CHECK_INTERVAL: 3000,
    WS_PING_INTERVAL: 10000,
    WS_PING_TIMEOUT: 15000,
    RX_BUFFER_MAX_BYTES: 256 * 1024,
    MAX_PACKET_SIZE: 8 * 1024
};
