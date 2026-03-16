/**
 * ESP-Drone 配置
 * ==========================
 * 服务器配置参数
 */

module.exports = {
    TCP_PORT: 8080,              // TCP 服务器端口（心跳/命令/PID参数等重要数据）
    HTTP_PORT: 3000,             // HTTP/WebSocket 端口（Web界面）
    DISCOVERY_PORT: 8081,        // UDP 广播发现端口
    UDP_TELEMETRY_PORT: 8082,    // UDP 遥测端口（高频姿态/电机/陀螺仪数据）
    VOFA_UDP_PORT: 8083,         // UDP vofa+ FireWater 协议转发端口
    DISCOVERY_INTERVAL: 2000,    // 广播间隔(ms)
    HEARTBEAT_TIMEOUT: 10000,    // 心跳超时时间(ms)
    HEARTBEAT_CHECK_INTERVAL: 3000,
    WS_PING_INTERVAL: 10000,
    WS_PING_TIMEOUT: 15000
};
