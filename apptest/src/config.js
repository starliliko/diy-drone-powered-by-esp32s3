/**
 * ESP-Drone 配置
 * ==========================
 * 服务器配置参数
 */

module.exports = {
    TCP_PORT: 8080,           // TCP 服务器端口（接收飞控数据）
    HTTP_PORT: 3000,          // HTTP/WebSocket 端口（Web界面）
    DISCOVERY_PORT: 8081,     // UDP 广播发现端口
    DISCOVERY_INTERVAL: 2000, // 广播间隔(ms)
    HEARTBEAT_TIMEOUT: 10000, // 心跳超时时间(ms)
    HEARTBEAT_CHECK_INTERVAL: 3000,
    WS_PING_INTERVAL: 10000,
    WS_PING_TIMEOUT: 15000
};
