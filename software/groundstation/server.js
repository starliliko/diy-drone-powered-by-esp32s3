/**
 * ESP Drone 地面站服务器入口
 * 模块化架构 - 主入口文件
 */

const DroneServer = require('./src/DroneServer');
const config = require('./src/config');

// 创建服务器实例
const server = new DroneServer();

// 启动服务器
server.start();

console.log('');
console.log('╔════════════════════════════════════════════════════════════╗');
console.log('║         ESP-Drone QGroundControl Lite 地面站               ║');
console.log('╠════════════════════════════════════════════════════════════╣');
console.log(`║  HTTP 服务:    http://localhost:${config.HTTP_PORT}                      ║`);
console.log(`║  WebSocket:    ws://localhost:${config.HTTP_PORT}                        ║`);
console.log(`║  TCP 端口:     ${config.TCP_PORT} (等待无人机连接)                   ║`);
console.log('╠════════════════════════════════════════════════════════════╣');
console.log('║  按 Ctrl+C 停止服务器                                      ║');
console.log('╚════════════════════════════════════════════════════════════╝');
console.log('');

// 优雅退出处理
process.on('SIGINT', () => {
    console.log('\n[INFO] 正在关闭服务器...');
    server.stop();
    process.exit(0);
});

process.on('SIGTERM', () => {
    console.log('\n[INFO] 收到终止信号，正在关闭...');
    server.stop();
    process.exit(0);
});

// 未捕获异常处理
process.on('uncaughtException', (err) => {
    console.error('[ERROR] 未捕获的异常:', err);
});

process.on('unhandledRejection', (reason, promise) => {
    console.error('[ERROR] 未处理的 Promise 拒绝:', reason);
});
