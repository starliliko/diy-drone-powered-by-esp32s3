#!/usr/bin/env node

/**
 * ESP-Drone 电机测试脚本
 * 
 * 用法:
 * node motor_test.js <mode> [motorId] [thrust]
 * 
 * 示例:
 * node motor_test.js stop                    - 停止所有电机
 * node motor_test.js sequential              - 顺序测试
 * node motor_test.js single 0 6554           - 测试电机0，推力10%
 * node motor_test.js all 6554                - 所有电机10%推力
 */

const net = require('net');

const CONFIG = {
    SERVER_HOST: '192.168.0.136',  // 飞控IP地址，需要修改为实际地址
    SERVER_PORT: 8080,
    TCP_TIMEOUT: 5000
};

const MAGIC = [0xAB, 0xCD];
const PROTOCOL_VERSION = 0x01;

const PacketType = {
    CONTROL: 0x02,
    CRTP: 0x03
};

/**
 * 创建包头
 */
function createHeader(pktType, seq, payloadLength) {
    const buffer = Buffer.allocUnsafe(8);
    buffer.writeUInt8(MAGIC[0], 0);
    buffer.writeUInt8(MAGIC[1], 1);
    buffer.writeUInt8(PROTOCOL_VERSION, 2);
    buffer.writeUInt8(pktType, 3);
    buffer.writeUInt16LE(seq, 4);
    buffer.writeUInt16LE(payloadLength, 6);
    return buffer;
}

/**
 * 发送参数设置命令
 */
function sendMotorParam(socket, paramName, motorId, value, seq) {
    // CRTP 参数写入
    // 格式: [PORT(2), CHANNEL, GROUP, NAME, DATA...]
    const payload = Buffer.allocUnsafe(20);
    let offset = 0;

    payload.writeUInt8(0x02, offset++);  // PARAM port
    payload.writeUInt8(0x02, offset++);  // channel 2 (SET)
    payload.writeUInt8(motorId, offset++); // 参数值

    // 发送包
    const header = createHeader(PacketType.CRTP, seq, offset);
    socket.write(Buffer.concat([header, payload.slice(0, offset)]));
}

/**
 * 连接到飞控并发送测试命令
 */
function testMotors(mode, motorId = 0, thrust = 0) {
    const socket = new net.Socket();
    let seq = 0;

    socket.setTimeout(CONFIG.TCP_TIMEOUT);

    socket.on('connect', () => {
        console.log(`✓ 已连接到飞控: ${CONFIG.SERVER_HOST}:${CONFIG.SERVER_PORT}`);

        // 根据模式发送不同的命令
        switch (mode) {
            case 'stop':
                console.log('📍 停止所有电机 (mode=0)');
                sendMotorParam(socket, 'mode', 0, 0, seq++);
                break;

            case 'sequential':
                console.log('📍 顺序测试电机 (mode=1)');
                sendMotorParam(socket, 'mode', 1, 0, seq++);
                break;

            case 'single':
                console.log(`📍 测试电机M${motorId + 1} (mode=2, thrust=${thrust})`);
                sendMotorParam(socket, 'id', motorId, 0, seq++);
                setTimeout(() => {
                    sendMotorParam(socket, 'mode', 2, 0, seq++);
                }, 100);
                break;

            case 'all':
                console.log(`📍 所有电机输出 (mode=3, thrust=${thrust})`);
                // thrust是第一个参数位置的值
                sendMotorParam(socket, 'mode', 3, 0, seq++);
                break;

            default:
                console.log('❌ 未知模式: ' + mode);
                socket.destroy();
                return;
        }

        setTimeout(() => {
            console.log('✓ 命令已发送');
            socket.destroy();
        }, 500);
    });

    socket.on('data', (data) => {
        // 接收来自飞控的数据，这里只是打印
        console.log(`📥 收到数据 (${data.length} bytes)`);
    });

    socket.on('error', (err) => {
        console.error('❌ 连接错误:', err.message);
        if (err.code === 'ECONNREFUSED') {
            console.log('💡 提示: 确保飞控已启动并连接到网络');
            console.log(`💡 尝试Ping: ping ${CONFIG.SERVER_HOST}`);
        }
    });

    socket.on('timeout', () => {
        console.error('❌ 连接超时');
        socket.destroy();
    });

    socket.on('close', () => {
        console.log('✓ 连接已关闭');
    });

    console.log(`🔌 正在连接到 ${CONFIG.SERVER_HOST}:${CONFIG.SERVER_PORT}...`);
    socket.connect(CONFIG.SERVER_PORT, CONFIG.SERVER_HOST);
}

// 解析命令行参数
const args = process.argv.slice(2);

if (args.length === 0) {
    console.log(`
ESP-Drone 电机测试脚本

用法:
  node motor_test.js stop                     # 停止所有电机
  node motor_test.js sequential               # 顺序测试 (M1→M2→M3→M4)
  node motor_test.js single <motorId> <thrust>
  node motor_test.js all <thrust>
  
参数说明:
  motorId: 0=M1, 1=M2, 2=M3, 3=M4
  thrust:  推力值 (0-65535)
           0 = 0%, 6554 = 10%, 13107 = 20%, 65535 = 100%

示例:
  node motor_test.js single 0 6554            # 测试M1，10%推力
  node motor_test.js all 6554                 # 全部电机10%推力
  node motor_test.js all 13107                # 全部电机20%推力
    `);
    process.exit(1);
}

const mode = args[0];
const motorId = args[1] ? parseInt(args[1]) : 0;
const thrust = args[2] ? parseInt(args[2]) : 6554;

testMotors(mode, motorId, thrust);
