#!/usr/bin/env node

/**
 * vofa+ UDP 接收测试工具
 * 用于测试 FireWater 协议数据接收
 * 
 * 使用方法:
 * node test-vofa-receiver.js [port]
 * 
 * 例如:
 * node test-vofa-receiver.js 8083
 */

const dgram = require('dgram');

const PORT = parseInt(process.argv[2] || '8083');
const server = dgram.createSocket('udp4');

let packetCount = 0;
let lastTimestamp = Date.now();

server.on('message', (msg, rinfo) => {
    const timestamp = Date.now();
    const data = msg.toString('utf-8').trim();
    
    packetCount++;
    
    // 解析 FireWater 格式
    const values = data.split(',').map(v => parseFloat(v));
    
    if (values.length === 13) {
        const [roll, pitch, yaw, gyroX, gyroY, gyroZ, accX, accY, accZ, m1, m2, m3, m4] = values;
        
        // 每10个包输出一条日志
        if (packetCount % 10 === 0) {
            console.log(`[${new Date().toLocaleTimeString()}] Packet #${packetCount}`);
            console.log(`  Attitude: roll=${roll.toFixed(2)}°, pitch=${pitch.toFixed(2)}°, yaw=${yaw.toFixed(2)}°`);
            console.log(`  Gyro: X=${gyroX.toFixed(3)}, Y=${gyroY.toFixed(3)}, Z=${gyroZ.toFixed(3)} rad/s`);
            console.log(`  Accel: X=${accX.toFixed(2)}, Y=${accY.toFixed(2)}, Z=${accZ.toFixed(2)} g`);
            console.log(`  Motors: M1=${m1.toFixed(0)}, M2=${m2.toFixed(0)}, M3=${m3.toFixed(0)}, M4=${m4.toFixed(0)}`);
            console.log(`  Source: ${rinfo.address}:${rinfo.port}\n`);
        }
        
        // 计算实际接收频率
        if (timestamp - lastTimestamp >= 1000) {
            console.log(`[INFO] 接收速率: ${packetCount} packets/sec`);
            packetCount = 0;
            lastTimestamp = timestamp;
        }
    } else {
        console.warn(`[WARN] 数据格式错误: 期望13个通道，收到${values.length}个`);
        console.warn(`       原始数据: ${data.substring(0, 100)}`);
    }
});

server.on('error', (err) => {
    console.error('Server error:', err);
    server.close();
});

server.bind(PORT, () => {
    console.log(`\n🎯 vofa+ FireWater 接收测试`);
    console.log(`监听端口: UDP:${PORT}`);
    console.log(`预期频率: 50Hz (20ms/包)`);
    console.log(`\n等待数据...（Ctrl+C 退出）\n`);
});

// 优雅关闭
process.on('SIGINT', () => {
    console.log('\n\n关闭接收器...');
    server.close();
    process.exit(0);
});
