/**
 * ESP-Drone 地面站主程序
 * ==========================
 * 应用入口点
 */

import { initThreeJS } from './visualization.js';
import { connectWS, setTelemetryCallback, setConnectionChangeCallback, log } from './websocket.js';
import { sendAction, setRemoteControlMode } from './control.js';
import { updateDashboard } from './dashboard.js';

let wsConnected = false;
let lastTelemetryTs = 0;
let droneConnected = false;
let hasDroneClient = false;

async function refreshDroneClientStatus() {
    try {
        const resp = await fetch('/api/status', { cache: 'no-store' });
        if (!resp.ok) {
            return;
        }
        const status = await resp.json();
        hasDroneClient = Array.isArray(status.clients) && status.clients.length > 0;
        updateDroneConnectionState();
    } catch (_) {
        // 忽略轮询异常，等待下一次
    }
}

function updateDroneConnectionState() {
    // 认为 5 秒内收到过遥测即飞控在线，避免瞬时抖动误判
    const telemetryAlive = (Date.now() - lastTelemetryTs) <= 5000;
    const nextState = wsConnected && (telemetryAlive || hasDroneClient);
    droneConnected = nextState;

    if (window.motorTest) {
        window.motorTest.setConnected(droneConnected);
    }

    if (window.pidTuner) {
        window.pidTuner.setConnected(droneConnected);
    }
}

// 页面切换
function switchPage(pageName) {
    document.querySelectorAll('.page').forEach(p => p.classList.remove('active'));
    document.getElementById('page-' + pageName).classList.add('active');

    document.querySelectorAll('.tab-btn').forEach(btn => btn.classList.remove('active'));
    event.target.classList.add('active');

    const nameMap = { flight: '飞行监控', motor: '电机测试', pid: 'PID调参' };
    log('切换到: ' + (nameMap[pageName] || pageName), 'info');
}

// 控制台折叠
function toggleConsole(element) {
    element.parentElement.style.height =
        element.parentElement.style.height === '30px' ? '150px' : '30px';
}

// 连接状态变化回调
function onConnectionChange(connected) {
    wsConnected = connected;
    if (!connected) {
        lastTelemetryTs = 0;
        hasDroneClient = false;
    } else {
        refreshDroneClientStatus();
    }
    updateDroneConnectionState();
}

// 扩展的遥测回调，同时更新仪表盘和电机测试模块
function onTelemetryUpdate(data) {
    lastTelemetryTs = Date.now();
    updateDroneConnectionState();

    // 更新仪表盘
    updateDashboard(data);

    // 更新电机测试模块的远程控制状态
    if (window.motorTest) {
        window.motorTest.updateRemoteState(
            data.remoteCtrlMode !== undefined ? data.remoteCtrlMode : 0,
            data.controlSource !== undefined ? data.controlSource : 0
        );
    }

    // 更新 PID 调参模块的实时曲线
    if (window.pidTuner) {
        window.pidTuner.updateTelemetry(data);
    }
}

// 暴露到全局作用域（供HTML onclick使用）
window.switchPage = switchPage;
window.toggleConsole = toggleConsole;
window.sendAction = sendAction;
window.setRemoteControlMode = setRemoteControlMode;

// 使用全局motorTest模块的函数
window.updateMotor = (id, val) => window.motorTest?.updateMotor(id, val);
window.stopAllMotors = () => window.motorTest?.stopAll();
window.setAllMotors = (val) => window.motorTest?.setAll(val);
window.sequentialTest = (thrust, duration) => window.motorTest?.sequential(thrust, duration);
window.testAllMotors = (thrust) => window.motorTest?.testAll(thrust);
window.triggerEscFirstCalibration = () => window.motorTest?.triggerEscFirstCalibration();

// 初始化
window.addEventListener('load', () => {
    initThreeJS();
    setTelemetryCallback(onTelemetryUpdate);
    setConnectionChangeCallback(onConnectionChange);
    connectWS();

    // 周期检查飞控是否仍在发送遥测，避免状态卡死在“已连接”
    setInterval(updateDroneConnectionState, 1000);
    // 周期查询服务器上的飞控客户端数量，作为无遥测时的连接兜底判断
    setInterval(() => {
        if (wsConnected) {
            refreshDroneClientStatus();
        }
    }, 1500);
});
