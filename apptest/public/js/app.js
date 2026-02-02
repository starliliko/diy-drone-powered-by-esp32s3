/**
 * ESP-Drone 地面站主程序
 * ==========================
 * 应用入口点
 */

import { initThreeJS } from './visualization.js';
import { connectWS, setTelemetryCallback, setConnectionChangeCallback, log } from './websocket.js';
import { sendAction, setRemoteControlMode } from './control.js';
import { updateDashboard } from './dashboard.js';

// 页面切换
function switchPage(pageName) {
    document.querySelectorAll('.page').forEach(p => p.classList.remove('active'));
    document.getElementById('page-' + pageName).classList.add('active');

    document.querySelectorAll('.tab-btn').forEach(btn => btn.classList.remove('active'));
    event.target.classList.add('active');

    log('切换到: ' + (pageName === 'flight' ? '飞行监控' : '电机测试'), 'info');
}

// 控制台折叠
function toggleConsole(element) {
    element.parentElement.style.height =
        element.parentElement.style.height === '30px' ? '150px' : '30px';
}

// 连接状态变化回调
function onConnectionChange(connected) {
    // 更新电机测试模块的连接状态
    if (window.motorTest) {
        window.motorTest.setConnected(connected);
    }
}

// 扩展的遥测回调，同时更新仪表盘和电机测试模块
function onTelemetryUpdate(data) {
    // 更新仪表盘
    updateDashboard(data);

    // 更新电机测试模块的远程控制状态
    if (window.motorTest) {
        window.motorTest.updateRemoteState(
            data.remoteCtrlMode !== undefined ? data.remoteCtrlMode : 0,
            data.controlSource !== undefined ? data.controlSource : 0
        );
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

// 初始化
window.addEventListener('load', () => {
    initThreeJS();
    setTelemetryCallback(onTelemetryUpdate);
    setConnectionChangeCallback(onConnectionChange);
    connectWS();
});
