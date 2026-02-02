/**
 * ESP-Drone 地面站主程序
 * ==========================
 * 应用入口点
 */

import { initThreeJS } from './visualization.js';
import { connectWS, setTelemetryCallback, log } from './websocket.js';
import { sendAction, setRemoteControlMode } from './control.js';
import { updateMotor, stopAllMotors, setAllMotors, sequentialTest, testAllMotors } from './motorTest.js';
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

// 暴露到全局作用域（供HTML onclick使用）
window.switchPage = switchPage;
window.toggleConsole = toggleConsole;
window.sendAction = sendAction;
window.setRemoteControlMode = setRemoteControlMode;
window.updateMotor = updateMotor;
window.stopAllMotors = stopAllMotors;
window.setAllMotors = setAllMotors;
window.sequentialTest = sequentialTest;
window.testAllMotors = testAllMotors;

// 初始化
window.addEventListener('load', () => {
    initThreeJS();
    setTelemetryCallback(updateDashboard);
    connectWS();
});
