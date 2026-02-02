/**
 * ESP-Drone 电机测试模块
 * ==========================
 * 处理电机测试功能
 */

import { log } from './websocket.js';

export async function updateMotor(motorId, thrust) {
    const thrustInt = parseInt(thrust);
    const percent = (thrustInt / 65535 * 100).toFixed(1);

    const motorNum = motorId + 1;
    const outputEl = document.getElementById('m' + motorNum + '-output');
    const rawEl = document.getElementById('m' + motorNum + '-raw');

    if (outputEl) outputEl.textContent = percent + '%';
    if (rawEl) rawEl.textContent = thrustInt;

    try {
        await fetch('/api/motor/set', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                motorId: motorId,
                thrust: thrustInt
            })
        });
    } catch (e) {
        log('电机控制失败: ' + e.message, 'error');
    }
}

export async function stopAllMotors() {
    log('急停：停止所有电机', 'warn');

    for (let i = 0; i < 4; i++) {
        const slider = document.getElementById('slider-m' + (i + 1));
        if (slider) slider.value = 0;
        await updateMotor(i, 0);
    }

    try {
        await fetch('/api/motor/estop', { method: 'POST' });
    } catch (e) {
        log('急停失败: ' + e.message, 'error');
    }
}

export async function setAllMotors(thrust) {
    const percent = (thrust / 65535 * 100).toFixed(0);
    log('设置所有电机推力: ' + percent + '%', 'info');

    for (let i = 0; i < 4; i++) {
        const slider = document.getElementById('slider-m' + (i + 1));
        if (slider) slider.value = thrust;
        await updateMotor(i, thrust);
        await new Promise(resolve => setTimeout(resolve, 50));
    }
}

export async function sequentialTest() {
    log('开始顺序测试电机...', 'info');
    const testThrust = 6554;

    try {
        await fetch('/api/motor/sequential', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ thrust: testThrust })
        });
        log('顺序测试已启动', 'success');
    } catch (e) {
        log('顺序测试失败: ' + e.message, 'error');
    }
}

export async function testAllMotors() {
    log('全部电机测试模式', 'info');
    const testThrust = 6554;
    await setAllMotors(testThrust);
}

export function updateMotorOutputDisplay(motorOutputs) {
    if (!motorOutputs || motorOutputs.length !== 4) return;

    // 用于电机测试页面的显示（m1-output, m2-output 等）
    for (let i = 1; i <= 4; i++) {
        const key = 'm' + i;
        if (motorOutputs[i - 1] !== undefined) {
            const percent = (motorOutputs[i - 1] / 65535 * 100).toFixed(1);
            const outputEl = document.getElementById(key + '-output');
            if (outputEl) {
                outputEl.textContent = percent + '%';
                outputEl.style.color = motorOutputs[i - 1] > 0 ? 'var(--qgc-green)' : 'var(--qgc-text-muted)';
            }
        }
    }

    // 用于飞行监控仪表板的显示（motor-m1-display 等）
    const dashboardIds = ['motor-m1-display', 'motor-m2-display', 'motor-m3-display', 'motor-m4-display'];
    dashboardIds.forEach((id, idx) => {
        const element = document.getElementById(id);
        if (element && motorOutputs[idx] !== undefined) {
            const percentage = Math.round((motorOutputs[idx] / 65535) * 100);
            element.textContent = `${percentage}%`;
            element.style.color = motorOutputs[idx] > 0 ? 'var(--qgc-green)' : 'var(--qgc-text-muted)';
        }
    });
}
