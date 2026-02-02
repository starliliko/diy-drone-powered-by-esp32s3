/**
 * ESP-Drone 遥测仪表板模块
 * ==========================
 * 处理遥测数据的UI更新
 */

import { updateDroneOrientation } from './visualization.js';
import { updateCtrlModeButtons } from './control.js';
import { log } from './websocket.js';

// 控制来源优先级信息
const ControlSourcePriority = {
    0: { name: '无控制', color: '#666', icon: '⚪', desc: '无活跃控制源' },
    1: { name: 'WiFi', color: '#2196F3', icon: '📶', desc: 'CRTP协议 (手机APP/UDP)' },
    2: { name: '地面站', color: '#4CAF50', icon: '🖥️', desc: 'TCP远程服务器' },
    3: { name: '遥控器', color: '#FF9800', icon: '🎮', desc: 'SBUS遥控器 (最高优先级)' }
};

// 控制源状态追踪
const controlSourceState = {
    current: 0,
    previous: 0,
    lastSwitchTime: 0,
    switchCount: 0,
    rcConnected: false,
    gcsConnected: false
};

export function updateDashboard(data) {
    // 姿态数据
    setText('roll-txt', data.roll.toFixed(1) + '°');
    setText('pitch-txt', data.pitch.toFixed(1) + '°');
    setText('yaw-txt', data.yaw.toFixed(1) + '°');
    setText('volt-txt', data.battVoltage.toFixed(2) + ' V');

    // 解锁状态
    const isArmed = data.isArmed;
    const armEl = document.getElementById('arming-txt');
    if (armEl) {
        armEl.textContent = isArmed ? (data.armingStateNameCN || '已解锁') : '未解锁';
        armEl.className = `t-value ${isArmed ? 'armed' : 'disarmed'}`;
    }

    // 实际飞行姿态模式
    setText('actual-mode-txt', data.actualFlightModeNameCN || '--');

    // 控制来源显示
    const ctrlSrcEl = document.getElementById('control-source-txt');
    const srcInfo = ControlSourcePriority[data.controlSource] || ControlSourcePriority[0];
    if (ctrlSrcEl) {
        ctrlSrcEl.innerHTML = `<span style="color:${srcInfo.color}">${srcInfo.icon} ${srcInfo.name}</span>`;
    }

    // 检测控制源切换
    if (data.controlSource !== controlSourceState.current) {
        const oldSrc = ControlSourcePriority[controlSourceState.current];
        const newSrc = ControlSourcePriority[data.controlSource];
        controlSourceState.previous = controlSourceState.current;
        controlSourceState.current = data.controlSource;
        controlSourceState.lastSwitchTime = Date.now();
        controlSourceState.switchCount++;

        if (controlSourceState.switchCount > 1) {
            log(`控制源切换: ${oldSrc.name} → ${newSrc.name} (${newSrc.desc})`, 'warning');
        }
    }

    // 更新连接状态
    controlSourceState.rcConnected = data.isRcConnected;
    controlSourceState.gcsConnected = data.isGcsConnected;

    // 顶部状态栏连接指示器
    const rcDot = document.getElementById('rc-dot');
    const gcsDot = document.getElementById('gcs-dot');
    if (rcDot) rcDot.className = `indicator-dot ${data.isRcConnected ? 'green' : 'red'}`;
    if (gcsDot) gcsDot.className = `indicator-dot ${data.isGcsConnected ? 'green' : 'red'}`;

    setText('mode-txt', data.flightModeNameCN || '--');
    setText('phase-txt', data.flightPhaseNameCN || '--');

    // 故障安全状态
    const fsEl = document.getElementById('failsafe-txt');
    if (fsEl) {
        fsEl.textContent = data.failsafeStateName || '正常';
        fsEl.style.color = (data.failsafeState > 0) ? 'var(--qgc-red)' : 'var(--qgc-text-main)';
    }

    // 电池状态
    const battPct = data.battPercent;
    const bBar = document.getElementById('batt-bar');
    if (bBar) {
        bBar.style.width = battPct + '%';
        bBar.style.background = (battPct < 20) ? 'var(--qgc-red)' : (battPct < 40 ? 'var(--qgc-orange)' : 'var(--qgc-green)');
    }
    setText('batt-val', battPct + '%');

    // 飞行时间
    const sec = Math.floor(data.flightTime / 1000);
    const m = Math.floor(sec / 60);
    const s = sec % 60;
    setText('time-txt', `${m}:${s.toString().padStart(2, '0')}`);

    // HUD 显示
    const hudModeEl = document.getElementById('hud-mode');
    if (hudModeEl) {
        hudModeEl.textContent = `${data.actualFlightModeNameCN || '等待'} | ${data.controlSourceNameCN || ''}`;
        hudModeEl.style.borderColor = isArmed ? 'var(--qgc-red)' : 'var(--qgc-green)';
        hudModeEl.style.color = isArmed ? 'var(--qgc-red)' : 'var(--qgc-green)';
    }

    // 更新远程控制模式按钮状态
    if (data.remoteCtrlMode !== undefined) {
        updateCtrlModeButtons(data.remoteCtrlMode);
    }

    // 更新3D模型
    updateDroneOrientation(data.roll, data.pitch, data.yaw);

    // 更新电机输出显示（使用全局motorTest模块）
    if (data.motorOutputs && window.motorTest) {
        window.motorTest.updateDisplay(data.motorOutputs);
    }
}

function setText(id, text) {
    const el = document.getElementById(id);
    if (el) el.textContent = text;
}
