/**
 * ESP-Drone 控制命令模块
 * ==========================
 * 处理飞控控制指令
 */

import { log } from './websocket.js';

// 控制指令类型
const ControlCmdType = {
    RPYT: 0x00,
    VELOCITY: 0x01,
    POSITION: 0x02,
    HOVER: 0x03,
    LAND: 0x04,
    EMERGENCY: 0x05,
    ARM: 0x06,
    DISARM: 0x07,
    SET_CONTROL_MODE: 0x10
};

export async function sendAction(action) {
    const actionNames = {
        'ARM': '发送解锁指令',
        'DISARM': '发送上锁指令',
        'TAKEOFF': '发送起飞指令',
        'LAND': '发送降落指令',
        'RTL': '发送返航指令',
        'HOVER': '发送悬停指令'
    };

    log(actionNames[action] || '未知指令: ' + action, 'info');

    const payload = { cmdType: 0, roll: 0, pitch: 0, yaw: 0, thrust: 0, mode: 0 };

    switch (action) {
        case 'ARM': payload.cmdType = ControlCmdType.ARM; break;
        case 'DISARM': payload.cmdType = ControlCmdType.DISARM; break;
        case 'LAND': payload.cmdType = ControlCmdType.LAND; break;
        case 'HOVER': payload.cmdType = ControlCmdType.HOVER; break;
        case 'TAKEOFF': payload.cmdType = ControlCmdType.HOVER; break;
        case 'RTL': payload.cmdType = ControlCmdType.EMERGENCY; break;
    }

    try {
        await fetch('/api/control', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(payload)
        });
    } catch (e) {
        log(e.message, 'error');
    }
}

export async function setRemoteControlMode(mode) {
    const modeNames = ['禁用', '启用', '协同'];
    log('设置远程控制模式: ' + modeNames[mode], 'info');

    const payload = {
        cmdType: ControlCmdType.SET_CONTROL_MODE,
        roll: 0, pitch: 0, yaw: 0, thrust: 0,
        mode: mode
    };

    try {
        await fetch('/api/control', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(payload)
        });
        updateCtrlModeButtons(mode);
    } catch (e) {
        log('发送控制模式失败: ' + e.message, 'error');
    }
}

export function updateCtrlModeButtons(activeMode) {
    const modeNames = ['禁用', '启用', '协同'];
    const modeIcons = ['🚫', '✅', '🤝'];

    const textEl = document.getElementById('ctrl-mode-text');
    const iconEl = document.querySelector('.ctrl-mode-icon');

    if (textEl) textEl.textContent = modeNames[activeMode] || '未知';
    if (iconEl) iconEl.textContent = modeIcons[activeMode] || '⚙️';

    const radioId = ['mode-disabled', 'mode-enabled', 'mode-shared'][activeMode];
    if (radioId) {
        const radio = document.getElementById(radioId);
        if (radio) radio.checked = true;
    }
}
