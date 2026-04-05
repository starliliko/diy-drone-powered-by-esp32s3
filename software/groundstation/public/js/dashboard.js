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

const protectionStateTracker = {
    initialized: false,
    key: 'NORMAL'
};

const FailsafeNameCN = {
    0: '无',
    1: '低电量',
    2: '严重低电',
    3: '遥控信号丢失',
    4: '地面站信号丢失',
    5: '传感器故障',
    6: '地理围栏'
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

    // 失控保护状态展示与日志
    const protection = evaluateProtectionState(data);
    updateProtectionUI(protection);
    emitProtectionLog(protection);

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

    // V3.2 新增：更新飞行数据面板
    setText('est-alt-txt', (data.estAltitude !== undefined ? data.estAltitude.toFixed(2) : '0.00') + ' m');
    setText('baro-alt-txt', (data.baroAltitude !== undefined ? data.baroAltitude.toFixed(2) : '0.00') + ' m');
    setText('tof-alt-txt', (data.tofDistance !== undefined ? data.tofDistance.toFixed(2) : '0.00') + ' m');
    setText('vel-x-txt', (data.estVelX !== undefined ? data.estVelX.toFixed(2) : '0.00') + ' m/s');
    setText('vel-y-txt', (data.estVelY !== undefined ? data.estVelY.toFixed(2) : '0.00') + ' m/s');
    setText('vel-z-txt', (data.estVelZ !== undefined ? data.estVelZ.toFixed(2) : '0.00') + ' m/s');
    // V3.3 机体坐标系速度
    setText('body-vel-x-txt', (data.bodyVelX !== undefined ? data.bodyVelX.toFixed(2) : '0.00') + ' m/s');
    setText('body-vel-y-txt', (data.bodyVelY !== undefined ? data.bodyVelY.toFixed(2) : '0.00') + ' m/s');
}

function evaluateProtectionState(data) {
    const failsafeCn = FailsafeNameCN[data.failsafeState] || '未知';

    if (data.isTumbled) {
        return {
            key: 'SITAW_TUMBLED',
            level: 'active',
            state: '保护触发',
            reason: '翻覆检测触发急停(SITAW_TU)',
            action: '已切断电机输出，需排除风险后复位解锁'
        };
    }

    if (data.isEmergency) {
        return {
            key: 'EMERGENCY',
            level: 'active',
            state: '保护触发',
            reason: '紧急停机(E-STOP)已触发',
            action: '已切断电机输出并强制上锁'
        };
    }

    if (data.isArmThrottleBlocked) {
        return {
            key: 'ARM_THROTTLE_BLOCK',
            level: 'warn',
            state: '保护预警',
            reason: '解锁后高油门保护拦截已激活',
            action: '请先将油门拉到低位保持后，再缓慢拉高'
        };
    }

    if (data.failsafeState === 3) {
        return {
            key: 'FS_RC_LOSS',
            level: 'active',
            state: '保护触发',
            reason: `故障安全: ${failsafeCn}`,
            action: '强制上锁并将推力置零'
        };
    }

    if (data.failsafeState === 2) {
        return {
            key: 'FS_CRITICAL_BATTERY',
            level: 'active',
            state: '保护触发',
            reason: `故障安全: ${failsafeCn}`,
            action: '进入严重低电保护，建议立即降落'
        };
    }

    if (data.failsafeState > 0) {
        return {
            key: `FS_${data.failsafeState}`,
            level: 'warn',
            state: '保护预警',
            reason: `故障安全: ${failsafeCn}`,
            action: '已进入保护状态，请尽快处置'
        };
    }

    if (data.armingState === 3) {
        return {
            key: 'ARMING_STANDBY_ERROR',
            level: 'warn',
            state: '保护预警',
            reason: '解锁状态异常(STANDBY_ERROR)',
            action: '禁止解锁，需排除故障后复位'
        };
    }

    if (data.isArmed && data.controlSource === 0) {
        return {
            key: 'NO_CONTROL_SOURCE',
            level: 'warn',
            state: '保护预警',
            reason: '已解锁但无有效控制源',
            action: '等待控制链路恢复，超时将触发看门狗保护'
        };
    }

    return {
        key: 'NORMAL',
        level: 'normal',
        state: '正常',
        reason: '无',
        action: '监控中'
    };
}

function updateProtectionUI(protection) {
    const stateEl = document.getElementById('protection-state-txt');
    if (stateEl) {
        stateEl.textContent = protection.state;
        stateEl.className = `t-value protection-state ${protection.level}`;
    }

    setText('protection-reason-txt', protection.reason);
    setText('protection-action-txt', protection.action);
}

function emitProtectionLog(protection) {
    if (!protectionStateTracker.initialized) {
        protectionStateTracker.initialized = true;
        protectionStateTracker.key = protection.key;
        return;
    }

    if (protection.key === protectionStateTracker.key) {
        return;
    }

    const prevKey = protectionStateTracker.key;
    protectionStateTracker.key = protection.key;

    if (protection.key === 'NORMAL') {
        log('失控保护解除：状态恢复正常（措施：恢复常规监控）', 'success');
        return;
    }

    const levelToLogType = {
        active: 'error',
        warn: 'warn',
        normal: 'info'
    };
    const logType = levelToLogType[protection.level] || 'warn';

    const prefix = prevKey === 'NORMAL' ? '失控保护触发' : '失控保护切换';
    log(`${prefix}：原因=${protection.reason}；措施=${protection.action}`, logType);
}

function setText(id, text) {
    const el = document.getElementById(id);
    if (el) el.textContent = text;
}
