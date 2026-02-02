/**
 * ESP-Drone Motor Test UI Module
 * 支持 QGC 风格的电机测试功能
 * 
 * 功能特性:
 * 1. 单电机测试时自动将其他电机归零
 * 2. 未连接时禁用所有电机测试功能
 * 3. 电机测试时正确显示控制源状态
 * 4. 3秒无操作自动停止电机（安全超时）
 * 5. 只有在远程控制"启用"模式下可用，协同模式不可用
 */

// 远程控制模式常量
const REMOTE_CTRL_MODE = {
    DISABLED: 0,  // 禁用
    ENABLED: 1,   // 启用（只有此模式可用电机测试）
    SHARED: 2     // 协同（不可用电机测试）
};

// 状态变量
let isDroneConnected = false;
let motorTestActive = false;
let autoResetTimer = null;
let currentRemoteCtrlMode = REMOTE_CTRL_MODE.DISABLED;
let currentControlSource = 0; // 当前控制源
const AUTO_RESET_TIMEOUT = 3000; // 3秒自动停止超时
const MAX_THRUST = 65535; // 最大推力值

// 4个电机的当前值缓存 (0-65535 raw 值)
let motorValues = [0, 0, 0, 0];

// 电机滑块和输出显示的元素 ID 映射
const MOTOR_IDS = ['m1', 'm2', 'm3', 'm4'];

/**
 * 检查电机测试是否可用
 * 只有在：连接 + 远程控制启用 + 控制源为地面站 时可用
 */
function isMotorTestAvailable() {
    // 必须已连接
    if (!isDroneConnected) return false;
    // 远程控制模式必须为"启用"（不是协同或禁用）
    if (currentRemoteCtrlMode !== REMOTE_CTRL_MODE.ENABLED) return false;
    // 控制源必须是地面站 (2) 或无控制 (0)
    // 如果是遥控器 (3)，则不允许电机测试
    if (currentControlSource === 3) return false;
    return true;
}

/**
 * 初始化电机测试模块
 */
function initMotorTest() {
    // 初始化 UI 状态
    updateMotorTestUIState();

    // 给每个滑块添加事件监听
    for (let i = 0; i < 4; i++) {
        const slider = document.getElementById(`slider-${MOTOR_IDS[i]}`);
        if (slider) {
            slider.addEventListener('input', (e) => {
                // 实时更新显示
                const rawValue = parseInt(e.target.value);
                const percent = Math.round((rawValue / MAX_THRUST) * 100);
                const outputEl = document.getElementById(`${MOTOR_IDS[i]}-output`);
                const rawEl = document.getElementById(`${MOTOR_IDS[i]}-raw`);
                if (outputEl) {
                    outputEl.textContent = `${percent}%`;
                }
                if (rawEl) {
                    rawEl.textContent = rawValue;
                }
            });
        }
    }

    console.log('[MotorTest] 电机测试模块初始化完成');
}

/**
 * 设置无人机连接状态
 * @param {boolean} connected - 是否已连接
 */
function setDroneConnected(connected) {
    isDroneConnected = connected;
    console.log('[MotorTest] 连接状态:', connected ? '已连接' : '未连接');
    updateMotorTestUIState();

    // 断开连接时停止电机测试并重置状态
    if (!connected) {
        motorTestActive = false;
        resetAutoTimer();
        for (let i = 0; i < 4; i++) {
            motorValues[i] = 0;
        }
        updateAllSliders();
    }
}

/**
 * 更新远程控制模式和控制源
 * @param {number} remoteCtrlMode - 远程控制模式 (0=禁用, 1=启用, 2=协同)
 * @param {number} controlSource - 控制源 (0=无, 1=WiFi, 2=地面站, 3=遥控器)
 */
function updateRemoteState(remoteCtrlMode, controlSource) {
    const prevMode = currentRemoteCtrlMode;
    const prevSource = currentControlSource;

    currentRemoteCtrlMode = remoteCtrlMode;
    currentControlSource = controlSource;

    if (prevMode !== remoteCtrlMode || prevSource !== controlSource) {
        console.log(`[MotorTest] 远程控制模式: ${remoteCtrlMode}, 控制源: ${controlSource}`);
        updateMotorTestUIState();

        // 如果电机测试不再可用，自动停止
        if (!isMotorTestAvailable() && motorTestActive) {
            console.log('[MotorTest] 电机测试条件不再满足，自动停止');
            stopAllMotors();
        }
    }
}

/**
 * 更新电机测试 UI 状态（启用/禁用）
 */
function updateMotorTestUIState() {
    const available = isMotorTestAvailable();
    const disabled = !available;

    // 禁用/启用所有电机滑块
    for (let i = 0; i < 4; i++) {
        const slider = document.getElementById(`slider-${MOTOR_IDS[i]}`);
        if (slider) {
            slider.disabled = disabled;
            if (disabled) {
                slider.style.opacity = '0.5';
                slider.style.cursor = 'not-allowed';
            } else {
                slider.style.opacity = '1';
                slider.style.cursor = 'pointer';
            }
        }
    }

    // 禁用/启用所有测试按钮
    const testButtons = document.querySelectorAll('.test-btn');
    testButtons.forEach(btn => {
        btn.disabled = disabled;
        if (disabled) {
            btn.style.opacity = '0.5';
            btn.style.cursor = 'not-allowed';
        } else {
            btn.style.opacity = '1';
            btn.style.cursor = 'pointer';
        }
    });

    // 更新状态显示
    const statusEl = document.getElementById('motor-test-status');
    if (statusEl) {
        if (!isDroneConnected) {
            statusEl.textContent = '未连接';
            statusEl.className = 'status-badge status-disconnected';
            statusEl.title = '无人机未连接';
        } else if (currentRemoteCtrlMode === REMOTE_CTRL_MODE.DISABLED) {
            statusEl.textContent = '不可用';
            statusEl.className = 'status-badge status-unavailable';
            statusEl.title = '远程控制已禁用';
        } else if (currentRemoteCtrlMode === REMOTE_CTRL_MODE.SHARED) {
            statusEl.textContent = '不可用';
            statusEl.className = 'status-badge status-unavailable';
            statusEl.title = '协同模式下不可用电机测试';
        } else if (currentControlSource === 3) {
            statusEl.textContent = '不可用';
            statusEl.className = 'status-badge status-unavailable';
            statusEl.title = '遥控器控制中，不可用电机测试';
        } else if (motorTestActive) {
            statusEl.textContent = '测试中';
            statusEl.className = 'status-badge status-testing';
            statusEl.title = '电机测试进行中';
        } else {
            statusEl.textContent = '就绪';
            statusEl.className = 'status-badge status-ready';
            statusEl.title = '可以进行电机测试';
        }
    }
}

/**
 * 更新所有滑块到当前缓存的值
 */
function updateAllSliders() {
    for (let i = 0; i < 4; i++) {
        const slider = document.getElementById(`slider-${MOTOR_IDS[i]}`);
        const outputEl = document.getElementById(`${MOTOR_IDS[i]}-output`);
        const rawEl = document.getElementById(`${MOTOR_IDS[i]}-raw`);
        const percent = Math.round((motorValues[i] / MAX_THRUST) * 100);

        if (slider) {
            slider.value = motorValues[i];
        }
        if (outputEl) {
            outputEl.textContent = `${percent}%`;
        }
        if (rawEl) {
            rawEl.textContent = motorValues[i];
        }
    }
}

/**
 * 重置自动停止定时器
 */
function resetAutoTimer() {
    if (autoResetTimer) {
        clearTimeout(autoResetTimer);
        autoResetTimer = null;
    }
}

/**
 * 启动自动停止定时器（3秒后自动停止所有电机）
 */
function startAutoResetTimer() {
    resetAutoTimer();

    autoResetTimer = setTimeout(() => {
        console.log('[MotorTest] 安全超时：自动停止所有电机');
        stopAllMotors();
    }, AUTO_RESET_TIMEOUT);
}

/**
 * 更新单个电机输出
 * 同时将其他电机归零（单电机测试模式）
 * @param {number} motorIndex - 电机索引 (0-3)
 * @param {number} rawValue - 原始输出值 (0-65535)
 */
async function updateMotor(motorIndex, rawValue) {
    if (!isDroneConnected) {
        console.warn('[MotorTest] 未连接，无法发送电机命令');
        return;
    }

    const value = parseInt(rawValue);
    motorTestActive = true;
    updateMotorTestUIState();

    // 单电机测试模式：其他电机归零
    for (let i = 0; i < 4; i++) {
        if (i === motorIndex) {
            motorValues[i] = value;
        } else {
            motorValues[i] = 0;
        }
    }

    // 更新所有滑块显示
    updateAllSliders();

    // 重启自动停止定时器
    startAutoResetTimer();

    // 发送命令到服务器 (motorId 0-3, thrust 0-65535 原始值)
    try {
        const response = await fetch('/api/motor/set', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ motorId: motorIndex, thrust: value })
        });

        if (!response.ok) {
            console.error('[MotorTest] 电机命令发送失败:', response.status);
        }
    } catch (error) {
        console.error('[MotorTest] 发送电机命令错误:', error);
    }
}

/**
 * 停止所有电机
 */
async function stopAllMotors() {
    motorTestActive = false;
    resetAutoTimer();

    // 重置所有电机值
    for (let i = 0; i < 4; i++) {
        motorValues[i] = 0;
    }
    updateAllSliders();
    updateMotorTestUIState();

    if (!isDroneConnected) {
        console.warn('[MotorTest] 未连接');
        return;
    }

    try {
        const response = await fetch('/api/motor/estop', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' }
        });

        if (response.ok) {
            console.log('[MotorTest] 紧急停止成功');
        }
    } catch (error) {
        console.error('[MotorTest] 紧急停止失败:', error);
    }
}

/**
 * 同时设置所有电机到相同的值
 * @param {number} rawValue - 原始输出值 (0-65535)
 */
async function setAllMotors(rawValue) {
    if (!isDroneConnected) {
        console.warn('[MotorTest] 未连接，无法发送电机命令');
        return;
    }

    const value = parseInt(rawValue);
    motorTestActive = true;

    // 设置所有电机相同值
    for (let i = 0; i < 4; i++) {
        motorValues[i] = value;
    }
    updateAllSliders();
    updateMotorTestUIState();

    // 重启自动停止定时器
    startAutoResetTimer();

    try {
        const response = await fetch('/api/motor/all', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ thrust: value })
        });

        if (!response.ok) {
            console.error('[MotorTest] 全电机命令发送失败:', response.status);
        }
    } catch (error) {
        console.error('[MotorTest] 发送全电机命令错误:', error);
    }
}

/**
 * 顺序测试所有电机
 * 每个电机以指定推力运行指定时长，然后切换到下一个
 * @param {number} thrustPercent - 推力百分比 (0-100)
 * @param {number} durationMs - 每个电机持续时间(毫秒)
 */
async function sequentialTest(thrustPercent = 15, durationMs = 2000) {
    if (!isDroneConnected) {
        console.warn('[MotorTest] 未连接，无法执行顺序测试');
        return;
    }

    motorTestActive = true;
    updateMotorTestUIState();
    resetAutoTimer(); // 顺序测试期间暂停自动停止

    const rawValue = Math.round((thrustPercent / 100) * MAX_THRUST);
    console.log(`[MotorTest] 开始顺序测试: ${thrustPercent}% 推力 (raw=${rawValue}), ${durationMs}ms 每电机`);

    try {
        const response = await fetch('/api/motor/sequential', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                thrust: rawValue,
                duration: durationMs
            })
        });

        if (response.ok) {
            // 模拟 UI 更新
            for (let i = 0; i < 4; i++) {
                // 设置当前电机
                for (let j = 0; j < 4; j++) {
                    motorValues[j] = (j === i) ? rawValue : 0;
                }
                updateAllSliders();

                await new Promise(r => setTimeout(r, durationMs));
            }

            // 测试完成，复位
            stopAllMotors();
            console.log('[MotorTest] 顺序测试完成');
        }
    } catch (error) {
        console.error('[MotorTest] 顺序测试失败:', error);
        stopAllMotors();
    }
}

/**
 * 测试所有电机（同时启动）
 * @param {number} thrustPercent - 推力百分比 (0-100)
 */
async function testAllMotors(thrustPercent = 10) {
    const rawValue = Math.round((thrustPercent / 100) * MAX_THRUST);
    console.log(`[MotorTest] 测试所有电机: ${thrustPercent}%`);
    await setAllMotors(rawValue);
}

/**
 * 更新电机输出显示（从遥测数据）
 * @param {Object|Array} data - 电机遥测数据 {m1, m2, m3, m4} 或 [m1, m2, m3, m4]
 */
function updateMotorOutputDisplay(data) {
    // 如果正在测试中，不更新UI（避免覆盖用户设置）
    if (motorTestActive) {
        return;
    }

    // 支持数组格式 [m1, m2, m3, m4] 或对象格式 {m1, m2, m3, m4}
    let motors;
    if (Array.isArray(data)) {
        motors = data;
    } else if (data && typeof data.m1 !== 'undefined') {
        motors = [data.m1, data.m2, data.m3, data.m4];
    } else {
        return;
    }

    // 从遥测数据更新显示
    for (let i = 0; i < 4; i++) {
        // 将原始值(0-65535)转换为百分比
        const percent = Math.round((motors[i] / 65535) * 100);
        // 更新飞行监控页面的电机显示
        const display = document.getElementById(`motor-m${i + 1}-display`);
        if (display) {
            display.textContent = `${percent}%`;
        }
        // 也更新电机测试页面的显示
        const output = document.getElementById(`m${i + 1}-output`);
        if (output) {
            output.textContent = `${percent}%`;
        }
    }
}

/**
 * 获取当前电机测试状态
 * @returns {Object} 状态对象
 */
function getMotorTestStatus() {
    return {
        connected: isDroneConnected,
        active: motorTestActive,
        available: isMotorTestAvailable(),
        remoteCtrlMode: currentRemoteCtrlMode,
        controlSource: currentControlSource,
        motors: [...motorValues]
    };
}

// 导出到全局
window.motorTest = {
    init: initMotorTest,
    setConnected: setDroneConnected,
    updateRemoteState: updateRemoteState,
    updateMotor: updateMotor,
    stopAll: stopAllMotors,
    setAll: setAllMotors,
    sequential: sequentialTest,
    testAll: testAllMotors,
    updateDisplay: updateMotorOutputDisplay,
    getStatus: getMotorTestStatus,
    isAvailable: isMotorTestAvailable
};

// 页面加载后初始化
document.addEventListener('DOMContentLoaded', () => {
    initMotorTest();
});
