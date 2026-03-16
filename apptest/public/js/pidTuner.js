/**
 * ESP-Drone PID 调参模块 v2
 * ==========================
 * 多通道实时曲线 + PID 参数调节 + 参数持久化
 */
;(function() {
"use strict";

// ========== 图表通道定义 ==========
var CHART_CHANNELS = {
    attitude: {
        label: '姿态角',
        unit: 'deg',
        color: '#2c9edd',
        enabled: true,
        series: [
            { key: 'roll',  label: 'Roll',  color: '#2c9edd' },
            { key: 'pitch', label: 'Pitch', color: '#27d965' },
            { key: 'yaw',   label: 'Yaw',   color: '#d99927' }
        ]
    },
    gyro: {
        label: '角速度',
        unit: 'deg/s',
        color: '#e74c3c',
        enabled: true,
        series: [
            { key: 'gyroX', label: 'Gyro X', color: '#e74c3c' },
            { key: 'gyroY', label: 'Gyro Y', color: '#2ecc71' },
            { key: 'gyroZ', label: 'Gyro Z', color: '#f39c12' }
        ]
    },
    motors: {
        label: '电机输出',
        unit: '',
        color: '#9b59b6',
        enabled: false,
        series: [
            { key: 'motor0', label: 'M1', color: '#e74c3c' },
            { key: 'motor1', label: 'M2', color: '#3498db' },
            { key: 'motor2', label: 'M3', color: '#2ecc71' },
            { key: 'motor3', label: 'M4', color: '#f39c12' }
        ]
    },
    accel: {
        label: '加速度',
        unit: 'mg',
        color: '#1abc9c',
        enabled: false,
        series: [
            { key: 'accX', label: 'Acc X', color: '#1abc9c' },
            { key: 'accY', label: 'Acc Y', color: '#e67e22' },
            { key: 'accZ', label: 'Acc Z', color: '#8e44ad' }
        ]
    },
    altitude: {
        label: '高度/距离',
        unit: 'm',
        color: '#00bcd4',
        enabled: false,
        series: [
            { key: 'estAltitude',  label: '估计高度', color: '#00bcd4' },
            { key: 'baroAltitude', label: '气压高度', color: '#ff9800' },
            { key: 'tofDistance',  label: 'ToF距离',  color: '#4caf50' }
        ]
    },
    velocity: {
        label: '速度',
        unit: 'm/s',
        color: '#ff5722',
        enabled: false,
        series: [
            { key: 'estVelX', label: 'Vel X', color: '#ff5722' },
            { key: 'estVelY', label: 'Vel Y', color: '#03a9f4' },
            { key: 'estVelZ', label: 'Vel Z', color: '#8bc34a' }
        ]
    }
};

// ========== PID 参数定义 ==========
var PID_GROUPS = {
    pid_rate: {
        label: '内环 (角速率)',
        desc: 'Rate PID',
        axes: ['roll', 'pitch', 'yaw'],
        gains: ['kp', 'ki', 'kd'],
        defaults: {
            roll_kp: 150.0, roll_ki: 0.0, roll_kd: 1.5,
            pitch_kp: 150.0, pitch_ki: 0.0, pitch_kd: 1.5,
            yaw_kp: 80.0, yaw_ki: 0.0, yaw_kd: 0.0
        },
        ranges: {
            kp: { min: 0, max: 500, step: 1 },
            ki: { min: 0, max: 200, step: 0.5 },
            kd: { min: 0, max: 20, step: 0.1 }
        }
    },
    pid_attitude: {
        label: '外环 (姿态)',
        desc: 'Attitude PID',
        axes: ['roll', 'pitch', 'yaw'],
        gains: ['kp', 'ki', 'kd'],
        defaults: {
            roll_kp: 4.0, roll_ki: 0.0, roll_kd: 0.0,
            pitch_kp: 4.0, pitch_ki: 0.0, pitch_kd: 0.0,
            yaw_kp: 4.0, yaw_ki: 0.0, yaw_kd: 0.0
        },
        ranges: {
            kp: { min: 0, max: 20, step: 0.1 },
            ki: { min: 0, max: 10, step: 0.1 },
            kd: { min: 0, max: 5, step: 0.05 }
        }
    }
};

var AXIS_LABELS = {
    roll:  { name: 'Roll (横滚)', color: '#2c9edd' },
    pitch: { name: 'Pitch (俯仰)', color: '#27d965' },
    yaw:   { name: 'Yaw (航向)',   color: '#d99927' }
};
var GAIN_LABELS = { kp: 'P', ki: 'I', kd: 'D' };

// ========== 状态 ==========
var currentParams = {};
var pendingChanges = {};
var isDroneConnected = false;
var activeGroup = 'pid_rate';
var CHART_MAX_POINTS = 360;
var CHART_SAMPLE_INTERVAL_S = 0.02; // 50Hz
var CHART_TRIM_BATCH = 64;
var CHART_RENDER_FPS = 30;
var CHART_SMOOTH_ALPHA = 0.25;
var CHART_SMOOTH_WINDOW = 3;
var statusPollTimer = null;
var chartDataStore = {};
var chartTimestamps = [];
var chartStartTime = 0;
var chartSampleIndex = 0;
var chartSmoothedValues = {};
var chartRawWindowStore = {};
var chartUPlots = {};
var chartDirty = false;
var chartLatestTelemetry = null;
var chartRafId = 0;
var chartLastRenderTs = 0;
var syncStatusQueued = false;

// ========== 初始化 ==========
function initPidTuner() {
    for (var gn in PID_GROUPS) {
        var g = PID_GROUPS[gn];
        for (var k in g.defaults) {
            currentParams[gn + '.' + k] = g.defaults[k];
        }
    }
    loadFromStorage();
    buildPidUI();
    buildChartToggles();
    buildCharts();
    startChartRenderLoop();

    if (statusPollTimer) clearInterval(statusPollTimer);
    refreshConnection();
    statusPollTimer = setInterval(refreshConnection, 1500);
    console.log('[PID Tuner] v2 init done');
}

function refreshConnection() {
    fetch('/api/status', { cache: 'no-store' })
        .then(function(r) { return r.json(); })
        .then(function(s) {
            var c = Array.isArray(s.clients) && s.clients.length > 0;
            setPidConnected(c);
        })
        .catch(function() {});
}

// ========== 图表通道切换按钮 ==========
function buildChartToggles() {
    var container = document.getElementById('pid-chart-toggles');
    if (!container) return;
    container.innerHTML = '';

    for (var id in CHART_CHANNELS) {
        (function(chId, ch) {
            var btn = document.createElement('button');
            btn.className = 'pid-ch-toggle' + (ch.enabled ? ' active' : '');
            btn.style.setProperty('--ch-color', ch.color);
            btn.textContent = ch.label;
            btn.dataset.channel = chId;
            btn.onclick = function() {
                ch.enabled = !ch.enabled;
                btn.classList.toggle('active', ch.enabled);
                buildCharts();
            };
            container.appendChild(btn);
        })(id, CHART_CHANNELS[id]);
    }
}

// ========== 多通道图表 (uPlot) ==========
function buildCharts() {
    var area = document.getElementById('pid-charts-area');
    if (!area) return;

    for (var id in chartUPlots) {
        if (chartUPlots[id] && chartUPlots[id].plot) chartUPlots[id].plot.destroy();
        if (chartUPlots[id] && chartUPlots[id].ro) chartUPlots[id].ro.disconnect();
    }
    chartUPlots = {};
    area.innerHTML = '';

    var enabledIds = [];
    for (var id in CHART_CHANNELS) {
        if (CHART_CHANNELS[id].enabled) enabledIds.push(id);
    }

    if (enabledIds.length === 0) {
        area.innerHTML = '<div class="pid-chart-empty">点击上方按钮启用图表通道</div>';
        return;
    }

    for (var i = 0; i < enabledIds.length; i++) {
        createUPlotChart(area, enabledIds[i], CHART_CHANNELS[enabledIds[i]]);
    }
}

function createUPlotChart(container, chId, channel) {
    var wrapper = document.createElement('div');
    wrapper.className = 'pid-chart-track';
    container.appendChild(wrapper);

    var seriesOpts = [{}];
    for (var i = 0; i < channel.series.length; i++) {
        seriesOpts.push({
            label: channel.series[i].label,
            stroke: channel.series[i].color,
            width: 1.5,
            points: { show: false }
        });
    }

    requestAnimationFrame(function() {
        var rect = wrapper.getBoundingClientRect();
        var w = Math.max(rect.width || 400, 200);
        var h = Math.max(rect.height || 120, 60);
        var userZoomed = false;

        var opts = {
            width: w,
            height: h,
            title: channel.label + (channel.unit ? ' (' + channel.unit + ')' : ''),
            series: seriesOpts,
            scales: {
                x: { time: false },
                y: {
                    auto: true,
                    range: function(u, dMin, dMax) {
                        if (dMin === dMax) return [dMin - 1, dMax + 1];
                        var pad = (dMax - dMin) * 0.1;
                        return [dMin - pad, dMax + pad];
                    }
                }
            },
            axes: [
                {
                    stroke: '#555',
                    grid: { stroke: '#1a1a1a', width: 1 },
                    ticks: { stroke: '#333' },
                    font: '9px Consolas, monospace',
                    size: 20,
                    values: function(u, vals) {
                        return vals.map(function(v) { return v.toFixed(0) + 's'; });
                    }
                },
                {
                    stroke: '#555',
                    grid: { stroke: '#1a1a1a', width: 1 },
                    ticks: { stroke: '#333' },
                    font: '9px Consolas, monospace',
                    size: 45,
                    values: function(u, vals) {
                        return vals.map(function(v) { return v.toFixed(1); });
                    }
                }
            ],
            cursor: {
                show: false,
                drag: { x: false, y: false, setScale: false },
                sync: { key: 'pid-sync' },
                points: { show: false }
            },
            legend: { show: true, live: false }
        };

        var data = getChannelDataRefs(channel);
        var plot = new uPlot(opts, data, wrapper);

        // 禁用鼠标交互，保持实时滚动更新
        var over = plot.root.querySelector('.u-over');
        if (over) {
            over.style.pointerEvents = 'none';
        }

        // ResizeObserver 自动适配尺寸
        var ro = null;
        if (typeof ResizeObserver !== 'undefined') {
            ro = new ResizeObserver(function(entries) {
                var r = entries[0].contentRect;
                if (r.width > 10 && r.height > 10) {
                    plot.setSize({ width: r.width, height: r.height });
                }
            });
            ro.observe(wrapper);
        }

        chartUPlots[chId] = {
            plot: plot, channel: channel, wrapper: wrapper, ro: ro, dataRef: data,
            isZoomed: function() { return userZoomed; },
            isInteracting: function() { return false; },
            resetZoom: function() { userZoomed = false; }
        };
    });
}

function getSeriesArray(key) {
    if (!chartDataStore[key]) chartDataStore[key] = [];
    return chartDataStore[key];
}

function getChannelDataRefs(channel) {
    var data = [chartTimestamps];
    for (var i = 0; i < channel.series.length; i++) {
        var key = channel.series[i].key;
        data.push(getSeriesArray(key));
    }
    return data;
}

function appendBounded(arr, val) {
    arr.push(val);
    if (arr.length > (CHART_MAX_POINTS + CHART_TRIM_BATCH)) {
        arr.splice(0, arr.length - CHART_MAX_POINTS);
    }
}

function resetChartData() {
    chartDataStore = {};
    chartTimestamps.length = 0;
    chartSmoothedValues = {};
    chartRawWindowStore = {};
    chartLatestTelemetry = null;
    chartSampleIndex = 0;
    chartStartTime = 0;
    chartDirty = true;
}

function consumeTelemetry(telemetry) {
    if (!telemetry) return false;

    if (chartStartTime === 0) chartStartTime = Date.now();
    var t = chartSampleIndex * CHART_SAMPLE_INTERVAL_S;
    chartSampleIndex++;
    appendBounded(chartTimestamps, t);

    var values = {
        roll: telemetry.roll || 0,
        pitch: telemetry.pitch || 0,
        yaw: telemetry.yaw || 0,
        gyroX: telemetry.gyroX || 0,
        gyroY: telemetry.gyroY || 0,
        gyroZ: telemetry.gyroZ || 0,
        accX: telemetry.accX || 0,
        accY: telemetry.accY || 0,
        accZ: telemetry.accZ || 0,
        estAltitude: telemetry.estAltitude || 0,
        baroAltitude: telemetry.baroAltitude || 0,
        tofDistance: telemetry.tofDistance || 0,
        estVelX: telemetry.estVelX || 0,
        estVelY: telemetry.estVelY || 0,
        estVelZ: telemetry.estVelZ || 0,
        motor0: telemetry.motor0 || 0,
        motor1: telemetry.motor1 || 0,
        motor2: telemetry.motor2 || 0,
        motor3: telemetry.motor3 || 0
    };

    if (telemetry.motorOutputs && telemetry.motorOutputs.length === 4) {
        values.motor0 = telemetry.motorOutputs[0];
        values.motor1 = telemetry.motorOutputs[1];
        values.motor2 = telemetry.motorOutputs[2];
        values.motor3 = telemetry.motorOutputs[3];
    }

    for (var key in values) {
        var rawVal = values[key];

        var rawWindow = chartRawWindowStore[key];
        if (!rawWindow) {
            rawWindow = [];
            chartRawWindowStore[key] = rawWindow;
        }
        rawWindow.push(rawVal);
        if (rawWindow.length > CHART_SMOOTH_WINDOW) rawWindow.shift();

        var sum = 0;
        for (var wi = 0; wi < rawWindow.length; wi++) sum += rawWindow[wi];
        var windowAvg = sum / rawWindow.length;

        var prev = chartSmoothedValues[key];
        var smoothVal = (prev === undefined)
            ? windowAvg
            : (prev + CHART_SMOOTH_ALPHA * (windowAvg - prev));
        chartSmoothedValues[key] = smoothVal;

        var arr = getSeriesArray(key);
        appendBounded(arr, smoothVal);
    }

    return true;
}

function updateChartData(telemetry) {
    chartLatestTelemetry = telemetry;
}

function chartRenderTick(ts) {
    chartRafId = requestAnimationFrame(chartRenderTick);

    var frameInterval = 1000 / CHART_RENDER_FPS;
    if ((ts - chartLastRenderTs) < frameInterval) return;
    chartLastRenderTs = ts;

    if (chartLatestTelemetry) {
        if (consumeTelemetry(chartLatestTelemetry)) {
            chartDirty = true;
        }
        chartLatestTelemetry = null;
    }

    if (!chartDirty) return;
    chartDirty = false;

    for (var id in chartUPlots) {
        var entry = chartUPlots[id];
        if (!entry.plot) continue;
        var keepScale = entry.isZoomed() || (entry.isInteracting && entry.isInteracting());
        entry.plot.setData(entry.dataRef, !keepScale);
    }
}

function startChartRenderLoop() {
    if (chartRafId) cancelAnimationFrame(chartRafId);
    chartRafId = 0;
    chartLastRenderTs = 0;
    chartRafId = requestAnimationFrame(chartRenderTick);
}

// ========== PID 参数 UI ==========
function buildPidUI() {
    var container = document.getElementById('pid-params-container');
    if (!container) return;

    var tabBar = document.getElementById('pid-group-tabs');
    if (tabBar) {
        tabBar.innerHTML = '';
        for (var gn in PID_GROUPS) {
            (function(groupName, group) {
                var btn = document.createElement('button');
                btn.className = 'pid-group-tab' + (groupName === activeGroup ? ' active' : '');
                btn.textContent = group.label;
                btn.title = group.desc;
                btn.onclick = function() { switchPidGroup(groupName, this); };
                tabBar.appendChild(btn);
            })(gn, PID_GROUPS[gn]);
        }
    }
    renderParamCards();
}

function renderParamCards() {
    var container = document.getElementById('pid-params-container');
    if (!container) return;
    container.innerHTML = '';

    var group = PID_GROUPS[activeGroup];
    if (!group) return;

    for (var ai = 0; ai < group.axes.length; ai++) {
        var axis = group.axes[ai];
        var axisInfo = AXIS_LABELS[axis];
        var card = document.createElement('div');
        card.className = 'pid-axis-card';
        card.style.borderLeftColor = axisInfo.color;

        var html = '<div class="pid-axis-header">'
            + '<span class="pid-axis-title" style="color:' + axisInfo.color + '">' + axisInfo.name + '</span>'
            + '<button class="pid-copy-btn" onclick="copyAxisParams(\'' + activeGroup + '\',\'' + axis + '\')" title="Roll/Pitch 对称复制">Copy</button>'
            + '</div><div class="pid-gains-row">';

        for (var gi2 = 0; gi2 < group.gains.length; gi2++) {
            var gain = group.gains[gi2];
            var paramKey = activeGroup + '.' + axis + '_' + gain;
            var range = group.ranges[gain];
            var val = currentParams[paramKey] !== undefined ? currentParams[paramKey] : 0;

            html += '<div class="pid-gain-item">'
                + '<label class="pid-gain-label">' + GAIN_LABELS[gain] + '</label>'
                + '<input type="number" class="pid-num-input" id="input-' + paramKey + '"'
                + ' value="' + val + '" min="' + range.min + '" max="' + range.max + '" step="' + range.step + '"'
                + ' onchange="onParamInput(\'' + paramKey + '\', this.value)">'
                + '<input type="range" class="pid-slider" id="slider-' + paramKey + '"'
                + ' value="' + val + '" min="' + range.min + '" max="' + range.max + '" step="' + range.step + '"'
                + ' oninput="onSliderInput(\'' + paramKey + '\', this.value)">'
                + '</div>';
        }
        html += '</div>';
        card.innerHTML = html;
        container.appendChild(card);
    }
}

function switchPidGroup(groupName, btnEl) {
    activeGroup = groupName;
    var tabs = document.querySelectorAll('.pid-group-tab');
    for (var i = 0; i < tabs.length; i++) tabs[i].classList.remove('active');
    if (btnEl) btnEl.classList.add('active');
    renderParamCards();
}

// ========== 参数输入 ==========
function onParamInput(paramKey, rawValue) {
    var val = parseFloat(rawValue);
    if (isNaN(val)) return;
    currentParams[paramKey] = val;
    pendingChanges[paramKey] = val;
    var slider = document.getElementById('slider-' + paramKey);
    if (slider) slider.value = val;
    scheduleSyncStatusUpdate();
}

function onSliderInput(paramKey, rawValue) {
    var val = parseFloat(rawValue);
    if (isNaN(val)) return;
    currentParams[paramKey] = val;
    pendingChanges[paramKey] = val;
    var input = document.getElementById('input-' + paramKey);
    if (input) input.value = val;
    scheduleSyncStatusUpdate();
}

function scheduleSyncStatusUpdate() {
    if (syncStatusQueued) return;
    syncStatusQueued = true;
    requestAnimationFrame(function() {
        syncStatusQueued = false;
        updateSyncStatus();
    });
}

function copyAxisParams(groupName, srcAxis) {
    var dstAxis = srcAxis === 'roll' ? 'pitch' : srcAxis === 'pitch' ? 'roll' : null;
    if (!dstAxis) { pidLog('航向轴不支持对称复制', 'warn'); return; }
    var group = PID_GROUPS[groupName];
    for (var i = 0; i < group.gains.length; i++) {
        var gain = group.gains[i];
        var srcKey = groupName + '.' + srcAxis + '_' + gain;
        var dstKey = groupName + '.' + dstAxis + '_' + gain;
        currentParams[dstKey] = currentParams[srcKey];
        pendingChanges[dstKey] = currentParams[srcKey];
        var inp = document.getElementById('input-' + dstKey);
        var sld = document.getElementById('slider-' + dstKey);
        if (inp) inp.value = currentParams[srcKey];
        if (sld) sld.value = currentParams[srcKey];
    }
    updateSyncStatus();
    pidLog(AXIS_LABELS[srcAxis].name + ' -> ' + AXIS_LABELS[dstAxis].name, 'info');
}

// ========== 同步到飞控 ==========
function syncToFC() {
    var keys = Object.keys(pendingChanges);
    if (keys.length === 0) { pidLog('没有待同步的变更', 'info'); return; }

    var params = keys.map(function(key) {
        var parts = key.split('.');
        return { group: parts[0], name: parts[1], value: pendingChanges[key] };
    });

    pidLog('同步 ' + params.length + ' 个参数...', 'info');
    var syncBtn = document.getElementById('pid-sync-btn');
    if (syncBtn) { syncBtn.disabled = true; syncBtn.textContent = '[...] 同步中...'; }

    fetch('/api/pid/batch', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ params: params })
    })
    .then(function(r) { return r.json(); })
    .then(function(data) {
        if (data.success) {
            var failed = data.results.filter(function(r) { return r.error; });
            if (failed.length > 0) {
                pidLog('[!] ' + failed.length + ' 项失败', 'warn');
            } else {
                pidLog('[OK] 已同步 ' + params.length + ' 个参数', 'success');
            }
            for (var i = 0; i < data.results.length; i++) {
                var r = data.results[i];
                if (r.success) delete pendingChanges[r.group + '.' + r.name];
            }
        } else {
            pidLog('[ERR] ' + data.error, 'error');
        }
        updateSyncStatus();
    })
    .catch(function(err) {
        pidLog('[ERR] ' + err.message, 'error');
        updateSyncStatus();
    });
}

function updateSyncStatus() {
    var count = Object.keys(pendingChanges).length;
    var badge = document.getElementById('pid-pending-count');
    var syncBtn = document.getElementById('pid-sync-btn');
    if (badge) {
        badge.textContent = count;
        badge.style.display = count > 0 ? 'inline-block' : 'none';
    }
    if (syncBtn) {
        syncBtn.disabled = count === 0;
        syncBtn.textContent = count > 0 ? ('Sync (' + count + ')') : 'Synced';
        syncBtn.className = 'pid-action-btn pid-btn-sync' + (count > 0 ? ' has-changes' : '');
    }
}

// ========== 持久化 ==========
function saveToStorage() {
    try {
        localStorage.setItem('espdrone_pid_params', JSON.stringify(currentParams));
        pidLog('[OK] 已保存到本地', 'success');
    } catch (e) { pidLog('保存失败: ' + e.message, 'error'); }
}

function loadFromStorage() {
    try {
        var saved = localStorage.getItem('espdrone_pid_params');
        if (saved) {
            var parsed = JSON.parse(saved);
            for (var k in parsed) currentParams[k] = parsed[k];
        }
    } catch (e) {}
}

function resetToDefaults() {
    if (!confirm('恢复所有 PID 参数到默认值？')) return;
    for (var gn in PID_GROUPS) {
        var g = PID_GROUPS[gn];
        for (var k in g.defaults) {
            var pk = gn + '.' + k;
            currentParams[pk] = g.defaults[k];
            pendingChanges[pk] = g.defaults[k];
        }
    }
    renderParamCards();
    updateSyncStatus();
    pidLog('[!] 已恢复默认值，请同步到飞控', 'info');
}

function exportParams() {
    var blob = new Blob([JSON.stringify({ version: 2, timestamp: new Date().toISOString(), params: currentParams }, null, 2)], { type: 'application/json' });
    var a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'pid_' + new Date().toISOString().slice(0, 10) + '.json';
    a.click();
    URL.revokeObjectURL(a.href);
    pidLog('[OK] 参数已导出', 'success');
}

function importParams() {
    var input = document.createElement('input');
    input.type = 'file'; input.accept = '.json';
    input.onchange = function(e) {
        var file = e.target.files[0];
        if (!file) return;
        var reader = new FileReader();
        reader.onload = function(ev) {
            try {
                var data = JSON.parse(ev.target.result);
                if (data.params) {
                    for (var k in data.params) {
                        currentParams[k] = data.params[k];
                        pendingChanges[k] = data.params[k];
                    }
                    renderParamCards();
                    updateSyncStatus();
                    pidLog('[OK] 导入 ' + Object.keys(data.params).length + ' 项', 'success');
                }
            } catch (err) { pidLog('导入失败: ' + err.message, 'error'); }
        };
        reader.readAsText(file);
    };
    input.click();
}

// ========== 日志 ==========
function pidLog(msg, type) {
    var el = document.getElementById('pid-log');
    if (!el) return;
    var d = document.createElement('div');
    d.className = 'pid-log-entry pid-log-' + (type || 'info');
    d.textContent = '[' + new Date().toLocaleTimeString() + '] ' + msg;
    el.appendChild(d);
    el.scrollTop = el.scrollHeight;
    while (el.children.length > 80) el.removeChild(el.firstChild);
}

// ========== 连接状态 ==========
function setPidConnected(connected) {
    var prev = isDroneConnected;
    isDroneConnected = connected;

    if (prev !== connected) {
        resetChartData();
    }

    var dot = document.getElementById('pid-conn-dot');
    var text = document.getElementById('pid-conn-text');
    if (dot) dot.className = 'indicator-dot ' + (connected ? 'green' : 'red');
    if (text) text.textContent = connected ? '飞控已连接' : '飞控未连接';
}

// ========== 暴露到全局 ==========
window.pidTuner = {
    init: initPidTuner,
    setConnected: setPidConnected,
    updateTelemetry: updateChartData
};

window.syncToFC = syncToFC;
window.saveToStorage = saveToStorage;
window.resetToDefaults = resetToDefaults;
window.exportParams = exportParams;
window.importParams = importParams;
window.switchPidGroup = switchPidGroup;
window.onParamInput = onParamInput;
window.onSliderInput = onSliderInput;
window.copyAxisParams = copyAxisParams;

// 初始化
function tryInitPid() {
    if (window._pidTunerInited) return;
    window._pidTunerInited = true;
    initPidTuner();
}

if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', tryInitPid);
} else {
    tryInitPid();
}

})();
