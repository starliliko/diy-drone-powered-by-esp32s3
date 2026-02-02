/**
 * ESP-Drone WebSocket 通信模块
 * ==========================
 * 处理与服务器的WebSocket连接
 */

let ws = null;
let onTelemetryCallback = null;
let onConnectionChangeCallback = null;

export function setTelemetryCallback(callback) {
    onTelemetryCallback = callback;
}

export function setConnectionChangeCallback(callback) {
    onConnectionChangeCallback = callback;
}

export function connectWS() {
    ws = new WebSocket(`ws://${window.location.host}`);

    ws.onopen = () => {
        log('已连接至地面站服务器', 'success');
        document.getElementById('comm-dot').className = 'indicator-dot green';
        document.getElementById('comm-status').textContent = '已连接';
        // 通知连接状态改变
        if (onConnectionChangeCallback) {
            onConnectionChangeCallback(true);
        }
    };

    ws.onclose = () => {
        log('与服务器断开连接', 'error');
        document.getElementById('comm-dot').className = 'indicator-dot red';
        document.getElementById('comm-status').textContent = '未连接';
        // 通知连接状态改变
        if (onConnectionChangeCallback) {
            onConnectionChangeCallback(false);
        }
        setTimeout(connectWS, 3000);
    };

    ws.onmessage = (evt) => {
        try {
            const msg = JSON.parse(evt.data);
            if (msg.type === 'telemetry' && onTelemetryCallback) {
                onTelemetryCallback(msg.data);
            }
        } catch (e) {
            console.error('Parse error:', e);
        }
    };
}

export function log(msg, type = 'info') {
    const el = document.getElementById('console-logs');
    if (!el) return;

    const entry = document.createElement('div');
    entry.className = `log-entry log-${type}`;
    entry.textContent = `[${new Date().toLocaleTimeString()}] ${msg}`;
    el.appendChild(entry);
    el.scrollTop = el.scrollHeight;
}
