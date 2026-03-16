/**
 * ESP-Drone WebSocket 通信模块
 * ==========================
 * 处理与服务器的WebSocket连接
 */

let ws = null;
let onTelemetryCallback = null;
let onConnectionChangeCallback = null;
let reconnectTimer = null;
let heartbeatTimer = null;
let reconnectAttempts = 0;
let lastSeenAt = 0;

const HEARTBEAT_INTERVAL = 2000;
const HEARTBEAT_TIMEOUT = 10000;
const RECONNECT_BASE_DELAY = 500;
const RECONNECT_MAX_DELAY = 10000;

export function setTelemetryCallback(callback) {
    onTelemetryCallback = callback;
}

export function setConnectionChangeCallback(callback) {
    onConnectionChangeCallback = callback;
}

export function connectWS() {
    clearTimeout(reconnectTimer);

    ws = new WebSocket(`ws://${window.location.host}`);

    ws.onopen = () => {
        reconnectAttempts = 0;
        lastSeenAt = Date.now();

        log('已连接至地面站服务器', 'success');
        document.getElementById('comm-dot').className = 'indicator-dot green';
        document.getElementById('comm-status').textContent = '已连接';

        clearInterval(heartbeatTimer);
        heartbeatTimer = setInterval(() => {
            if (!ws || ws.readyState !== WebSocket.OPEN) {
                return;
            }

            // 应用层心跳（浏览器端无法主动发原生 ping 帧）
            ws.send(JSON.stringify({ type: 'ping', timestamp: Date.now() }));

            if (Date.now() - lastSeenAt > HEARTBEAT_TIMEOUT) {
                log('连接心跳超时，正在重连...', 'warn');
                ws.close();
            }
        }, HEARTBEAT_INTERVAL);

        // 通知连接状态改变
        if (onConnectionChangeCallback) {
            onConnectionChangeCallback(true);
        }
    };

    ws.onclose = () => {
        clearInterval(heartbeatTimer);

        log('与服务器断开连接', 'error');
        document.getElementById('comm-dot').className = 'indicator-dot red';
        document.getElementById('comm-status').textContent = '未连接';

        // 通知连接状态改变
        if (onConnectionChangeCallback) {
            onConnectionChangeCallback(false);
        }

        const backoff = Math.min(
            RECONNECT_MAX_DELAY,
            RECONNECT_BASE_DELAY * Math.pow(2, reconnectAttempts)
        );
        reconnectAttempts += 1;
        reconnectTimer = setTimeout(connectWS, backoff);
    };

    ws.onmessage = (evt) => {
        try {
            const msg = JSON.parse(evt.data);
            lastSeenAt = Date.now();

            if (msg.type === 'pong') {
                return;
            }

            if (msg.type === 'telemetry' && onTelemetryCallback) {
                onTelemetryCallback(msg.data);
            }
        } catch (e) {
            console.error('Parse error:', e);
        }
    };

    ws.onerror = () => {
        // 统一走 close 流程，避免重复状态处理
        try {
            ws.close();
        } catch (_) {
            // ignore
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
