/**
 * vofa+ 客户端配置管理
 * 通过 API 与服务器通信，配置 FireWater 协议转发
 *
 * 关键概念：
 *   vofa+ 软件中的 "本地端口" = 
 * + 监听数据的端口
 *   我们的服务器会把遥测数据 UDP 发送到该端口
 */

let vofaState = {
    enabled: true,
    clients: [],       // [{ip, port}, ...]
    clientCount: 0,
    statusCheckInterval: null
};

/**
 * 切换 vofa+ 配置面板显示/隐藏
 */
function toggleVofaPanel() {
    const panel = document.getElementById('vofa-panel');
    if (panel.style.display === 'none') {
        panel.style.display = 'block';
        refreshVofaStatus();
        startVofaStatusPolling();
    } else {
        panel.style.display = 'none';
        stopVofaStatusPolling();
    }
}

/**
 * 从服务器刷新 vofa+ 状态
 */
async function refreshVofaStatus() {
    try {
        const resp = await fetch('/api/vofa/status', { cache: 'no-store' });
        if (!resp.ok) return;
        const status = await resp.json();
        vofaState.enabled = status.enabled;
        vofaState.clients = status.clients || [];
        vofaState.clientCount = status.clientCount || 0;
        updateVofaUI();
    } catch (err) {
        console.error('[vofa+] Status fetch failed:', err.message);
    }
}

/**
 * 启用/禁用 vofa+ 转发
 */
async function toggleVofa(enabled) {
    try {
        const resp = await fetch('/api/vofa/enable', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ enabled })
        });
        if (!resp.ok) {
            alert('设置失败');
            updateVofaUI();
            return;
        }
        const result = await resp.json();
        vofaState.enabled = result.enabled;
        updateVofaUI();
    } catch (err) {
        console.error('[vofa+] Toggle failed:', err.message);
        alert('设置失败: ' + err.message);
        updateVofaUI();
    }
}

/**
 * 添加 vofa+ 客户端 (IP + 端口)
 */
async function addVofaClient() {
    const ipInput = document.getElementById('vofa-client-ip');
    const portInput = document.getElementById('vofa-client-port');
    const ip = ipInput.value.trim();
    const port = parseInt(portInput.value.trim()) || 1347;

    if (!ip || !isValidIP(ip)) {
        alert('请输入有效的 IP 地址');
        return;
    }
    if (port < 1 || port > 65535) {
        alert('端口范围: 1-65535');
        return;
    }

    // 检查重复
    if (vofaState.clients.some(c => c.ip === ip && c.port === port)) {
        alert(`${ip}:${port} 已添加`);
        return;
    }

    try {
        const resp = await fetch('/api/vofa/add-client', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ ip, port })
        });
        if (!resp.ok) {
            alert('添加失败');
            return;
        }
        // 直接从服务器刷新完整状态
        await refreshVofaStatus();
        console.log(`[vofa+] Added client: ${ip}:${port}`);
    } catch (err) {
        console.error('[vofa+] Add client failed:', err.message);
        alert('添加失败: ' + err.message);
    }
}

/**
 * 移除 vofa+ 客户端
 */
async function removeVofaClient(key) {
    try {
        const resp = await fetch('/api/vofa/remove-client', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ key })
        });
        if (!resp.ok) {
            alert('移除失败');
            return;
        }
        await refreshVofaStatus();
        console.log(`[vofa+] Removed client: ${key}`);
    } catch (err) {
        console.error('[vofa+] Remove client failed:', err.message);
        alert('移除失败: ' + err.message);
    }
}

/**
 * 更新 vofa+ UI
 */
function updateVofaUI() {
    const checkbox = document.getElementById('vofa-enable');
    if (checkbox) {
        checkbox.checked = vofaState.enabled;
    }

    const statusDiv = document.getElementById('vofa-status');
    if (statusDiv) {
        if (vofaState.enabled) {
            statusDiv.textContent = `✓ 就绪 (${vofaState.clientCount} 个客户端)`;
            statusDiv.className = 'vofa-status';
        } else {
            statusDiv.textContent = '✕ 禁用';
            statusDiv.className = 'vofa-status disabled';
        }
    }

    const clientsList = document.getElementById('vofa-clients');
    if (clientsList) {
        if (vofaState.clients.length === 0) {
            clientsList.innerHTML = '<div style="color: var(--qgc-text-muted); font-size: 0.8rem; text-align: center; padding: 8px;">暂无客户端，请添加 vofa+ 的 IP 和本地端口</div>';
        } else {
            clientsList.innerHTML = vofaState.clients
                .map(c => {
                    const key = `${c.ip}:${c.port}`;
                    return `
                    <div class="vofa-client-item">
                        <span>${key}</span>
                        <button onclick="removeVofaClient('${key}')" class="vofa-btn-remove">移除</button>
                    </div>`;
                })
                .join('');
        }
    }
}

/**
 * 验证 IP 地址格式
 */
function isValidIP(ip) {
    const ipv4Pattern = /^(\d{1,3}\.){3}\d{1,3}$/;
    if (!ipv4Pattern.test(ip)) return false;
    return ip.split('.').every(p => parseInt(p) <= 255);
}

function startVofaStatusPolling() {
    if (vofaState.statusCheckInterval) return;
    vofaState.statusCheckInterval = setInterval(refreshVofaStatus, 3000);
}

function stopVofaStatusPolling() {
    if (vofaState.statusCheckInterval) {
        clearInterval(vofaState.statusCheckInterval);
        vofaState.statusCheckInterval = null;
    }
}

document.addEventListener('DOMContentLoaded', () => {
    refreshVofaStatus();
});
