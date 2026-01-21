# 🚁 ESP32 飞控连接电脑完整指南

## 📌 核心原理

ESP32 和电脑**不是直接连接**，而是通过 **Wi-Fi 路由器**作为中转：

```
┌─────────────┐                ┌─────────────┐                ┌─────────────┐
│   ESP32     │   Wi-Fi        │   路由器     │   Wi-Fi/网线   │   你的电脑   │
│   飞控      │ ─────────────> │  (中转站)   │ <───────────── │  (服务器)   │
└─────────────┘                └─────────────┘                └─────────────┘
    ↓                                ↓                              ↓
通过 Wi-Fi                       分配 IP 地址                   运行服务器
连接到路由器                     给 ESP32 和电脑                监听 8080 端口
```

**连接流程：**
1. ✅ ESP32 连接到你的 Wi-Fi 路由器
2. ✅ 电脑也连接到同一个 Wi-Fi 路由器
3. ✅ 电脑运行服务器程序（监听端口 8080）
4. ✅ ESP32 通过路由器找到电脑的 IP，发送数据到 8080 端口

---

## 🔧 详细配置步骤

### 步骤 1️⃣：获取你的电脑 IP 地址

**在电脑上打开 PowerShell，运行：**

```powershell
ipconfig
```

**找到你正在使用的网络适配器：**
- 如果用 **Wi-Fi**：找 "无线局域网适配器 WLAN" 或 "WLAN"
- 如果用 **网线**：找 "以太网适配器 以太网"

**记下 IPv4 地址**，例如：
```
IPv4 地址 . . . . . . . . . . . . : 192.168.1.100
```

**✅ 这个 `192.168.1.100` 就是你要用的 IP 地址！**

---

### 步骤 2️⃣：配置 ESP32 飞控

#### 方法 A：通过 menuconfig 配置（推荐）

1. **打开配置界面：**
   ```powershell
   idf.py menuconfig
   ```

2. **进入配置菜单：**
   ```
   diy-drone config
     → wireless config
       → STA Mode & Remote Server
   ```

3. **配置 Wi-Fi 连接：**
   ```
   [*] Enable STA+AP Mode                     <-- 勾选（空格键）
       Router SSID to connect (MyWiFi)        <-- 输入你的 Wi-Fi 名称
       Router Password (********)             <-- 输入 Wi-Fi 密码
       STA connection retry count (10)        <-- 保持默认
   ```

4. **配置远程服务器：**
   ```
   [*] Enable Remote Server Connection        <-- 勾选
       Remote Server IP Address (192.168.1.100)  <-- 填你的电脑 IP
       Remote Server Port (8080)              <-- 保持 8080
       Reconnect interval (5)                 <-- 保持默认
       Heartbeat interval (1000)              <-- 保持默认
       Telemetry upload interval (100)        <-- 保持默认
   ```

5. **保存并退出：**
   - 按 `Q` 键退出
   - 选择 `Yes` 保存

---

#### 方法 B：直接查找并修改代码

如果 menuconfig 找不到选项，可以直接修改源代码。

**查找配置文件：**
```powershell
# 在项目根目录搜索
grep -r "REMOTE_SERVER" components/ main/
```

**可能的位置：**
- `components/drivers/general/wifi/remote_server.c`
- `components/core/crazyflie/hal/src/wifi.c`

**修改示例：**
```c
// Wi-Fi 配置
#define WIFI_SSID "你的WiFi名称"
#define WIFI_PASSWORD "你的WiFi密码"

// 远程服务器配置
#define REMOTE_SERVER_ENABLE 1
#define REMOTE_SERVER_IP "192.168.1.100"   // 你的电脑 IP
#define REMOTE_SERVER_PORT 8080
```

---

### 步骤 3️⃣：编译并烧录飞控

```powershell
# 编译
idf.py build

# 烧录（COM3 改成你的端口号）
idf.py -p COM3 flash

# 查看日志
idf.py -p COM3 monitor
```

**烧录成功后，在监视器中应该看到：**
```
I (xxxx) wifi: connected to MyWiFi
I (xxxx) wifi: got ip: 192.168.1.xxx
I (xxxx) remote_server: Connecting to 192.168.1.100:8080...
I (xxxx) remote_server: Connected to remote server
```

---

### 步骤 4️⃣：启动电脑端服务器

**在 `apptest` 目录运行：**
```powershell
cd "D:\UAV\esp drone\diy-drone\apptest"
npm start
```

**服务器启动后会显示：**
```
✓ TCP Server listening on port 8080
✓ HTTP/WebSocket Server listening on port 3000
  Open http://localhost:3000 in your browser
```

---

### 步骤 5️⃣：验证连接

1. **打开浏览器：** http://localhost:3000

2. **检查连接状态：**
   - 左上角圆点应该是 **绿色**（WebSocket 已连接）
   - "飞控连接" 显示 **1**

3. **观看实时数据：**
   - 姿态角（Roll/Pitch/Yaw）在变化
   - 电池电量显示
   - 传感器数据更新

4. **服务器终端会显示：**
   ```
   [192.168.1.xxx:xxxx] Drone connected
   [192.168.1.xxx:xxxx] Telemetry: Roll=0.5° Pitch=-1.2° Yaw=45.3° ...
   ```

---

## ❓ 常见问题排查

### ❌ ESP32 无法连接到 Wi-Fi

**检查项：**
1. Wi-Fi 名称和密码是否正确（注意大小写）
2. 路由器是否是 **2.4GHz** 频段（ESP32 不支持 5GHz）
3. 路由器是否开启了 MAC 地址过滤

**解决方法：**
- 在 `idf.py monitor` 中查看错误信息
- 确认路由器设置允许新设备连接

---

### ❌ ESP32 连上 Wi-Fi 但无法连接服务器

**检查项：**
1. 电脑 IP 地址是否正确
2. 电脑防火墙是否阻止了 8080 端口
3. ESP32 和电脑是否在同一网络

**解决方法：**

**关闭防火墙测试：**
```powershell
# 查看防火墙状态
Get-NetFirewallProfile | Select-Object Name, Enabled

# 临时允许 8080 端口
New-NetFirewallRule -DisplayName "ESP-Drone Server" -Direction Inbound -LocalPort 8080 -Protocol TCP -Action Allow
```

**Ping 测试连通性：**
```powershell
# 在电脑上 ping ESP32
ping 192.168.1.xxx  # ESP32 的 IP（从 monitor 中看到）
```

---

### ❌ 服务器显示连接但无数据

**可能原因：**
- 飞控未启动（电池未连接）
- 遥测数据发送被禁用

**解决方法：**
- 检查飞控电源
- 确认 `Telemetry upload interval` 不为 0
- 查看 ESP32 monitor 日志

---

## 📊 网络拓扑示例

### 家庭网络（常见）

```
路由器: 192.168.1.1
    ├── 电脑 (Wi-Fi):   192.168.1.100  <-- 在这里运行服务器
    ├── 手机:           192.168.1.50
    └── ESP32 飞控:     192.168.1.150  <-- 自动分配
```

**ESP32 配置：** `REMOTE_SERVER_IP = "192.168.1.100"`

---

### 公司/学校网络

```
路由器: 10.2.1.1
    ├── 电脑 (Wi-Fi):   10.2.207.137  <-- 在这里运行服务器
    └── ESP32 飞控:     10.2.207.200  <-- 自动分配
```

**ESP32 配置：** `REMOTE_SERVER_IP = "10.2.207.137"`

---

## 🎯 快速检查清单

在开始之前，确认以下事项：

- [ ] **路由器正常工作**，其他设备能连接
- [ ] **电脑已连接到路由器**（Wi-Fi 或网线）
- [ ] **获取了电脑的局域网 IP**（不是 127.0.0.1）
- [ ] **ESP32 配置了正确的 Wi-Fi 和服务器 IP**
- [ ] **电脑服务器已启动**（`npm start`）
- [ ] **电脑防火墙允许 8080 端口**
- [ ] **ESP32 已烧录最新固件**

全部打勾？那就准备起飞吧！🚀

---

## 🔗 相关文档

- [远程服务器通信配置指南](../文档/远程服务器通信配置指南.md) - 详细协议说明
- [README.md](README.md) - 服务器 API 文档
- [通信架构改进记录](../文档/通信架构改进记录.md) - 通信系统设计

---

## 💡 高级提示

### 使用固定 IP（避免 IP 变化）

如果每次重启电脑 IP 都变，可以在路由器中设置 **DHCP 保留** 或在电脑上配置**静态 IP**。

### 远程访问（外网）

如果需要在外网访问，需要：
1. 路由器配置端口转发（8080 → 电脑内网 IP）
2. 使用动态域名服务（DDNS）
3. 修改 ESP32 配置为公网 IP 或域名

### 多飞控支持

服务器自动支持多个飞控同时连接，每个飞控显示独立状态。
