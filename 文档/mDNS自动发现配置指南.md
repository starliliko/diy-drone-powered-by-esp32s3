# mDNS 自动发现配置指南

## 功能说明

ESP32 现在可以通过 **mDNS (组播DNS)** 自动发现局域网内的服务器，无需手动配置服务器 IP 地址。

### 工作原理

1. **服务器端**：Node.js 服务器启动时自动广播 mDNS 服务 `_esp-drone._tcp.local`
2. **ESP32 端**：连接 WiFi 后，自动查询 mDNS 服务，获取服务器 IP 地址
3. **自动连接**：ESP32 使用发现的 IP 地址连接服务器

### 优势

- ✅ **零配置**：无需在固件中硬编码 IP
- ✅ **动态适应**：服务器 IP 变化后自动发现
- ✅ **局域网即插即用**：只要在同一网络即可工作
- ✅ **降级保护**：mDNS 失败时使用配置的备用 IP

## 配置步骤

### 1. 服务器端配置（已完成）

服务器代码已自动启用 mDNS 广播，无需额外配置。启动服务器时会看到：

```
✓ TCP Server listening on port 8080
✓ HTTP/WebSocket Server listening on port 3000
✓ mDNS Service published: _esp-drone._tcp.local:8080
  ESP32 will auto-discover this server
```

### 2. ESP32 固件配置

#### 方法 A：使用 menuconfig（推荐）

```bash
idf.py menuconfig
```

导航到：
```
Component config → 
  Crazyflie Config → 
    Network → 
      WiFi Configuration → 
        [*] Enable Remote Server Connection
          [*] Use mDNS for Server Discovery
          (esp-drone-server) mDNS Service Hostname
          (192.168.1.100) Remote Server IP Address (Fallback)
```

配置选项说明：
- **Use mDNS for Server Discovery**: 启用自动发现（默认开启）
- **mDNS Service Hostname**: 搜索的服务名称（默认 `esp-drone-server`，需与服务器端一致）
- **Remote Server IP Address (Fallback)**: 备用 IP，mDNS 失败时使用

#### 方法 B：直接修改 sdkconfig

在 `sdkconfig` 文件中添加/修改：

```ini
CONFIG_REMOTE_SERVER_ENABLE=y
CONFIG_REMOTE_SERVER_USE_MDNS=y
CONFIG_REMOTE_SERVER_MDNS_HOSTNAME="esp-drone-server"
CONFIG_REMOTE_SERVER_IP="192.168.1.100"
CONFIG_REMOTE_SERVER_PORT=8080
```

### 3. 编译并烧录固件

```bash
idf.py build flash monitor
```

## 运行验证

### 1. 启动服务器

```bash
cd apptest
npm start
```

预期输出：
```
✓ TCP Server listening on port 8080
✓ HTTP/WebSocket Server listening on port 3000
✓ mDNS Service published: _esp-drone._tcp.local:8080
  ESP32 will auto-discover this server
```

### 2. ESP32 日志输出

ESP32 连接 WiFi 后，会尝试 mDNS 发现：

```
I (5230) REMOTE: Discovering server via mDNS...
I (8450) REMOTE: mDNS: Found server at 192.168.178.231 (hostname: esp-drone-server)
I (8460) REMOTE: Connecting to 192.168.178.231:8080...
I (8520) REMOTE: Connected to server!
```

如果 mDNS 失败，会使用备用 IP：
```
I (5230) REMOTE: Discovering server via mDNS...
W (8230) REMOTE: No mDNS service found
I (8240) REMOTE: Connecting to 192.168.1.100:8080...
```

## 故障排查

### mDNS 发现失败

**可能原因：**

1. **防火墙阻止 mDNS**
   - mDNS 使用 UDP 端口 5353
   - Windows: 检查 Windows Defender 防火墙
   - macOS/Linux: 检查 `iptables` 规则

2. **路由器禁用组播**
   - 部分路由器默认禁用 mDNS/组播
   - 在路由器设置中启用 "IGMP Snooping" 或 "Multicast"

3. **不同子网**
   - mDNS 仅在同一子网内工作
   - 确保 ESP32 和服务器在同一网段（如 192.168.1.x）

4. **服务名称不匹配**
   - 检查固件配置的服务名与服务器端一致
   - 默认都是 `esp-drone-server`

### 测试 mDNS 服务

**Windows（需安装 Bonjour）：**
```cmd
dns-sd -B _esp-drone._tcp
```

**macOS/Linux：**
```bash
avahi-browse -r _esp-drone._tcp
```

**Node.js 测试脚本：**
```javascript
const Bonjour = require('bonjour-service');
const bonjour = new Bonjour();

bonjour.find({ type: 'esp-drone' }, (service) => {
    console.log('Found service:', service);
});
```

## 高级配置

### 修改服务名称

如果需要自定义服务名，需同时修改服务器端和固件端：

**服务器端（app.js）：**
```javascript
this.mdnsService = this.bonjour.publish({
    name: 'My Custom Drone Server',
    type: 'my-drone',  // 改为自定义名称
    port: CONFIG.TCP_PORT,
});
```

**固件端（menuconfig）：**
```
CONFIG_REMOTE_SERVER_MDNS_HOSTNAME="my-drone"
```

### 禁用 mDNS（仅使用固定 IP）

在 menuconfig 中：
```
[ ] Use mDNS for Server Discovery
```

或在 sdkconfig 中：
```ini
CONFIG_REMOTE_SERVER_USE_MDNS=n
```

## 技术细节

### mDNS 协议

- **标准**: RFC 6762 (Multicast DNS)
- **端口**: UDP 5353
- **地址**: 224.0.0.251 (IPv4), ff02::fb (IPv6)
- **查询超时**: 3 秒
- **服务类型**: `_esp-drone._tcp.local`

### ESP-IDF mDNS 组件

- 组件: `esp_mdns`
- API: `mdns_query_ptr()`
- 查询类型: PTR (指针记录)
- 结果类型: IPv4 地址优先

### 性能影响

- **启动延迟**: 首次连接增加 ~3 秒（mDNS 查询时间）
- **内存占用**: +2KB（mDNS 组件）
- **网络流量**: 极小（仅启动时查询一次）

## 参考资料

- [ESP-IDF mDNS 文档](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/mdns.html)
- [Bonjour Service NPM](https://www.npmjs.com/package/bonjour-service)
- [RFC 6762 - Multicast DNS](https://datatracker.ietf.org/doc/html/rfc6762)
