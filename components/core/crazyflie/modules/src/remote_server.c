/**
 * @file remote_server.c
 * @brief 远程服务器通信模块实现
 *
 * 实现 TCP 客户端连接到远程服务器，支持：
 * - 遥测数据上传
 * - 控制指令接收
 * - CRTP 透传
 * - 心跳保活
 * - 断线自动重连
 */

#include "sdkconfig.h"

#ifdef CONFIG_REMOTE_SERVER_ENABLE

#include <string.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "remote_server.h"
#include "wifi_esp32.h"
#include "crtp.h"
#include "commander.h"
#include "crtp_commander.h"
#include "vehicle_state.h"
#include "stabilizer_types.h"
#include "log.h"
#include "param.h"
#include "pm_esplane.h"
#include "stm32_legacy.h"
#include "static_mem.h"

#define DEBUG_MODULE "REMOTE"
#include "debug_cf.h"

/*===========================================================================
 * 辅助宏
 *===========================================================================*/

#ifndef CLAMP
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#endif

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

/*===========================================================================
 * 配置参数
 *===========================================================================*/

#define REMOTE_SERVER_IP CONFIG_REMOTE_SERVER_IP
#define REMOTE_SERVER_PORT CONFIG_REMOTE_SERVER_PORT
#define RECONNECT_INTERVAL_MS (CONFIG_REMOTE_SERVER_RECONNECT_INTERVAL * 1000)
#define HEARTBEAT_INTERVAL_MS CONFIG_REMOTE_SERVER_HEARTBEAT_INTERVAL
#define TELEMETRY_INTERVAL_MS CONFIG_REMOTE_SERVER_TELEMETRY_INTERVAL
#define SOCKET_TIMEOUT_MS 5000

#define TX_QUEUE_SIZE 16
#define RX_BUFFER_SIZE 256

// UDP 广播发现配置
#define DISCOVERY_PORT 8081        // 广播监听端口
#define DISCOVERY_MAGIC 0x45535044 // "ESPD" 魔数
#define DISCOVERY_TIMEOUT_MS 5000  // 发现超时时间

// 任务配置
#define REMOTE_TX_TASK_NAME "remoteTx"
#define REMOTE_TX_TASK_PRI 3
#define REMOTE_TX_TASK_STACK 4096

#define REMOTE_RX_TASK_NAME "remoteRx"
#define REMOTE_RX_TASK_PRI 3
#define REMOTE_RX_TASK_STACK 4096

#define REMOTE_TELEM_TASK_NAME "remoteTelem"
#define REMOTE_TELEM_TASK_PRI 2
#define REMOTE_TELEM_TASK_STACK 3072

/*===========================================================================
 * 状态变量
 *===========================================================================*/

static bool isInit = false;
static int sock = -1;
static volatile RemoteConnectionState connectionState = REMOTE_STATE_DISCONNECTED;
static SemaphoreHandle_t sockMutex = NULL;
static QueueHandle_t txQueue = NULL;
static EventGroupHandle_t eventGroup = NULL;

// 动态服务器地址
static char serverIP[16] = REMOTE_SERVER_IP; // 默认使用配置的IP
static bool serverDiscovered = false;

#define EVT_CONNECTED (1 << 0)
#define EVT_DISCONNECTED (1 << 1)
#define EVT_RECONNECT (1 << 2)

// 统计信息
static uint32_t txPacketCount = 0;
static uint32_t rxPacketCount = 0;
static uint32_t reconnectCount = 0;
static uint32_t connectionStartTime = 0;

// 回调函数
static RemoteStateCallback stateCallback = NULL;
static RemoteControlCallback controlCallback = NULL;

// 序列号
static uint16_t txSeq = 0;

// 发送队列项
typedef struct
{
    uint8_t type;
    uint16_t size;
    uint8_t data[REMOTE_MAX_PAYLOAD_SIZE];
} TxQueueItem;

// 日志变量
static uint8_t logConnected = 0;
static uint16_t logTxRate = 0;
static uint16_t logRxRate = 0;

// 远程控制模式（默认启用，当连接后且无其他控制源时接管）
static volatile RemoteControlMode remoteControlMode = REMOTE_CTRL_MODE_ENABLED;

/*===========================================================================
 * 内部函数声明
 *===========================================================================*/

static void remoteServerRxTask(void *param);
static void remoteServerTxTask(void *param);
static void remoteServerTelemetryTask(void *param);
static bool connectToServer(void);
static void disconnectFromServer(void);
static void processReceivedPacket(const RemotePacketHeader *header, const uint8_t *payload);
static void handleControlPacket(const uint8_t *data, uint16_t size);
static void handleCRTPPacket(const uint8_t *data, uint16_t size);
static bool sendPacketInternal(RemotePacketType type, const uint8_t *data, uint16_t size);
static void updateConnectionState(RemoteConnectionState newState);
static bool validateControlCmd(const RemoteControlCmd *cmd);

/*===========================================================================
 * 控制指令安全限制
 *===========================================================================*/

#define REMOTE_MAX_ROLL_RAW 4500     // ±45度 * 100
#define REMOTE_MAX_PITCH_RAW 4500    // ±45度 * 100
#define REMOTE_MAX_YAW_RATE_RAW 2000 // ±200度/秒 * 10
#define REMOTE_MAX_THRUST 60000      // 留10%安全裕度
#define REMOTE_MIN_THRUST 0

// 统计变量
static uint32_t invalidCmdCount = 0;

/*===========================================================================
 * 公开 API 实现
 *===========================================================================*/

void remoteServerInit(void)
{
    if (isInit)
    {
        return;
    }

    DEBUG_PRINT("Initializing remote server module\n");
    DEBUG_PRINT("Server: %s:%d\n", REMOTE_SERVER_IP, REMOTE_SERVER_PORT);

    // 创建同步原语
    sockMutex = xSemaphoreCreateMutex();
    ASSERT(sockMutex);

    txQueue = xQueueCreate(TX_QUEUE_SIZE, sizeof(TxQueueItem));
    ASSERT(txQueue);

    eventGroup = xEventGroupCreate();
    ASSERT(eventGroup);

    isInit = true;
    DEBUG_PRINT("Remote server module resources created\n");
}

static bool isStarted = false; // 防止多次启动

void remoteServerStart(void)
{
    if (!isInit)
    {
        ESP_LOGE("REMOTE", "Remote server not initialized");
        return;
    }

    if (isStarted)
    {
        ESP_LOGW("REMOTE", "Remote server already started");
        return;
    }
    isStarted = true;

    ESP_LOGI("REMOTE", "Starting remote server tasks");

    // 创建任务
    xTaskCreate(remoteServerRxTask, REMOTE_RX_TASK_NAME, REMOTE_RX_TASK_STACK,
                NULL, REMOTE_RX_TASK_PRI, NULL);
    xTaskCreate(remoteServerTxTask, REMOTE_TX_TASK_NAME, REMOTE_TX_TASK_STACK,
                NULL, REMOTE_TX_TASK_PRI, NULL);
    xTaskCreate(remoteServerTelemetryTask, REMOTE_TELEM_TASK_NAME, REMOTE_TELEM_TASK_STACK,
                NULL, REMOTE_TELEM_TASK_PRI, NULL);

    ESP_LOGI("REMOTE", "Remote server tasks started");
}

bool remoteServerTest(void)
{
    return isInit;
}

RemoteConnectionState remoteServerGetState(void)
{
    return connectionState;
}

bool remoteServerIsConnected(void)
{
    return connectionState == REMOTE_STATE_CONNECTED;
}

void remoteServerSetStateCallback(RemoteStateCallback cb)
{
    stateCallback = cb;
}

void remoteServerSetControlCallback(RemoteControlCallback cb)
{
    controlCallback = cb;
}

bool remoteServerSendCRTP(const uint8_t *data, uint32_t size)
{
    if (!isInit || connectionState != REMOTE_STATE_CONNECTED)
    {
        return false;
    }
    return remoteServerSendPacket(REMOTE_PKT_CRTP, data, (uint16_t)size);
}

bool remoteServerSendPacket(RemotePacketType type, const uint8_t *data, uint16_t size)
{
    if (!isInit || size > REMOTE_MAX_PAYLOAD_SIZE)
    {
        return false;
    }

    TxQueueItem item;
    item.type = type;
    item.size = size;
    if (data && size > 0)
    {
        memcpy(item.data, data, size);
    }

    return xQueueSend(txQueue, &item, M2T(10)) == pdTRUE;
}

void remoteServerReconnect(void)
{
    if (isInit)
    {
        xEventGroupSetBits(eventGroup, EVT_RECONNECT);
    }
}

const char *remoteServerGetIP(void)
{
    return serverIP; // 返回动态发现的IP或配置的IP
}

void remoteServerGetStats(uint32_t *txCount, uint32_t *rxCount,
                          uint32_t *reconnects, uint32_t *uptime)
{
    if (txCount)
        *txCount = txPacketCount;
    if (rxCount)
        *rxCount = rxPacketCount;
    if (reconnects)
        *reconnects = reconnectCount;
    if (uptime)
    {
        if (connectionState == REMOTE_STATE_CONNECTED && connectionStartTime > 0)
        {
            *uptime = (xTaskGetTickCount() - connectionStartTime) / configTICK_RATE_HZ;
        }
        else
        {
            *uptime = 0;
        }
    }
}

/*===========================================================================
 * 连接管理
 *===========================================================================*/

/**
 * @brief 使用 UDP 广播发现服务器 IP 地址
 *
 * 监听服务器发送的广播包，格式：
 * - 4字节魔数 (0x45535044 = "ESPD")
 * - 2字节端口号 (网络字节序)
 *
 * @return true 如果成功发现，false 超时
 */
/**
 * @brief 获取STA接口的网络信息并计算子网广播地址
 */
static uint32_t getSubnetBroadcastAddr(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif == NULL)
    {
        ESP_LOGW("REMOTE", "Failed to get STA netif");
        return INADDR_BROADCAST;
    }

    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK)
    {
        ESP_LOGW("REMOTE", "Failed to get IP info");
        return INADDR_BROADCAST;
    }

    // 子网广播地址 = IP | (~netmask)
    uint32_t ip = ip_info.ip.addr;
    uint32_t mask = ip_info.netmask.addr;
    uint32_t bcast = ip | (~mask);

    ESP_LOGI("REMOTE", "Local IP: " IPSTR ", Mask: " IPSTR ", Broadcast: " IPSTR,
             IP2STR(&ip_info.ip), IP2STR(&ip_info.netmask), IP2STR((esp_ip4_addr_t *)&bcast));

    return bcast;
}

static bool discoverServerViaUDP(void)
{
    ESP_LOGI("REMOTE", "Discovering server via UDP broadcast...");

    int discoverySock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (discoverySock < 0)
    {
        ESP_LOGE("REMOTE", "Failed to create discovery socket: errno=%d", errno);
        return false;
    }

    // 允许发送广播
    int broadcast = 1;
    setsockopt(discoverySock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    // 允许端口复用
    int reuse = 1;
    setsockopt(discoverySock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    // 设置接收超时 500ms
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 500 * 1000;
    setsockopt(discoverySock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // 绑定到任意端口（不绑定8081，避免和服务器冲突）
    struct sockaddr_in bindAddr;
    memset(&bindAddr, 0, sizeof(bindAddr));
    bindAddr.sin_family = AF_INET;
    bindAddr.sin_port = htons(0); // 系统自动分配端口
    bindAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(discoverySock, (struct sockaddr *)&bindAddr, sizeof(bindAddr)) < 0)
    {
        ESP_LOGE("REMOTE", "Failed to bind discovery socket: errno=%d", errno);
        close(discoverySock);
        return false;
    }

    // 获取绑定的端口号
    socklen_t addrLen = sizeof(bindAddr);
    getsockname(discoverySock, (struct sockaddr *)&bindAddr, &addrLen);
    ESP_LOGI("REMOTE", "Discovery socket bound to port %d", ntohs(bindAddr.sin_port));

    // 构造发现请求包（魔数 + TCP端口=0表示请求）
    uint8_t req[6] = {0};
    uint32_t magicNet = htonl(DISCOVERY_MAGIC);
    memcpy(req, &magicNet, 4);
    req[4] = 0;
    req[5] = 0;

    // 获取子网广播地址
    uint32_t subnetBcast = getSubnetBroadcastAddr();

    struct sockaddr_in bcastAddr;
    memset(&bcastAddr, 0, sizeof(bcastAddr));
    bcastAddr.sin_family = AF_INET;
    bcastAddr.sin_port = htons(DISCOVERY_PORT);

    TickType_t startTick = xTaskGetTickCount();
    TickType_t timeoutTicks = M2T(DISCOVERY_TIMEOUT_MS);
    int sendCount = 0;

    // 接收循环
    uint8_t buffer[16];
    struct sockaddr_in srcAddr;
    socklen_t srcLen;

    while ((xTaskGetTickCount() - startTick) < timeoutTicks)
    {
        // 每秒发送一次发现请求
        if (sendCount == 0 || (xTaskGetTickCount() - startTick) >= M2T(sendCount * 1000))
        {
            // 发送到子网广播地址
            bcastAddr.sin_addr.s_addr = subnetBcast;
            int ret = sendto(discoverySock, req, sizeof(req), 0, (struct sockaddr *)&bcastAddr, sizeof(bcastAddr));
            ESP_LOGI("REMOTE", "Sent discovery request to subnet broadcast (ret=%d)", ret);

            // 也发送到全网广播地址
            bcastAddr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
            ret = sendto(discoverySock, req, sizeof(req), 0, (struct sockaddr *)&bcastAddr, sizeof(bcastAddr));
            ESP_LOGI("REMOTE", "Sent discovery request to 255.255.255.255 (ret=%d)", ret);

            sendCount++;
        }

        // 尝试接收回应
        srcLen = sizeof(srcAddr);
        int len = recvfrom(discoverySock, buffer, sizeof(buffer), 0,
                           (struct sockaddr *)&srcAddr, &srcLen);

        if (len < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // 超时，继续循环
                continue;
            }
            ESP_LOGW("REMOTE", "Discovery recv error: errno=%d", errno);
            continue;
        }

        ESP_LOGI("REMOTE", "Received %d bytes from %s:%d", len,
                 inet_ntoa(srcAddr.sin_addr), ntohs(srcAddr.sin_port));

        if (len < 6)
        {
            ESP_LOGW("REMOTE", "Discovery invalid packet (len=%d)", len);
            continue;
        }

        // 验证魔数
        uint32_t magic = (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];
        if (magic != DISCOVERY_MAGIC)
        {
            ESP_LOGW("REMOTE", "Invalid discovery magic: 0x%08lX", (unsigned long)magic);
            continue;
        }

        // 检查TCP端口（非0表示服务器回应）
        uint16_t tcpPort = (buffer[4] << 8) | buffer[5];
        if (tcpPort == 0)
        {
            // 这是另一个ESP32的请求，忽略
            ESP_LOGD("REMOTE", "Ignoring discovery request from %s", inet_ntoa(srcAddr.sin_addr));
            continue;
        }

        // 使用发送方的 IP 地址
        char *ipStr = inet_ntoa(srcAddr.sin_addr);
        strncpy(serverIP, ipStr, sizeof(serverIP) - 1);
        serverIP[sizeof(serverIP) - 1] = '\0';
        serverDiscovered = true;

        ESP_LOGI("REMOTE", "Discovered server at %s:%d (TCP port from packet: %d)",
                 serverIP, REMOTE_SERVER_PORT, tcpPort);
        close(discoverySock);
        return true;
    }

    close(discoverySock);
    ESP_LOGW("REMOTE", "Discovery timeout after %d requests", sendCount);
    return false;
}

static void updateConnectionState(RemoteConnectionState newState)
{
    if (connectionState != newState)
    {
        connectionState = newState;
        logConnected = (newState == REMOTE_STATE_CONNECTED) ? 1 : 0;

        // 更新 VehicleState 中的 GCS 连接状态
        vehicleSetGcsConnected(newState == REMOTE_STATE_CONNECTED);

        DEBUG_PRINT("Connection state: %d\n", newState);

        if (stateCallback)
        {
            stateCallback(newState);
        }
    }
}

static bool connectToServer(void)
{
    struct sockaddr_in destAddr;
    int err;

    updateConnectionState(REMOTE_STATE_CONNECTING);

    // 如果还未发现服务器，尝试通过 UDP 广播发现
    if (!serverDiscovered)
    {
        discoverServerViaUDP();
    }

    // 解析服务器地址（使用动态发现的IP或配置的IP）
    destAddr.sin_addr.s_addr = inet_addr(serverIP);
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(REMOTE_SERVER_PORT);

    // 创建 socket
    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0)
    {
        DEBUG_PRINT("Unable to create socket: errno %d\n", errno);
        updateConnectionState(REMOTE_STATE_ERROR);
        return false;
    }

    // 设置超时
    struct timeval timeout;
    timeout.tv_sec = SOCKET_TIMEOUT_MS / 1000;
    timeout.tv_usec = (SOCKET_TIMEOUT_MS % 1000) * 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    // 连接服务器
    ESP_LOGI("REMOTE", "Connecting to %s:%d...", serverIP, REMOTE_SERVER_PORT);
    err = connect(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (err != 0)
    {
        ESP_LOGE("REMOTE", "Socket connect failed: errno %d", errno);
        close(sock);
        sock = -1;
        updateConnectionState(REMOTE_STATE_DISCONNECTED);
        return false;
    }

    ESP_LOGI("REMOTE", "Connected to server!");
    updateConnectionState(REMOTE_STATE_CONNECTED);
    xEventGroupSetBits(eventGroup, EVT_CONNECTED);
    xEventGroupClearBits(eventGroup, EVT_DISCONNECTED);

    connectionStartTime = xTaskGetTickCount();
    reconnectCount++;

    return true;
}

static void disconnectFromServer(void)
{
    if (sock >= 0)
    {
        close(sock);
        sock = -1;
    }
    updateConnectionState(REMOTE_STATE_DISCONNECTED);
    xEventGroupSetBits(eventGroup, EVT_DISCONNECTED);
    xEventGroupClearBits(eventGroup, EVT_CONNECTED);
    connectionStartTime = 0;
}

/*===========================================================================
 * 发送任务
 *===========================================================================*/

static bool sendPacketInternal(RemotePacketType type, const uint8_t *data, uint16_t size)
{
    if (sock < 0 || connectionState != REMOTE_STATE_CONNECTED)
    {
        return false;
    }

    // 构造包头
    RemotePacketHeader header;
    header.magic[0] = REMOTE_MAGIC_0;
    header.magic[1] = REMOTE_MAGIC_1;
    header.version = REMOTE_PROTOCOL_VERSION;
    header.type = type;
    header.seq = txSeq++;
    header.length = size;

    // 发送（加锁保护）
    bool success = false;
    if (xSemaphoreTake(sockMutex, M2T(100)) == pdTRUE)
    {
        // 先发包头
        int sent = send(sock, &header, sizeof(header), 0);
        if (sent == sizeof(header))
        {
            // 再发数据
            if (size > 0 && data)
            {
                sent = send(sock, data, size, 0);
                success = (sent == size);
            }
            else
            {
                success = true;
            }
        }
        xSemaphoreGive(sockMutex);

        if (success)
        {
            txPacketCount++;
        }
    }

    return success;
}

static void remoteServerTxTask(void *param)
{
    TxQueueItem item;
    TickType_t lastHeartbeat = 0;

    ESP_LOGI("REMOTE", "TX task started, attempting connection...");

    // 等待初始连接
    while (!connectToServer())
    {
        ESP_LOGW("REMOTE", "Connection failed, retrying in %d ms...", RECONNECT_INTERVAL_MS);
        vTaskDelay(M2T(RECONNECT_INTERVAL_MS));
    }

    while (true)
    {
        // 检查是否需要重连
        EventBits_t bits = xEventGroupGetBits(eventGroup);
        if ((bits & EVT_DISCONNECTED) || (bits & EVT_RECONNECT))
        {
            xEventGroupClearBits(eventGroup, EVT_RECONNECT);
            DEBUG_PRINT("Reconnecting...\n");
            disconnectFromServer();
            vTaskDelay(M2T(RECONNECT_INTERVAL_MS));
            connectToServer();
            continue;
        }

        // 从队列取数据发送
        if (xQueueReceive(txQueue, &item, M2T(HEARTBEAT_INTERVAL_MS / 2)) == pdTRUE)
        {
            if (!sendPacketInternal(item.type, item.data, item.size))
            {
                // 发送失败，触发重连
                xEventGroupSetBits(eventGroup, EVT_DISCONNECTED);
            }
        }

        // 定期发送心跳
        TickType_t now = xTaskGetTickCount();
        if ((now - lastHeartbeat) >= M2T(HEARTBEAT_INTERVAL_MS))
        {
            uint32_t timestamp = now;
            if (!sendPacketInternal(REMOTE_PKT_HEARTBEAT, (uint8_t *)&timestamp, sizeof(timestamp)))
            {
                xEventGroupSetBits(eventGroup, EVT_DISCONNECTED);
            }
            lastHeartbeat = now;
        }
    }
}

/*===========================================================================
 * 接收任务
 *===========================================================================*/

static void remoteServerRxTask(void *param)
{
    uint8_t rxBuffer[RX_BUFFER_SIZE];
    int rxLen = 0;
    int parsePos = 0;

    ESP_LOGI("REMOTE", "RX task started, waiting for connection...");

    while (true)
    {
        // 等待连接建立
        xEventGroupWaitBits(eventGroup, EVT_CONNECTED, pdFALSE, pdTRUE, portMAX_DELAY);

        if (sock < 0)
        {
            vTaskDelay(M2T(100));
            continue;
        }

        // 接收数据
        int len = recv(sock, rxBuffer + rxLen, RX_BUFFER_SIZE - rxLen, 0);
        if (len < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // 超时，继续
                continue;
            }
            ESP_LOGE("REMOTE", "recv failed: errno %d", errno);
            xEventGroupSetBits(eventGroup, EVT_DISCONNECTED);
            continue;
        }
        else if (len == 0)
        {
            // 连接关闭
            ESP_LOGW("REMOTE", "Server closed connection");
            xEventGroupSetBits(eventGroup, EVT_DISCONNECTED);
            continue;
        }

        rxLen += len;

        // 解析接收到的数据包
        parsePos = 0;
        while (parsePos + sizeof(RemotePacketHeader) <= rxLen)
        {
            RemotePacketHeader *header = (RemotePacketHeader *)(rxBuffer + parsePos);

            // 验证魔数
            if (header->magic[0] != REMOTE_MAGIC_0 || header->magic[1] != REMOTE_MAGIC_1)
            {
                // 同步丢失，跳过一个字节
                parsePos++;
                continue;
            }

            // 检查是否有完整的包
            int totalLen = sizeof(RemotePacketHeader) + header->length;
            if (parsePos + totalLen > rxLen)
            {
                // 数据不完整，等待更多
                break;
            }

            // 处理包
            uint8_t *payload = rxBuffer + parsePos + sizeof(RemotePacketHeader);
            processReceivedPacket(header, payload);
            rxPacketCount++;

            parsePos += totalLen;
        }

        // 移动剩余数据到缓冲区开头
        if (parsePos > 0 && parsePos < rxLen)
        {
            memmove(rxBuffer, rxBuffer + parsePos, rxLen - parsePos);
            rxLen -= parsePos;
        }
        else if (parsePos >= rxLen)
        {
            rxLen = 0;
        }
    }
}

static void processReceivedPacket(const RemotePacketHeader *header, const uint8_t *payload)
{
    switch (header->type)
    {
    case REMOTE_PKT_HEARTBEAT:
        // 服务器心跳响应，可用于 RTT 计算
        break;

    case REMOTE_PKT_CONTROL:
        handleControlPacket(payload, header->length);
        break;

    case REMOTE_PKT_CRTP:
        handleCRTPPacket(payload, header->length);
        break;

    case REMOTE_PKT_ACK:
        // 确认包
        break;

    case REMOTE_PKT_CONFIG:
        // 配置包，可用于运行时参数调整
        DEBUG_PRINT("Received config packet\n");
        break;

    default:
        DEBUG_PRINT("Unknown packet type: %d\n", header->type);
        break;
    }
}

static void handleControlPacket(const uint8_t *data, uint16_t size)
{
    if (size < sizeof(RemoteControlCmd))
    {
        ESP_LOGW("REMOTE", "Control packet too small: %d", size);
        return;
    }

    const RemoteControlCmd *cmd = (const RemoteControlCmd *)data;

    ESP_LOGI("REMOTE", "CMD: type=0x%02X, mode=%d", cmd->cmdType, cmd->mode);

    // 输入验证
    if (!validateControlCmd(cmd))
    {
        invalidCmdCount++;
        ESP_LOGW("REMOTE", "Invalid control command rejected (total: %lu)", invalidCmdCount);
        return;
    }

    // 如果注册了回调，调用回调
    if (controlCallback)
    {
        controlCallback(cmd);
    }

    // 根据指令类型处理
    setpoint_t setpoint = {0};

    switch (cmd->cmdType)
    {
    case CTRL_CMD_RPYT:
        setpoint.mode.roll = modeAbs;
        setpoint.mode.pitch = modeAbs;
        setpoint.mode.yaw = modeVelocity;
        // 钳位到安全范围
        setpoint.attitude.roll = ((float)CLAMP(cmd->roll, -REMOTE_MAX_ROLL_RAW, REMOTE_MAX_ROLL_RAW)) / 100.0f;
        setpoint.attitude.pitch = ((float)CLAMP(cmd->pitch, -REMOTE_MAX_PITCH_RAW, REMOTE_MAX_PITCH_RAW)) / 100.0f;
        setpoint.attitudeRate.yaw = ((float)CLAMP(cmd->yaw, -REMOTE_MAX_YAW_RATE_RAW, REMOTE_MAX_YAW_RATE_RAW)) / 10.0f;
        setpoint.thrust = CLAMP(cmd->thrust, REMOTE_MIN_THRUST, REMOTE_MAX_THRUST);
        // 使用新的REMOTE优先级（高于普通CRTP）
        commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_REMOTE);
        break;

    case CTRL_CMD_EMERGENCY:
        // 紧急停机 - 使用新的vehicle_state系统
        vehicleEmergencyStop();
        // 同时发送停止命令
        setpoint.thrust = 0;
        setpoint.mode.roll = modeDisable;
        setpoint.mode.pitch = modeDisable;
        setpoint.mode.yaw = modeDisable;
        commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX);
        DEBUG_PRINT("EMERGENCY STOP received!\n");
        break;

    case CTRL_CMD_HOVER:
        // 悬停模式
        setpoint.mode.x = modeVelocity;
        setpoint.mode.y = modeVelocity;
        setpoint.mode.z = modeAbs;
        setpoint.velocity.x = 0;
        setpoint.velocity.y = 0;
        // 高度限制在合理范围内 (0.1m - 3m)
        {
            float targetHeight = cmd->thrust / 1000.0f;
            targetHeight = (targetHeight < 0.1f) ? 0.1f : ((targetHeight > 3.0f) ? 3.0f : targetHeight);
            setpoint.position.z = targetHeight;
        }
        commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_REMOTE);
        break;

    case CTRL_CMD_LAND:
        // 降落模式 - 设置飞行模式并发送降落命令
        vehicleSetFlightMode(FLIGHT_MODE_LAND);
        setpoint.mode.z = modeVelocity;
        setpoint.velocity.z = -0.3f; // 下降速度 0.3m/s
        commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_REMOTE);
        DEBUG_PRINT("LAND command received\n");
        break;

    case CTRL_CMD_ARM:
        // 解锁 - 使用新的vehicle_state系统
        if (vehicleArm(false))
        {
            DEBUG_PRINT("ARM command: SUCCESS\n");
        }
        else
        {
            DEBUG_PRINT("ARM command: FAILED - %s\n", vehicleGetArmFailReasonStr());
        }
        break;

    case CTRL_CMD_DISARM:
        // 上锁 - 使用新的vehicle_state系统
        vehicleDisarm(false);
        // 同时发送停止命令
        setpoint.thrust = 0;
        setpoint.mode.roll = modeDisable;
        setpoint.mode.pitch = modeDisable;
        setpoint.mode.yaw = modeDisable;
        commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_REMOTE);
        DEBUG_PRINT("DISARM command received\n");
        break;

    case CTRL_CMD_SET_CONTROL_MODE:
        // 设置远程控制模式 - mode字段存储控制模式
        if (cmd->mode <= REMOTE_CTRL_MODE_SHARED)
        {
            remoteControlMode = (RemoteControlMode)cmd->mode;
            ESP_LOGI("REMOTE", "Control mode set to: %d", remoteControlMode);
        }
        else
        {
            ESP_LOGW("REMOTE", "Invalid control mode: %d", cmd->mode);
        }
        break;

    default:
        DEBUG_PRINT("Unknown control command type: %d\n", cmd->cmdType);
        break;
    }
}

/**
 * @brief 验证控制指令是否在安全范围内
 */
static bool validateControlCmd(const RemoteControlCmd *cmd)
{
    if (cmd == NULL)
    {
        return false;
    }

    // 检查指令类型是否有效
    // CTRL_CMD_SET_CONTROL_MODE = 0x10, 其他命令在 0x00-0x07 范围
    if (cmd->cmdType > CTRL_CMD_DISARM && cmd->cmdType != CTRL_CMD_SET_CONTROL_MODE)
    {
        return false;
    }

    // 对于RPYT模式，检查范围（允许一定裕度）
    if (cmd->cmdType == CTRL_CMD_RPYT)
    {
        // 超出范围200%视为无效（可能是数据损坏）
        if (abs(cmd->roll) > REMOTE_MAX_ROLL_RAW * 2 ||
            abs(cmd->pitch) > REMOTE_MAX_PITCH_RAW * 2 ||
            abs(cmd->yaw) > REMOTE_MAX_YAW_RATE_RAW * 2)
        {
            return false;
        }

        // 推力超出最大值150%视为无效
        if (cmd->thrust > 65535)
        {
            return false;
        }
    }

    return true;
}

static void handleCRTPPacket(const uint8_t *data, uint16_t size)
{
    if (size < 1 || size > CRTP_MAX_DATA_SIZE + 1)
    {
        ESP_LOGW("REMOTE", "CRTP packet invalid size=%d", size);
        return;
    }

    // 将 CRTP 包发送到本地 CRTP 处理系统
    CRTPPacket p = {0};
    p.size = size - 1;
    memcpy(p.raw, data, size);

    ESP_LOGI("REMOTE", "CRTP RX: port=%d ch=%d size=%d", p.port, p.channel, p.size);

    if (!crtpInjectPacket(&p))
    {
        ESP_LOGW("REMOTE", "Dropped CRTP packet port=%d ch=%d", p.port, p.channel);
    }
}

/*===========================================================================
 * 遥测上报任务
 *===========================================================================*/

// 使用 log 系统获取状态数据
#include "log.h"

static void remoteServerTelemetryTask(void *param)
{
    RemoteTelemetryData telemetry;

    // 获取日志变量ID（只需要获取一次）
    static bool logIdsInit = false;
    static logVarId_t rollId, pitchId, yawId;
    static logVarId_t gyroXId, gyroYId, gyroZId;
    static logVarId_t accXId, accYId, accZId;
    static logVarId_t motorM1Id, motorM2Id, motorM3Id, motorM4Id;
    // V3.2 新增：高度和速度日志变量ID
    static logVarId_t estZId, estVxId, estVyId, estVzId;
    static logVarId_t baroHeightId, tofDistanceId;
    static uint32_t lastMotorPrintTick = 0;

    // 等待一段时间让日志系统初始化
    vTaskDelay(M2T(2000));

    while (true)
    {
        vTaskDelay(M2T(TELEMETRY_INTERVAL_MS));

        if (connectionState != REMOTE_STATE_CONNECTED)
        {
            continue;
        }

        // 初始化日志变量ID
        if (!logIdsInit)
        {
            rollId = logGetVarId("stabilizer", "roll");
            pitchId = logGetVarId("stabilizer", "pitch");
            yawId = logGetVarId("stabilizer", "yaw");

            gyroXId = logGetVarId("gyro", "x");
            gyroYId = logGetVarId("gyro", "y");
            gyroZId = logGetVarId("gyro", "z");

            accXId = logGetVarId("acc", "x");
            accYId = logGetVarId("acc", "y");
            accZId = logGetVarId("acc", "z");

            // 获取电机输出日志ID
            // 优先使用power_distribution的motor组（正常飞行输出），电机测试时motors组值会覆盖
            motorM1Id = logGetVarId("motor", "m1");
            motorM2Id = logGetVarId("motor", "m2");
            motorM3Id = logGetVarId("motor", "m3");
            motorM4Id = logGetVarId("motor", "m4");

            // 若motor组不存在则回退到motors组（电机测试模块）
            if (!LOG_VARID_IS_VALID(motorM1Id))
                motorM1Id = logGetVarId("motors", "m1");
            if (!LOG_VARID_IS_VALID(motorM2Id))
                motorM2Id = logGetVarId("motors", "m2");
            if (!LOG_VARID_IS_VALID(motorM3Id))
                motorM3Id = logGetVarId("motors", "m3");
            if (!LOG_VARID_IS_VALID(motorM4Id))
                motorM4Id = logGetVarId("motors", "m4");

            // V3.2 新增：获取高度和速度日志ID
            // 使用stateEstimate组（导出的最终状态），而非kalman内部状态
            estZId = logGetVarId("stateEstimate", "z");
            estVxId = logGetVarId("stateEstimate", "vx");
            estVyId = logGetVarId("stateEstimate", "vy");
            estVzId = logGetVarId("stateEstimate", "vz");
            // 校准后的气压高度 (kalman_baro组)
            baroHeightId = logGetVarId("kalman_baro", "height");
            // ToF距离 (mtf01组)
            tofDistanceId = logGetVarId("mtf01", "distance");

            logIdsInit = true;
            DEBUG_PRINT("Telemetry log IDs initialized\n");
            DEBUG_PRINT("  estZ=%d, estVx=%d, estVy=%d, estVz=%d, baro=%d, tof=%d\n",
                        estZId, estVxId, estVyId, estVzId, baroHeightId, tofDistanceId);
        }

        // 填充遥测数据
        // 姿态角（从日志获取，单位：度）
        telemetry.roll = (int16_t)(logGetFloat(rollId) * 100);
        telemetry.pitch = (int16_t)(logGetFloat(pitchId) * 100);
        telemetry.yaw = (int16_t)(logGetFloat(yawId) * 100);

        // 角速度（从陀螺仪获取，单位：deg/s）
        telemetry.gyroX = (int16_t)(logGetFloat(gyroXId) * 10);
        telemetry.gyroY = (int16_t)(logGetFloat(gyroYId) * 10);
        telemetry.gyroZ = (int16_t)(logGetFloat(gyroZId) * 10);

        // 加速度（单位：g，转换为 mg）
        telemetry.accX = (int16_t)(logGetFloat(accXId) * 1000);
        telemetry.accY = (int16_t)(logGetFloat(accYId) * 1000);
        telemetry.accZ = (int16_t)(logGetFloat(accZId) * 1000);

        // 电池状态（暂时固定为100%）
        telemetry.battVoltage = 4200; // 4.2V
        telemetry.battPercent = 100;

        // 飞行状态（从新的vehicle_state系统获取 - PX4风格）
        VehicleState vstate = vehicleStateGet();
        telemetry.armingState = (uint8_t)vstate.armingState;
        telemetry.flightMode = (uint8_t)vstate.flightMode;
        telemetry.flightPhase = (uint8_t)vstate.flightPhase;
        telemetry.failsafeState = (uint8_t)vstate.failsafeState;

        // 状态标志位
        telemetry.statusFlags = 0;
        if (vstate.isArmed)
            telemetry.statusFlags |= STATUS_FLAG_ARMED;
        if (vstate.isFlying)
            telemetry.statusFlags |= STATUS_FLAG_FLYING;
        if (vstate.isEmergency)
            telemetry.statusFlags |= STATUS_FLAG_EMERGENCY;
        if (vstate.isRcConnected)
            telemetry.statusFlags |= STATUS_FLAG_RC_CONNECTED;
        if (vstate.isGcsConnected)
            telemetry.statusFlags |= STATUS_FLAG_GCS_CONNECTED;
        if (vstate.isBatteryLow)
            telemetry.statusFlags |= STATUS_FLAG_BATTERY_LOW;

        // 飞行时间
        telemetry.flightTime = vstate.flightTime;

        // 时间戳
        telemetry.timestamp = xTaskGetTickCount();

        // === V3 新增字段 ===
        // 实际飞行姿态模式（由传感器能力决定）
        FlightMode actualMode = getFlightMode();
        telemetry.actualFlightMode = (uint8_t)actualMode;

        // 控制来源逻辑：
        // 1. 遥控器连接时：控制源=遥控器（最高优先级），远程可切换为协同模式
        // 2. 无遥控器 + 远程控制启用 + 已连接：控制源=地面站
        // 3. 无遥控器 + 远程控制禁用/未连接：控制源=无控制
        // 4. 协同模式：遥控器有输入时显示遥控器，否则显示地面站
        int activePriority = commanderGetActivePriority();
        bool rcConnected = vstate.isRcConnected;
        bool gcsConnected = (connectionState == REMOTE_STATE_CONNECTED);

        if (activePriority == COMMANDER_PRIORITY_EXTRX)
        {
            // 遥控器正在控制（最高优先级）
            telemetry.controlSource = (uint8_t)COMMANDER_PRIORITY_EXTRX;
        }
        else if (rcConnected && remoteControlMode == REMOTE_CTRL_MODE_SHARED && gcsConnected)
        {
            // 协同模式：遥控器连接但没有输入，地面站可以控制
            if (activePriority == COMMANDER_PRIORITY_REMOTE)
            {
                telemetry.controlSource = (uint8_t)COMMANDER_PRIORITY_REMOTE;
            }
            else
            {
                // 协同模式下无活动控制，显示遥控器（遥控器始终优先）
                telemetry.controlSource = (uint8_t)COMMANDER_PRIORITY_EXTRX;
            }
        }
        else if (!rcConnected && gcsConnected && remoteControlMode == REMOTE_CTRL_MODE_ENABLED)
        {
            // 无遥控器 + 远程控制启用 + 已连接：控制源=地面站
            telemetry.controlSource = (uint8_t)COMMANDER_PRIORITY_REMOTE;
        }
        else if (activePriority != COMMANDER_PRIORITY_DISABLE)
        {
            // 其他活动控制源
            telemetry.controlSource = (uint8_t)activePriority;
        }
        else
        {
            // 无活动控制源
            telemetry.controlSource = (uint8_t)COMMANDER_PRIORITY_DISABLE;
        }

        // 远程控制模式
        telemetry.remoteCtrlMode = (uint8_t)remoteControlMode;

        // === 电机输出 (V3.1新增) ===
        if (LOG_VARID_IS_VALID(motorM1Id))
        {
            unsigned int m1 = logGetUint(motorM1Id);
            telemetry.motorPower[0] = (uint16_t)(m1 > 65535 ? 65535 : m1);
        }
        else
        {
            telemetry.motorPower[0] = 0;
        }

        if (LOG_VARID_IS_VALID(motorM2Id))
        {
            unsigned int m2 = logGetUint(motorM2Id);
            telemetry.motorPower[1] = (uint16_t)(m2 > 65535 ? 65535 : m2);
        }
        else
        {
            telemetry.motorPower[1] = 0;
        }

        if (LOG_VARID_IS_VALID(motorM3Id))
        {
            unsigned int m3 = logGetUint(motorM3Id);
            telemetry.motorPower[2] = (uint16_t)(m3 > 65535 ? 65535 : m3);
        }
        else
        {
            telemetry.motorPower[2] = 0;
        }

        if (LOG_VARID_IS_VALID(motorM4Id))
        {
            unsigned int m4 = logGetUint(motorM4Id);
            telemetry.motorPower[3] = (uint16_t)(m4 > 65535 ? 65535 : m4);
        }
        else
        {
            telemetry.motorPower[3] = 0;
        }

        // 串口输出电机实时数据（1Hz）
        if ((xTaskGetTickCount() - lastMotorPrintTick) > M2T(1000))
        {
            lastMotorPrintTick = xTaskGetTickCount();
            DEBUG_PRINT("MOTOR_OUT: M1=%u M2=%u M3=%u M4=%u\n",
                        telemetry.motorPower[0], telemetry.motorPower[1],
                        telemetry.motorPower[2], telemetry.motorPower[3]);
        }

        // === V3.2 新增：高度和速度数据 ===
        // 估计器高度 (kalman stateZ, 单位: m -> mm)
        if (LOG_VARID_IS_VALID(estZId))
        {
            telemetry.estAltitude = (int32_t)(logGetFloat(estZId) * 1000.0f);
        }
        else
        {
            telemetry.estAltitude = 0;
        }

        // 校准后的气压计高度 (单位: m -> mm)
        if (LOG_VARID_IS_VALID(baroHeightId))
        {
            telemetry.baroAltitude = (int32_t)(logGetFloat(baroHeightId) * 1000.0f);
        }
        else
        {
            telemetry.baroAltitude = 0;
        }

        // ToF距离 (mtf01 distance, 单位: mm)
        if (LOG_VARID_IS_VALID(tofDistanceId))
        {
            telemetry.tofDistance = (int32_t)logGetUint(tofDistanceId);
        }
        else
        {
            telemetry.tofDistance = 0;
        }

        // 估计器速度X (kalman statePX, 单位: m/s -> mm/s)
        if (LOG_VARID_IS_VALID(estVxId))
        {
            telemetry.estVelX = (int16_t)(logGetFloat(estVxId) * 1000.0f);
        }
        else
        {
            telemetry.estVelX = 0;
        }

        // 估计器速度Y (kalman statePY, 单位: m/s -> mm/s)
        if (LOG_VARID_IS_VALID(estVyId))
        {
            telemetry.estVelY = (int16_t)(logGetFloat(estVyId) * 1000.0f);
        }
        else
        {
            telemetry.estVelY = 0;
        }

        // 估计器速度Z (世界坐标系垂直速度, 单位: m/s -> mm/s)
        if (LOG_VARID_IS_VALID(estVzId))
        {
            telemetry.estVelZ = (int16_t)(logGetFloat(estVzId) * 1000.0f);
        }
        else
        {
            telemetry.estVelZ = 0;
        }

        // 发送遥测数据
        remoteServerSendPacket(REMOTE_PKT_TELEMETRY,
                               (uint8_t *)&telemetry, sizeof(telemetry));
    }
}

/*===========================================================================
 * 控制模式管理 API
 *===========================================================================*/

RemoteControlMode remoteServerGetControlMode(void)
{
    return remoteControlMode;
}

void remoteServerSetControlMode(RemoteControlMode mode)
{
    if (mode <= REMOTE_CTRL_MODE_SHARED)
    {
        remoteControlMode = mode;
        DEBUG_PRINT("Remote control mode set to: %d\n", mode);
    }
}

/*===========================================================================
 * 日志和参数
 *===========================================================================*/

LOG_GROUP_START(remote)
LOG_ADD(LOG_UINT8, connected, &logConnected)
LOG_ADD(LOG_UINT16, txRate, &logTxRate)
LOG_ADD(LOG_UINT16, rxRate, &logRxRate)
LOG_ADD(LOG_UINT32, invalidCmd, &invalidCmdCount)
LOG_GROUP_STOP(remote)

#endif /* CONFIG_REMOTE_SERVER_ENABLE */
