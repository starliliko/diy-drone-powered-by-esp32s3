/**
 * @file remote_server.c
 * @brief 远程服务器通信模块实现
 *
 * 实现 TCP 客户端连接到远程服务器，支持�?
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
#include <stdlib.h>
#include <math.h>

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
#include "vehicle_state.h"
#include "sitaw.h"
#include "stabilizer.h"
#include "stabilizer_types.h"
#include "log.h"
#include "param.h"
#include "pm_esplane.h"
#include "stm32_legacy.h"
#include "static_mem.h"

#define DEBUG_MODULE "REMOTE"
#include "debug_cf.h"

/*===========================================================================
 * 辅助�?
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

// UDP 遥测发送配�?
// 包格�? [0x45 'E'][0x53 'S'][0x50 'P'][0x55 'U'][type=0x01][seq][payload]
#define UDP_TELEM_MAGIC_0 0x45
#define UDP_TELEM_MAGIC_1 0x53
#define UDP_TELEM_MAGIC_2 0x50
#define UDP_TELEM_MAGIC_3 0x55
#define UDP_TELEM_PKT_TELEMETRY 0x01
#define UDP_TELEM_HEADER_SIZE   6   // magic(4) + type(1) + seq(1)

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

// Keep GCS/manual roll command direction aligned with SBUS/CRTP manual control
// conventions. Without this, a positive GCS roll command tilts opposite to the
// physical roll direction used elsewhere in the project.
#define REMOTE_SIGN_ROLL (-1.0f)

/*===========================================================================
 * MAVLink v2 协议
 *===========================================================================*/

#define MAVLINK_STX_V1 0xFE
#define MAVLINK_STX_V2 0xFD

#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_SYS_STATUS 1
#define MAVLINK_MSG_ID_PARAM_SET 23
#define MAVLINK_MSG_ID_ATTITUDE 30
#define MAVLINK_MSG_ID_SERVO_OUTPUT_RAW 36
#define MAVLINK_MSG_ID_LOCAL_POSITION_NED 32
#define MAVLINK_MSG_ID_MANUAL_CONTROL 69
#define MAVLINK_MSG_ID_COMMAND_LONG 76
#define MAVLINK_MSG_ID_COMMAND_ACK 77
#define MAVLINK_MSG_ID_HIGHRES_IMU 105

#define MAVLINK_MSG_ID_HEARTBEAT_CRC 50
#define MAVLINK_MSG_ID_SYS_STATUS_CRC 124
#define MAVLINK_MSG_ID_PARAM_SET_CRC 168
#define MAVLINK_MSG_ID_ATTITUDE_CRC 39
#define MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_CRC 222
#define MAVLINK_MSG_ID_LOCAL_POSITION_NED_CRC 185
#define MAVLINK_MSG_ID_MANUAL_CONTROL_CRC 243
#define MAVLINK_MSG_ID_COMMAND_LONG_CRC 152
#define MAVLINK_MSG_ID_COMMAND_ACK_CRC 143
#define MAVLINK_MSG_ID_HIGHRES_IMU_CRC 93

#define MAV_COMP_ID_AUTOPILOT1 1
#define MAV_TYPE_QUADROTOR 2
#define MAV_AUTOPILOT_GENERIC 0

#define MAV_MODE_FLAG_CUSTOM_MODE_ENABLED 0x01
#define MAV_MODE_FLAG_SAFETY_ARMED 0x80

#define MAV_STATE_UNINIT 0
#define MAV_STATE_STANDBY 3
#define MAV_STATE_ACTIVE 4
#define MAV_STATE_CRITICAL 5

#define MAV_CMD_NAV_LAND 21
#define MAV_CMD_COMPONENT_ARM_DISARM 400

#define MAV_RESULT_ACCEPTED 0
#define MAV_RESULT_UNSUPPORTED 3
#define MAV_RESULT_FAILED 4

typedef struct __attribute__((packed))
{
    uint8_t stx;
    uint8_t len;
    uint8_t incompatFlags;
    uint8_t compatFlags;
    uint8_t seq;
    uint8_t sysId;
    uint8_t compId;
    uint8_t msgid0;
    uint8_t msgid1;
    uint8_t msgid2;
} MavlinkV2Header;

typedef struct __attribute__((packed))
{
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t r;
    uint16_t buttons;
    uint8_t target;
} MavlinkManualControl;

typedef struct __attribute__((packed))
{
    float param1;
    float param2;
    float param3;
    float param4;
    float param5;
    float param6;
    float param7;
    uint16_t command;
    uint8_t targetSystem;
    uint8_t targetComponent;
    uint8_t confirmation;
} MavlinkCommandLong;

typedef struct __attribute__((packed))
{
    float paramValue;
    uint8_t targetSystem;
    uint8_t targetComponent;
    char paramId[16];
    uint8_t paramType;
} MavlinkParamSet;

typedef struct
{
    const char *alias;
    const char *group;
    const char *name;
} MavParamAlias;

/*===========================================================================
 * 状态变�?
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

// UDP 遥测通道
static uint16_t udpTelemetryPort = 0; // 从发现响应读取，0=不可�?
static int      udpSock          = -1;
static uint8_t  udpTxSeq         = 0;
static bool remoteRollSignWarned = false;

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

// 序列�?
static uint16_t txSeq = 0;
static uint8_t mavlinkTxSeq = 0;

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

// 远程控制模式（默认启用，当连接后且无其他控制源时接管�?
static volatile RemoteControlMode remoteControlMode = REMOTE_CTRL_MODE_ENABLED;

static uint8_t mavlinkGroundSysId = 255;
static uint8_t mavlinkGroundCompId = 190;

static const MavParamAlias kMavParamAliases[] = {
    {"pa_r_kp", "pid_attitude", "roll_kp"},
    {"pa_r_ki", "pid_attitude", "roll_ki"},
    {"pa_r_kd", "pid_attitude", "roll_kd"},
    {"pa_p_kp", "pid_attitude", "pitch_kp"},
    {"pa_p_ki", "pid_attitude", "pitch_ki"},
    {"pa_p_kd", "pid_attitude", "pitch_kd"},
    {"pa_y_kp", "pid_attitude", "yaw_kp"},
    {"pa_y_ki", "pid_attitude", "yaw_ki"},
    {"pa_y_kd", "pid_attitude", "yaw_kd"},
    {"pp_th_base", "posCtlPid", "thrustBase"},
    {"pp_th_min", "posCtlPid", "thrustMin"},
    {"pp_rp_lim", "posCtlPid", "rpLimit"},
    {"pp_xy_vmax", "posCtlPid", "xyVelMax"},
    {"pp_z_vmax", "posCtlPid", "zVelMax"},
};

static uint8_t mavlinkGetExtraCrc(uint32_t msgId)
{
    switch (msgId)
    {
    case MAVLINK_MSG_ID_HEARTBEAT:
        return MAVLINK_MSG_ID_HEARTBEAT_CRC;
    case MAVLINK_MSG_ID_SYS_STATUS:
        return MAVLINK_MSG_ID_SYS_STATUS_CRC;
    case MAVLINK_MSG_ID_PARAM_SET:
        return MAVLINK_MSG_ID_PARAM_SET_CRC;
    case MAVLINK_MSG_ID_ATTITUDE:
        return MAVLINK_MSG_ID_ATTITUDE_CRC;
    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
        return MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_CRC;
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        return MAVLINK_MSG_ID_LOCAL_POSITION_NED_CRC;
    case MAVLINK_MSG_ID_MANUAL_CONTROL:
        return MAVLINK_MSG_ID_MANUAL_CONTROL_CRC;
    case MAVLINK_MSG_ID_COMMAND_LONG:
        return MAVLINK_MSG_ID_COMMAND_LONG_CRC;
    case MAVLINK_MSG_ID_COMMAND_ACK:
        return MAVLINK_MSG_ID_COMMAND_ACK_CRC;
    case MAVLINK_MSG_ID_HIGHRES_IMU:
        return MAVLINK_MSG_ID_HIGHRES_IMU_CRC;
    default:
        return 0;
    }
}

static uint16_t mavlinkCrcAccumulate(uint8_t data, uint16_t crc)
{
    uint8_t tmp = data ^ (uint8_t)(crc & 0xFF);
    tmp = (uint8_t)(tmp ^ (tmp << 4));
    return (uint16_t)((crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4));
}

static uint16_t mavlinkCrcCalculate(const uint8_t *buffer, uint16_t len, uint8_t extra)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc = mavlinkCrcAccumulate(buffer[i], crc);
    }
    crc = mavlinkCrcAccumulate(extra, crc);
    return crc;
}

static int remoteServerGetSocketSnapshot(void)
{
    int currentSock = -1;

    if (xSemaphoreTake(sockMutex, M2T(100)) == pdTRUE)
    {
        currentSock = sock;
        xSemaphoreGive(sockMutex);
    }

    return currentSock;
}

static void remoteServerSetSocket(int newSock)
{
    if (xSemaphoreTake(sockMutex, M2T(100)) == pdTRUE)
    {
        sock = newSock;
        xSemaphoreGive(sockMutex);
    }
}

static int remoteServerDetachSocket(void)
{
    int detachedSock = -1;

    if (xSemaphoreTake(sockMutex, M2T(100)) == pdTRUE)
    {
        detachedSock = sock;
        sock = -1;
        xSemaphoreGive(sockMutex);
    }

    return detachedSock;
}

/*===========================================================================
 * 内部函数声明
 *===========================================================================*/

static void remoteServerRxTask(void *param);
static void remoteServerTxTask(void *param);
static void remoteServerTelemetryTask(void *param);
static bool connectToServer(void);
static void disconnectFromServer(void);
static bool sendMavlinkMessage(uint32_t msgId, const void *payload, uint8_t payloadLen);
static bool sendMavlinkCommandAck(uint16_t command, uint8_t result);
static void processMavlinkMessage(uint32_t msgId, const uint8_t *payload, uint8_t payloadLen, uint8_t sysId, uint8_t compId);
static void handleMavlinkManualControl(const uint8_t *payload, uint8_t payloadLen);
static void handleMavlinkCommandLong(const uint8_t *payload, uint8_t payloadLen);
static void handleMavlinkParamSet(const uint8_t *payload, uint8_t payloadLen);
static void handleControlPacket(const uint8_t *data, uint16_t size);
static void handleCRTPPacket(const uint8_t *data, uint16_t size);
static bool sendPacketInternal(RemotePacketType type, const uint8_t *data, uint16_t size);
static bool sendUDPTelemetry(const uint8_t *payload, uint16_t size);
static void updateConnectionState(RemoteConnectionState newState);
static bool validateControlCmd(const RemoteControlCmd *cmd);
static ActualFlightMode mapActualFlightMode(VehicleFlightMode mode)
{
    switch (mode)
    {
    case FLIGHT_MODE_POSITION:
    case FLIGHT_MODE_OFFBOARD:
    case FLIGHT_MODE_MISSION:
    case FLIGHT_MODE_RTL:
        return ACTUAL_MODE_POSHOLD;

    case FLIGHT_MODE_ALTITUDE:
    case FLIGHT_MODE_LAND:
    case FLIGHT_MODE_TAKEOFF:
        return ACTUAL_MODE_ALTHOLD;

    case FLIGHT_MODE_MANUAL:
    case FLIGHT_MODE_STABILIZE:
    default:
        return ACTUAL_MODE_STABILIZE;
    }
}

/*===========================================================================
 * 控制指令安全限制
 *===========================================================================*/

#define REMOTE_MAX_ROLL_RAW 4500     // ±45�?* 100
#define REMOTE_MAX_PITCH_RAW 4500    // ±45�?* 100
#define REMOTE_MAX_YAW_RATE_RAW 2000 // ±200�?�?* 10
#define REMOTE_MAX_THRUST 60000      // �?0%安全裕度
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
 * @brief 使用 UDP 广播发现服务�?IP 地址
 *
 * 监听服务器发送的广播包，格式�?
 * - 4字节魔数 (0x45535044 = "ESPD")
 * - 2字节端口�?(网络字节�?
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

    // 允许发送广�?
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

    // 绑定到任意端口（不绑�?081，避免和服务器冲突）
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

    // 构造发现请求包（魔�?+ TCP端口=0表示请求�?
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
        // 每秒发送一次发现请�?
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
                // 超时，继续循�?
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

        // 读取 UDP 遥测端口�?字节扩展响应�?
        if (len >= 8)
        {
            udpTelemetryPort = (buffer[6] << 8) | buffer[7];
            ESP_LOGI("REMOTE", "UDP telemetry port: %d", udpTelemetryPort);
        }
        else
        {
            udpTelemetryPort = 0;
            ESP_LOGW("REMOTE", "Server response too short (%d bytes), UDP telemetry unavailable", len);
        }

        // 使用发送方�?IP 地址
        char *ipStr = inet_ntoa(srcAddr.sin_addr);
        strncpy(serverIP, ipStr, sizeof(serverIP) - 1);
        serverIP[sizeof(serverIP) - 1] = '\0';
        serverDiscovered = true;

        // 如果服务器支�?UDP 遥测，提前创�?UDP socket
        if (udpTelemetryPort > 0)
        {
            if (udpSock >= 0)
            {
                close(udpSock);
            }
            udpSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (udpSock < 0)
            {
                ESP_LOGE("REMOTE", "Failed to create UDP telemetry socket: errno=%d", errno);
                udpTelemetryPort = 0; // 回退�?TCP
            }
            else
            {
                ESP_LOGI("REMOTE", "UDP telemetry socket created (fd=%d)", udpSock);
            }
        }

        ESP_LOGI("REMOTE", "Discovered server at %s:%d (TCP=%d, UDP=%d)",
                 serverIP, REMOTE_SERVER_PORT, tcpPort, udpTelemetryPort);
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

        // 更新 VehicleState 中的 GCS 连接状�?
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
    int newSock;

    updateConnectionState(REMOTE_STATE_CONNECTING);

    // 如果还未发现服务器，尝试通过 UDP 广播发现
    if (!serverDiscovered)
    {
        if (!discoverServerViaUDP())
        {
            // 发现失败，不回退到硬编码 IP，等待下次重连重�?
            ESP_LOGW("REMOTE", "Server not found via UDP broadcast, will retry later");
            updateConnectionState(REMOTE_STATE_DISCONNECTED);
            return false;
        }
    }

    // 解析服务器地址（使用动态发现的IP�?
    destAddr.sin_addr.s_addr = inet_addr(serverIP);
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(REMOTE_SERVER_PORT);

    // 创建 socket
    newSock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (newSock < 0)
    {
        DEBUG_PRINT("Unable to create socket: errno %d\n", errno);
        updateConnectionState(REMOTE_STATE_ERROR);
        return false;
    }

    // 设置超时
    struct timeval timeout;
    timeout.tv_sec = SOCKET_TIMEOUT_MS / 1000;
    timeout.tv_usec = (SOCKET_TIMEOUT_MS % 1000) * 1000;
    setsockopt(newSock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(newSock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    // 连接服务�?
    ESP_LOGI("REMOTE", "Connecting to %s:%d...", serverIP, REMOTE_SERVER_PORT);
    err = connect(newSock, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (err != 0)
    {
        ESP_LOGE("REMOTE", "Socket connect failed: errno %d", errno);
        close(newSock);
        updateConnectionState(REMOTE_STATE_DISCONNECTED);
        return false;
    }

    remoteServerSetSocket(newSock);

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
    int tcpSock = remoteServerDetachSocket();

    if (tcpSock >= 0)
    {
        shutdown(tcpSock, SHUT_RDWR);
        close(tcpSock);
    }
    if (udpSock >= 0)
    {
        close(udpSock);
        udpSock = -1;
    }
    udpTelemetryPort = 0;
    // 重置发现状态，下次重连时重新通过 UDP 广播发现服务�?
    serverDiscovered = false;
    updateConnectionState(REMOTE_STATE_DISCONNECTED);
    xEventGroupSetBits(eventGroup, EVT_DISCONNECTED);
    xEventGroupClearBits(eventGroup, EVT_CONNECTED);
    connectionStartTime = 0;
}

/*===========================================================================
 * 发送任�?
 *===========================================================================*/

static bool sendMavlinkMessage(uint32_t msgId, const void *payload, uint8_t payloadLen)
{
    if (connectionState != REMOTE_STATE_CONNECTED)
    {
        return false;
    }

    uint8_t extra = mavlinkGetExtraCrc(msgId);
    if (extra == 0)
    {
        return false;
    }

    uint8_t packet[10 + 255 + 2];
    const uint16_t packetLen = (uint16_t)(10 + payloadLen + 2);

    packet[0] = MAVLINK_STX_V2;
    packet[1] = payloadLen;
    packet[2] = 0;
    packet[3] = 0;
    packet[4] = mavlinkTxSeq++;
    packet[5] = 1;
    packet[6] = MAV_COMP_ID_AUTOPILOT1;
    packet[7] = (uint8_t)(msgId & 0xFF);
    packet[8] = (uint8_t)((msgId >> 8) & 0xFF);
    packet[9] = (uint8_t)((msgId >> 16) & 0xFF);

    if (payloadLen > 0 && payload)
    {
        memcpy(&packet[10], payload, payloadLen);
    }

    uint16_t crc = mavlinkCrcCalculate(&packet[1], (uint16_t)(9 + payloadLen), extra);
    packet[10 + payloadLen] = (uint8_t)(crc & 0xFF);
    packet[10 + payloadLen + 1] = (uint8_t)((crc >> 8) & 0xFF);

    bool success = false;
    if (xSemaphoreTake(sockMutex, M2T(100)) == pdTRUE)
    {
        int currentSock = sock;
        if (currentSock >= 0)
        {
            int sent = send(currentSock, packet, packetLen, 0);
            success = (sent == packetLen);
            if (success)
            {
                txPacketCount++;
            }
        }
        xSemaphoreGive(sockMutex);
    }

    return success;
}

static bool sendMavlinkCommandAck(uint16_t command, uint8_t result)
{
    uint8_t payload[3];
    payload[0] = (uint8_t)(command & 0xFF);
    payload[1] = (uint8_t)((command >> 8) & 0xFF);
    payload[2] = result;
    return sendMavlinkMessage(MAVLINK_MSG_ID_COMMAND_ACK, payload, sizeof(payload));
}

static paramVarId_t resolveMavlinkParamVarId(const char *paramId)
{
    paramVarId_t invalid = {.id = 0xffffu, .ptr = 0xffffu};

    if (!paramId || !paramId[0])
    {
        return invalid;
    }

    const char *dot = strchr(paramId, '.');
    if (dot)
    {
        const size_t groupLen = (size_t)(dot - paramId);
        const size_t nameLen = strlen(dot + 1);
        if (groupLen == 0 || nameLen == 0 || groupLen >= 32 || nameLen >= 32)
        {
            return invalid;
        }

        char group[32] = {0};
        char name[32] = {0};
        memcpy(group, paramId, groupLen);
        memcpy(name, dot + 1, nameLen);
        return paramGetVarId(group, name);
    }

    for (size_t i = 0; i < (sizeof(kMavParamAliases) / sizeof(kMavParamAliases[0])); i++)
    {
        if (strcmp(paramId, kMavParamAliases[i].alias) == 0)
        {
            return paramGetVarId((char *)kMavParamAliases[i].group, (char *)kMavParamAliases[i].name);
        }
    }

    return invalid;
}

static void handleMavlinkParamSet(const uint8_t *payload, uint8_t payloadLen)
{
    if (payloadLen < sizeof(MavlinkParamSet))
    {
        return;
    }

    const MavlinkParamSet *ps = (const MavlinkParamSet *)payload;

    if ((ps->targetSystem != 0) && (ps->targetSystem != 1))
    {
        return;
    }

    if ((ps->targetComponent != 0) && (ps->targetComponent != MAV_COMP_ID_AUTOPILOT1))
    {
        return;
    }

    char paramId[17] = {0};
    memcpy(paramId, ps->paramId, 16);

    paramVarId_t varid = resolveMavlinkParamVarId(paramId);
    if (!PARAM_VARID_IS_VALID(varid))
    {
        ESP_LOGW("REMOTE", "Unknown MAVLink param id: '%s'", paramId);
        return;
    }

    const int paramType = paramGetType(varid);
    if (paramType & PARAM_RONLY)
    {
        ESP_LOGW("REMOTE", "Reject write to readonly param id='%s'", paramId);
        return;
    }

    const int rawType = (paramType & (~PARAM_RONLY));
    if (rawType == PARAM_FLOAT)
    {
        paramSetFloat(varid, ps->paramValue);
    }
    else
    {
        paramSetInt(varid, (int)lrintf(ps->paramValue));
    }
}

static void handleMavlinkManualControl(const uint8_t *payload, uint8_t payloadLen)
{
    if (payloadLen < sizeof(MavlinkManualControl))
    {
        return;
    }

    const MavlinkManualControl *mc = (const MavlinkManualControl *)payload;
    setpoint_t setpoint = {0};

    setpoint.mode.roll = modeAbs;
    setpoint.mode.pitch = modeAbs;
    setpoint.mode.yaw = modeVelocity;

    if (!remoteRollSignWarned)
    {
        DEBUG_PRINT("WARNING: remote roll command uses sign compensation to match SBUS/CRTP manual roll convention\n");
        remoteRollSignWarned = true;
    }

    setpoint.attitude.roll = REMOTE_SIGN_ROLL * ((float)CLAMP(mc->y, -1000, 1000) / 100.0f);
    setpoint.attitude.pitch = ((float)CLAMP(mc->x, -1000, 1000) / 100.0f);
    setpoint.attitudeRate.yaw = ((float)CLAMP(mc->r, -1000, 1000) / 10.0f);

    uint16_t z = (uint16_t)CLAMP(mc->z, 0, 1000);
    setpoint.thrust = (uint16_t)((z * 65535UL) / 1000UL);

    commanderApplyFlightMode(&setpoint);
    commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_REMOTE);
}

static void handleMavlinkCommandLong(const uint8_t *payload, uint8_t payloadLen)
{
    if (payloadLen < sizeof(MavlinkCommandLong))
    {
        return;
    }

    const MavlinkCommandLong *cmd = (const MavlinkCommandLong *)payload;

    switch (cmd->command)
    {
    case MAV_CMD_COMPONENT_ARM_DISARM:
    {
        if (cmd->param1 > 0.5f)
        {
            if (vehicleArm(false))
            {
                sendMavlinkCommandAck(MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_ACCEPTED);
            }
            else
            {
                sendMavlinkCommandAck(MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_FAILED);
            }
        }
        else
        {
            const bool force = (cmd->param2 > 10000.0f);
            if (force)
            {
                vehicleEmergencyStop();
            }
            else
            {
                vehicleDisarm(false);
            }

            setpoint_t stopSetpoint = {0};
            stopSetpoint.thrust = 0;
            stopSetpoint.mode.roll = modeDisable;
            stopSetpoint.mode.pitch = modeDisable;
            stopSetpoint.mode.yaw = modeDisable;
            commanderSetSetpoint(&stopSetpoint, COMMANDER_PRIORITY_REMOTE);

            sendMavlinkCommandAck(MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_ACCEPTED);
        }
        break;
    }

    case MAV_CMD_NAV_LAND:
    {
        vehicleSetFlightMode(FLIGHT_MODE_LAND);
        setpoint_t setpoint = {0};
        setpoint.mode.z = modeVelocity;
        setpoint.velocity.z = -0.3f;
        commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_REMOTE);
        sendMavlinkCommandAck(MAV_CMD_NAV_LAND, MAV_RESULT_ACCEPTED);
        break;
    }

    default:
        sendMavlinkCommandAck(cmd->command, MAV_RESULT_UNSUPPORTED);
        break;
    }
}

static void processMavlinkMessage(uint32_t msgId, const uint8_t *payload, uint8_t payloadLen, uint8_t sysId, uint8_t compId)
{
    mavlinkGroundSysId = sysId;
    mavlinkGroundCompId = compId;

    switch (msgId)
    {
    case MAVLINK_MSG_ID_MANUAL_CONTROL:
        handleMavlinkManualControl(payload, payloadLen);
        break;

    case MAVLINK_MSG_ID_COMMAND_LONG:
        handleMavlinkCommandLong(payload, payloadLen);
        break;

    case MAVLINK_MSG_ID_PARAM_SET:
        handleMavlinkParamSet(payload, payloadLen);
        break;

    case MAVLINK_MSG_ID_HEARTBEAT:
        break;

    default:
        break;
    }
}

static bool sendPacketInternal(RemotePacketType type, const uint8_t *data, uint16_t size)
{
    if (connectionState != REMOTE_STATE_CONNECTED)
    {
        return false;
    }

    // 构造包�?
    RemotePacketHeader header;
    header.magic[0] = REMOTE_MAGIC_0;
    header.magic[1] = REMOTE_MAGIC_1;
    header.version = REMOTE_PROTOCOL_VERSION;
    header.type = type;
    header.seq = txSeq++;
    header.length = size;

    // 发送（加锁保护�?
    bool success = false;
    if (xSemaphoreTake(sockMutex, M2T(100)) == pdTRUE)
    {
        int currentSock = sock;

        if (currentSock < 0)
        {
            xSemaphoreGive(sockMutex);
            return false;
        }

        // 先发包头
        int sent = send(currentSock, &header, sizeof(header), 0);
        if (sent == sizeof(header))
        {
            // 再发数据
            if (size > 0 && data)
            {
                sent = send(currentSock, data, size, 0);
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

/**
 * @brief 通过 UDP 发送遥测数据（高频路径，不占用 TCP 带宽�?
 * 包格�? [4字节魔数 ESPU] + [1字节类型] + [1字节序号] + [payload]
 */
static bool sendUDPTelemetry(const uint8_t *payload, uint16_t size)
{
    if (udpSock < 0 || udpTelemetryPort == 0 || !serverDiscovered)
    {
        return false;
    }

    // 构�?UDP 包头 (6字节)
    uint8_t header[UDP_TELEM_HEADER_SIZE];
    header[0] = UDP_TELEM_MAGIC_0;
    header[1] = UDP_TELEM_MAGIC_1;
    header[2] = UDP_TELEM_MAGIC_2;
    header[3] = UDP_TELEM_MAGIC_3;
    header[4] = UDP_TELEM_PKT_TELEMETRY;
    header[5] = udpTxSeq++;

    // 组装完整�?
    uint16_t totalSize = UDP_TELEM_HEADER_SIZE + size;
    uint8_t *buf = (uint8_t *)malloc(totalSize);
    if (!buf)
    {
        return false;
    }
    memcpy(buf, header, UDP_TELEM_HEADER_SIZE);
    memcpy(buf + UDP_TELEM_HEADER_SIZE, payload, size);

    struct sockaddr_in destAddr;
    destAddr.sin_family = AF_INET;
    destAddr.sin_port   = htons(udpTelemetryPort);
    destAddr.sin_addr.s_addr = inet_addr(serverIP);

    int ret = sendto(udpSock, buf, totalSize, 0,
                     (struct sockaddr *)&destAddr, sizeof(destAddr));
    free(buf);

    return ret == totalSize;
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
        // 检查是否需要重�?
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

        // 从队列取数据发�?
        if (xQueueReceive(txQueue, &item, M2T(HEARTBEAT_INTERVAL_MS / 2)) == pdTRUE)
        {
            if (!sendPacketInternal(item.type, item.data, item.size))
            {
                // 发送失败，触发重连
                xEventGroupSetBits(eventGroup, EVT_DISCONNECTED);
            }
        }

        // 定期发送心�?
        TickType_t now = xTaskGetTickCount();
        if ((now - lastHeartbeat) >= M2T(HEARTBEAT_INTERVAL_MS))
        {
            uint8_t hb[9] = {0};
            VehicleState vstate = vehicleStateGet();
            uint8_t baseMode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            if (vstate.isArmed)
            {
                baseMode |= MAV_MODE_FLAG_SAFETY_ARMED;
            }
            hb[4] = MAV_TYPE_QUADROTOR;
            hb[5] = MAV_AUTOPILOT_GENERIC;
            hb[6] = baseMode;
            hb[7] = vstate.isEmergency ? MAV_STATE_CRITICAL : (vstate.isArmed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);
            hb[8] = 3;

            if (!sendMavlinkMessage(MAVLINK_MSG_ID_HEARTBEAT, hb, sizeof(hb)))
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

        int currentSock = remoteServerGetSocketSnapshot();

        if (currentSock < 0)
        {
            vTaskDelay(M2T(100));
            continue;
        }

        // 接收数据
        int len = recv(currentSock, rxBuffer + rxLen, RX_BUFFER_SIZE - rxLen, 0);
        if (len < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // 超时，继�?
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

        // 解析接收到的 MAVLink 数据
        parsePos = 0;
        while (parsePos < rxLen)
        {
            uint8_t stx = rxBuffer[parsePos];
            bool isV2 = (stx == MAVLINK_STX_V2);
            bool isV1 = (stx == MAVLINK_STX_V1);

            if (!isV2 && !isV1)
            {
                parsePos++;
                continue;
            }

            int headerLen = isV2 ? 10 : 6;
            if (parsePos + headerLen > rxLen)
            {
                break;
            }

            uint8_t payloadLen = rxBuffer[parsePos + 1];
            uint8_t incompatFlags = isV2 ? rxBuffer[parsePos + 2] : 0;
            bool hasSignature = isV2 && ((incompatFlags & 0x01) != 0);
            int signatureLen = hasSignature ? 13 : 0;
            int frameLen = headerLen + payloadLen + 2 + signatureLen;

            if (parsePos + frameLen > rxLen)
            {
                break;
            }

            uint32_t msgId = 0;
            uint8_t sysId = 0;
            uint8_t compId = 0;
            uint8_t *payload = NULL;
            uint16_t computed = 0;
            uint16_t received = 0;
            uint8_t extra = 0;

            if (isV2)
            {
                msgId = (uint32_t)rxBuffer[parsePos + 7] |
                        ((uint32_t)rxBuffer[parsePos + 8] << 8) |
                        ((uint32_t)rxBuffer[parsePos + 9] << 16);
                sysId = rxBuffer[parsePos + 5];
                compId = rxBuffer[parsePos + 6];
                payload = &rxBuffer[parsePos + 10];
                extra = mavlinkGetExtraCrc(msgId);
                computed = mavlinkCrcCalculate(&rxBuffer[parsePos + 1], (uint16_t)(9 + payloadLen), extra);
            }
            else
            {
                msgId = rxBuffer[parsePos + 5];
                sysId = rxBuffer[parsePos + 3];
                compId = rxBuffer[parsePos + 4];
                payload = &rxBuffer[parsePos + 6];
                extra = mavlinkGetExtraCrc(msgId);
                computed = mavlinkCrcCalculate(&rxBuffer[parsePos + 1], (uint16_t)(5 + payloadLen), extra);
            }

            if (extra == 0)
            {
                parsePos += frameLen;
                continue;
            }

            received = (uint16_t)payload[payloadLen] | ((uint16_t)payload[payloadLen + 1] << 8);
            if (computed != received)
            {
                parsePos++;
                continue;
            }

            processMavlinkMessage(msgId, payload, payloadLen, sysId, compId);
            rxPacketCount++;
            parsePos += frameLen;
        }

        // 移动剩余数据到缓冲区开�?
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
        // 钳位到安全范�?
        if (!remoteRollSignWarned)
        {
            DEBUG_PRINT("WARNING: remote roll command uses sign compensation to match SBUS/CRTP manual roll convention\n");
            remoteRollSignWarned = true;
        }
        setpoint.attitude.roll = REMOTE_SIGN_ROLL *
                                 (((float)CLAMP(cmd->roll, -REMOTE_MAX_ROLL_RAW, REMOTE_MAX_ROLL_RAW)) / 100.0f);
        setpoint.attitude.pitch = ((float)CLAMP(cmd->pitch, -REMOTE_MAX_PITCH_RAW, REMOTE_MAX_PITCH_RAW)) / 100.0f;
        setpoint.attitudeRate.yaw = ((float)CLAMP(cmd->yaw, -REMOTE_MAX_YAW_RATE_RAW, REMOTE_MAX_YAW_RATE_RAW)) / 10.0f;
        setpoint.thrust = CLAMP(cmd->thrust, REMOTE_MIN_THRUST, REMOTE_MAX_THRUST);
        // 根据飞行模式转换 setpoint 语义
        commanderApplyFlightMode(&setpoint);
        commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_REMOTE);
        break;

    case CTRL_CMD_EMERGENCY:
        // 紧急停�?- 使用新的vehicle_state系统
        vehicleEmergencyStop();
        // 同时发送停止命�?
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
        // 降落模式 - 设置飞行模式并发送降落命�?
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
        // 同时发送停止命�?
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

    // 检查指令类型是否有�?
    // CTRL_CMD_SET_CONTROL_MODE = 0x10, 其他命令�?0x00-0x07 范围
    if (cmd->cmdType > CTRL_CMD_DISARM && cmd->cmdType != CTRL_CMD_SET_CONTROL_MODE)
    {
        return false;
    }

    // 对于RPYT模式，检查范围（允许一定裕度）
    if (cmd->cmdType == CTRL_CMD_RPYT)
    {
        // 超出范围200%视为无效（可能是数据损坏�?
        if (abs(cmd->roll) > REMOTE_MAX_ROLL_RAW * 2 ||
            abs(cmd->pitch) > REMOTE_MAX_PITCH_RAW * 2 ||
            abs(cmd->yaw) > REMOTE_MAX_YAW_RATE_RAW * 2)
        {
            return false;
        }

        // 推力超出最大�?50%视为无效
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

    // �?CRTP 包发送到本地 CRTP 处理系统
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

// 使用 log 系统获取状态数�?
#include "log.h"

static void remoteServerTelemetryTask(void *param)
{
    RemoteTelemetryData telemetry;
    TickType_t lastWakeTime;

    // 获取日志变量ID（只需要获取一次）
    static bool logIdsInit = false;
    static logVarId_t rollId, pitchId, yawId;
    static logVarId_t gyroXId, gyroYId, gyroZId;
    static logVarId_t accXId, accYId, accZId;
    static logVarId_t motorM1Id, motorM2Id, motorM3Id, motorM4Id;
    // V3.2 新增：高度和速度日志变量ID
    static logVarId_t estZId, estVxId, estVyId, estVzId;
    static logVarId_t baroHeightId, tofDistanceId;
    // V3.3 新增：机体坐标系速度
    static logVarId_t bodyVxId, bodyVyId;
    // V3.4 新增：磁力计
    static logVarId_t magXId, magYId, magZId, magHeadingId;
    static uint32_t lastMotorPrintTick = 0;

    // 等待一段时间让日志系统初始�?
    vTaskDelay(M2T(2000));
    lastWakeTime = xTaskGetTickCount();

    while (true)
    {
        vTaskDelayUntil(&lastWakeTime, M2T(TELEMETRY_INTERVAL_MS));

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

            // 若motor组不存在则回退到motors组（电机测试模块�?
            if (!LOG_VARID_IS_VALID(motorM1Id))
                motorM1Id = logGetVarId("motors", "m1");
            if (!LOG_VARID_IS_VALID(motorM2Id))
                motorM2Id = logGetVarId("motors", "m2");
            if (!LOG_VARID_IS_VALID(motorM3Id))
                motorM3Id = logGetVarId("motors", "m3");
            if (!LOG_VARID_IS_VALID(motorM4Id))
                motorM4Id = logGetVarId("motors", "m4");

            // V3.2 新增：获取高度和速度日志ID
            // 使用stateEstimate组（导出的最终状态），而非kalman内部状�?
            estZId = logGetVarId("stateEstimate", "z");
            estVxId = logGetVarId("stateEstimate", "vx");
            estVyId = logGetVarId("stateEstimate", "vy");
            estVzId = logGetVarId("stateEstimate", "vz");
            // 校准后的气压高度 (kalman_baro�?
            baroHeightId = logGetVarId("kalman_baro", "height");
            // ToF距离 (mtf01�?
            tofDistanceId = logGetVarId("mtf01", "distance");

            // V3.3 新增：机体坐标系速度
            // 从kalman滤波器内部状态获�?(单位: m/s)
            bodyVxId = logGetVarId("kalman", "statePX");
            bodyVyId = logGetVarId("kalman", "statePY");

            // V3.4 新增：磁力计数据
            magXId = logGetVarId("mag", "x");
            magYId = logGetVarId("mag", "y");
            magZId = logGetVarId("mag", "z");
            magHeadingId = logGetVarId("mag", "heading");

            logIdsInit = true;
            DEBUG_PRINT("Telemetry log IDs initialized\n");
            DEBUG_PRINT("  estZ=%d, estVx=%d, estVy=%d, estVz=%d, baro=%d, tof=%d\n",
                        estZId, estVxId, estVyId, estVzId, baroHeightId, tofDistanceId);
            DEBUG_PRINT("  bodyVx=%d, bodyVy=%d (valid: %d, %d)\n",
                        bodyVxId, bodyVyId, LOG_VARID_IS_VALID(bodyVxId), LOG_VARID_IS_VALID(bodyVyId));
            DEBUG_PRINT("WARNING: telemetry.gyroY uses pitch sign compensation to match telemetry.pitch\n");
        }

        // 填充遥测数据
        // 姿态角（从日志获取，单位：度）
        telemetry.roll = (int16_t)(logGetFloat(rollId) * 100);
        telemetry.pitch = (int16_t)(logGetFloat(pitchId) * 100);
        telemetry.yaw = (int16_t)(logGetFloat(yawId) * 100);

        // 角速度（从陀螺仪获取，单位：deg/s�?
        telemetry.gyroX = (int16_t)(logGetFloat(gyroXId) * 10);
        // Export pitch rate in the same sign convention as telemetry.pitch and PID pitch control.
        telemetry.gyroY = (int16_t)(-logGetFloat(gyroYId) * 10);
        telemetry.gyroZ = (int16_t)(logGetFloat(gyroZId) * 10);

        // 加速度（单位：g，转换为 mg�?
        telemetry.accX = (int16_t)(logGetFloat(accXId) * 1000);
        telemetry.accY = (int16_t)(logGetFloat(accYId) * 1000);
        telemetry.accZ = (int16_t)(logGetFloat(accZId) * 1000);

        // 电池状态（暂时固定�?00%�?
        telemetry.battVoltage = 4200; // 4.2V
        telemetry.battPercent = 100;

        // 飞行状态（从新的vehicle_state系统获取 - PX4风格�?
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
#ifdef SITAW_TU_ENABLED
        if (sitAwTuDetected())
            telemetry.statusFlags |= STATUS_FLAG_TUMBLED;
#endif
        if (stabilizerIsArmThrottleBlocked())
            telemetry.statusFlags |= STATUS_FLAG_ARM_THROTTLE_BLOCK;

        // 飞行时间
        telemetry.flightTime = vstate.flightTime;

        // 时间�?
        telemetry.timestamp = xTaskGetTickCount();

        // === V3 新增字段 ===
        // 实际飞行姿态模式（由传感器能力决定�?
        VehicleFlightMode actualMode = vehicleGetFlightMode();
        telemetry.actualFlightMode = (uint8_t)mapActualFlightMode(actualMode);

        // 控制来源逻辑�?
        // 1. 遥控器连接时：控制源=遥控器（最高优先级），远程可切换为协同模式
        // 2. 无遥控器 + 远程控制启用 + 已连接：控制�?地面�?
        // 3. 无遥控器 + 远程控制禁用/未连接：控制�?无控�?
        // 4. 协同模式：遥控器有输入时显示遥控器，否则显示地面�?
        int activePriority = commanderGetActivePriority();
        bool rcConnected = vstate.isRcConnected;
        bool gcsConnected = (connectionState == REMOTE_STATE_CONNECTED);

        if (activePriority == COMMANDER_PRIORITY_EXTRX)
        {
            // 遥控器正在控制（最高优先级�?
            telemetry.controlSource = (uint8_t)COMMANDER_PRIORITY_EXTRX;
        }
        else if (rcConnected && remoteControlMode == REMOTE_CTRL_MODE_SHARED && gcsConnected)
        {
            // 协同模式：遥控器连接但没有输入，地面站可以控�?
            if (activePriority == COMMANDER_PRIORITY_REMOTE)
            {
                telemetry.controlSource = (uint8_t)COMMANDER_PRIORITY_REMOTE;
            }
            else
            {
                // 协同模式下无活动控制，显示遥控器（遥控器始终优先�?
                telemetry.controlSource = (uint8_t)COMMANDER_PRIORITY_EXTRX;
            }
        }
        else if (!rcConnected && gcsConnected && remoteControlMode == REMOTE_CTRL_MODE_ENABLED)
        {
            // 无遥控器 + 远程控制启用 + 已连接：控制�?地面�?
            telemetry.controlSource = (uint8_t)COMMANDER_PRIORITY_REMOTE;
        }
        else if (activePriority != COMMANDER_PRIORITY_DISABLE)
        {
            // 其他活动控制�?
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

        // 串口输出电机实时数据�?Hz�?
        if ((xTaskGetTickCount() - lastMotorPrintTick) > M2T(1000))
        {
            lastMotorPrintTick = xTaskGetTickCount();
            DEBUG_PRINT("MOTOR_OUT: M1=%u M2=%u M3=%u M4=%u\n",
                        telemetry.motorPower[0], telemetry.motorPower[1],
                        telemetry.motorPower[2], telemetry.motorPower[3]);
        }

        // === V3.2 新增：高度和速度数据 ===
        // 估计器高�?(kalman stateZ, 单位: m -> mm)
        if (LOG_VARID_IS_VALID(estZId))
        {
            telemetry.estAltitude = (int32_t)(logGetFloat(estZId) * 1000.0f);
        }
        else
        {
            telemetry.estAltitude = 0;
        }

        // 校准后的气压计高�?(单位: m -> mm)
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

        // 机体坐标系速度 (V3.3新增)
        if (LOG_VARID_IS_VALID(bodyVxId))
        {
            float rawVx = logGetFloat(bodyVxId);
            telemetry.bodyVelX = (int16_t)(rawVx * 1000.0f);
            // 调试打印（每5秒一次）
            static uint32_t lastBodyVelPrint = 0;
            if (xTaskGetTickCount() - lastBodyVelPrint > pdMS_TO_TICKS(5000))
            {
                DEBUG_PRINT("[BodyVel] statePX=%.4f, statePY=%.4f -> mm/s: %d, %d\n",
                            rawVx, logGetFloat(bodyVyId), telemetry.bodyVelX, telemetry.bodyVelY);
                lastBodyVelPrint = xTaskGetTickCount();
            }
        }
        else
        {
            telemetry.bodyVelX = 0;
        }
        if (LOG_VARID_IS_VALID(bodyVyId))
        {
            telemetry.bodyVelY = (int16_t)(logGetFloat(bodyVyId) * 1000.0f);
        }
        else
        {
            telemetry.bodyVelY = 0;
        }

        // 磁力计数据 (V3.4新增)
        telemetry.magX = LOG_VARID_IS_VALID(magXId) ? (int16_t)(logGetFloat(magXId) * 1000.0f) : 0;
        telemetry.magY = LOG_VARID_IS_VALID(magYId) ? (int16_t)(logGetFloat(magYId) * 1000.0f) : 0;
        telemetry.magZ = LOG_VARID_IS_VALID(magZId) ? (int16_t)(logGetFloat(magZId) * 1000.0f) : 0;
        telemetry.magHeading = LOG_VARID_IS_VALID(magHeadingId) ? (int16_t)(logGetFloat(magHeadingId) * 100.0f) : 0;

        // 通过 MAVLink v2 上报标准遥测消息

        // ATTITUDE (id=30)
        {
            uint8_t payload[28] = {0};
            uint32_t timeBootMs = telemetry.timestamp;
            float rollRad = ((float)telemetry.roll / 100.0f) * ((float)M_PI / 180.0f);
            float pitchRad = ((float)telemetry.pitch / 100.0f) * ((float)M_PI / 180.0f);
            float yawRad = ((float)telemetry.yaw / 100.0f) * ((float)M_PI / 180.0f);
            float rollRateRad = ((float)telemetry.gyroX / 10.0f) * ((float)M_PI / 180.0f);
            float pitchRateRad = ((float)telemetry.gyroY / 10.0f) * ((float)M_PI / 180.0f);
            float yawRateRad = ((float)telemetry.gyroZ / 10.0f) * ((float)M_PI / 180.0f);

            memcpy(&payload[0], &timeBootMs, sizeof(timeBootMs));
            memcpy(&payload[4], &rollRad, sizeof(float));
            memcpy(&payload[8], &pitchRad, sizeof(float));
            memcpy(&payload[12], &yawRad, sizeof(float));
            memcpy(&payload[16], &rollRateRad, sizeof(float));
            memcpy(&payload[20], &pitchRateRad, sizeof(float));
            memcpy(&payload[24], &yawRateRad, sizeof(float));

            sendMavlinkMessage(MAVLINK_MSG_ID_ATTITUDE, payload, sizeof(payload));
        }

        // LOCAL_POSITION_NED (id=32)
        {
            uint8_t payload[28] = {0};
            uint32_t timeBootMs = telemetry.timestamp;
            float x = 0.0f;
            float y = 0.0f;
            float z = -((float)telemetry.estAltitude / 1000.0f);
            float vx = (float)telemetry.estVelX / 1000.0f;
            float vy = (float)telemetry.estVelY / 1000.0f;
            float vz = (float)telemetry.estVelZ / 1000.0f;

            memcpy(&payload[0], &timeBootMs, sizeof(timeBootMs));
            memcpy(&payload[4], &x, sizeof(float));
            memcpy(&payload[8], &y, sizeof(float));
            memcpy(&payload[12], &z, sizeof(float));
            memcpy(&payload[16], &vx, sizeof(float));
            memcpy(&payload[20], &vy, sizeof(float));
            memcpy(&payload[24], &vz, sizeof(float));

            sendMavlinkMessage(MAVLINK_MSG_ID_LOCAL_POSITION_NED, payload, sizeof(payload));
        }

        // SERVO_OUTPUT_RAW (id=36)
        // 这里复用 servo1-4 字段承载四电机当前输出，便于地面站显示。
        {
            uint8_t payload[21] = {0};
            uint32_t timeUsec = telemetry.timestamp * 1000UL;
            uint8_t port = 0;
            uint16_t s1 = telemetry.motorPower[0];
            uint16_t s2 = telemetry.motorPower[1];
            uint16_t s3 = telemetry.motorPower[2];
            uint16_t s4 = telemetry.motorPower[3];
            uint16_t s5 = 0;
            uint16_t s6 = 0;
            uint16_t s7 = 0;
            uint16_t s8 = 0;
            uint8_t rssi = 255;

            memcpy(&payload[0], &timeUsec, sizeof(timeUsec));
            memcpy(&payload[4], &port, sizeof(port));
            memcpy(&payload[5], &s1, sizeof(s1));
            memcpy(&payload[7], &s2, sizeof(s2));
            memcpy(&payload[9], &s3, sizeof(s3));
            memcpy(&payload[11], &s4, sizeof(s4));
            memcpy(&payload[13], &s5, sizeof(s5));
            memcpy(&payload[15], &s6, sizeof(s6));
            memcpy(&payload[17], &s7, sizeof(s7));
            memcpy(&payload[19], &s8, sizeof(s8));
            memcpy(&payload[20], &rssi, sizeof(rssi));

            sendMavlinkMessage(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, payload, sizeof(payload));
        }

        // HIGHRES_IMU (id=105)
        {
            uint8_t payload[62] = {0};
            uint64_t timeUsec = (uint64_t)telemetry.timestamp * 1000ULL;
            float xacc = ((float)telemetry.accX / 1000.0f) * 9.80665f;
            float yacc = ((float)telemetry.accY / 1000.0f) * 9.80665f;
            float zacc = ((float)telemetry.accZ / 1000.0f) * 9.80665f;
            float xgyro = ((float)telemetry.gyroX / 10.0f) * ((float)M_PI / 180.0f);
            float ygyro = ((float)telemetry.gyroY / 10.0f) * ((float)M_PI / 180.0f);
            float zgyro = ((float)telemetry.gyroZ / 10.0f) * ((float)M_PI / 180.0f);
            float xmag = (float)telemetry.magX / 1000.0f;
            float ymag = (float)telemetry.magY / 1000.0f;
            float zmag = (float)telemetry.magZ / 1000.0f;
            float absPress = 0.0f;
            float diffPress = 0.0f;
            float pressAlt = (float)telemetry.baroAltitude / 1000.0f;
            float temperature = 0.0f;
            uint16_t fieldsUpdated = 0x01FF;

            memcpy(&payload[0], &timeUsec, sizeof(timeUsec));
            memcpy(&payload[8], &xacc, sizeof(float));
            memcpy(&payload[12], &yacc, sizeof(float));
            memcpy(&payload[16], &zacc, sizeof(float));
            memcpy(&payload[20], &xgyro, sizeof(float));
            memcpy(&payload[24], &ygyro, sizeof(float));
            memcpy(&payload[28], &zgyro, sizeof(float));
            memcpy(&payload[32], &xmag, sizeof(float));
            memcpy(&payload[36], &ymag, sizeof(float));
            memcpy(&payload[40], &zmag, sizeof(float));
            memcpy(&payload[44], &absPress, sizeof(float));
            memcpy(&payload[48], &diffPress, sizeof(float));
            memcpy(&payload[52], &pressAlt, sizeof(float));
            memcpy(&payload[56], &temperature, sizeof(float));
            memcpy(&payload[60], &fieldsUpdated, sizeof(fieldsUpdated));

            sendMavlinkMessage(MAVLINK_MSG_ID_HIGHRES_IMU, payload, sizeof(payload));
        }

        // SYS_STATUS (id=1) 低频上报
        static uint32_t lastSysStatusTick = 0;
        if ((telemetry.timestamp - lastSysStatusTick) >= 500)
        {
            uint8_t payload[31] = {0};
            uint16_t voltageMv = telemetry.battVoltage;
            int8_t battRemain = (int8_t)telemetry.battPercent;

            memcpy(&payload[14], &voltageMv, sizeof(voltageMv));
            memcpy(&payload[30], &battRemain, sizeof(battRemain));

            sendMavlinkMessage(MAVLINK_MSG_ID_SYS_STATUS, payload, sizeof(payload));
            lastSysStatusTick = telemetry.timestamp;
        }
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
 * 日志和参�?
 *===========================================================================*/

LOG_GROUP_START(remote)
LOG_ADD(LOG_UINT8, connected, &logConnected)
LOG_ADD(LOG_UINT16, txRate, &logTxRate)
LOG_ADD(LOG_UINT16, rxRate, &logRxRate)
LOG_ADD(LOG_UINT32, invalidCmd, &invalidCmdCount)
LOG_GROUP_STOP(remote)

#endif /* CONFIG_REMOTE_SERVER_ENABLE */
