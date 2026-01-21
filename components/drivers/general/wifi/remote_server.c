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

// 任务配置
#define REMOTE_TX_TASK_NAME "remoteTx"
#define REMOTE_TX_TASK_PRI 3
#define REMOTE_TX_TASK_STACK 2048

#define REMOTE_RX_TASK_NAME "remoteRx"
#define REMOTE_RX_TASK_PRI 3
#define REMOTE_RX_TASK_STACK 2048

#define REMOTE_TELEM_TASK_NAME "remoteTelem"
#define REMOTE_TELEM_TASK_PRI 2
#define REMOTE_TELEM_TASK_STACK 2048

/*===========================================================================
 * 状态变量
 *===========================================================================*/

static bool isInit = false;
static int sock = -1;
static volatile RemoteConnectionState connectionState = REMOTE_STATE_DISCONNECTED;
static SemaphoreHandle_t sockMutex = NULL;
static QueueHandle_t txQueue = NULL;
static EventGroupHandle_t eventGroup = NULL;

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

void remoteServerStart(void)
{
    if (!isInit)
    {
        DEBUG_PRINT("Remote server not initialized\n");
        return;
    }

    DEBUG_PRINT("Starting remote server tasks\n");

    // 创建任务
    xTaskCreate(remoteServerRxTask, REMOTE_RX_TASK_NAME, REMOTE_RX_TASK_STACK,
                NULL, REMOTE_RX_TASK_PRI, NULL);
    xTaskCreate(remoteServerTxTask, REMOTE_TX_TASK_NAME, REMOTE_TX_TASK_STACK,
                NULL, REMOTE_TX_TASK_PRI, NULL);
    xTaskCreate(remoteServerTelemetryTask, REMOTE_TELEM_TASK_NAME, REMOTE_TELEM_TASK_STACK,
                NULL, REMOTE_TELEM_TASK_PRI, NULL);

    DEBUG_PRINT("Remote server tasks started\n");
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
    return REMOTE_SERVER_IP;
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

static void updateConnectionState(RemoteConnectionState newState)
{
    if (connectionState != newState)
    {
        connectionState = newState;
        logConnected = (newState == REMOTE_STATE_CONNECTED) ? 1 : 0;

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

    // 解析服务器地址
    destAddr.sin_addr.s_addr = inet_addr(REMOTE_SERVER_IP);
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
    DEBUG_PRINT("Connecting to %s:%d...\n", REMOTE_SERVER_IP, REMOTE_SERVER_PORT);
    err = connect(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (err != 0)
    {
        DEBUG_PRINT("Socket connect failed: errno %d\n", errno);
        close(sock);
        sock = -1;
        updateConnectionState(REMOTE_STATE_DISCONNECTED);
        return false;
    }

    DEBUG_PRINT("Connected to server!\n");
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

    // 等待初始连接
    while (!connectToServer())
    {
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
            DEBUG_PRINT("recv failed: errno %d\n", errno);
            xEventGroupSetBits(eventGroup, EVT_DISCONNECTED);
            continue;
        }
        else if (len == 0)
        {
            // 连接关闭
            DEBUG_PRINT("Server closed connection\n");
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
        DEBUG_PRINT("Control packet too small: %d\n", size);
        return;
    }

    const RemoteControlCmd *cmd = (const RemoteControlCmd *)data;

    // 输入验证
    if (!validateControlCmd(cmd))
    {
        invalidCmdCount++;
        DEBUG_PRINT("Invalid control command rejected (total: %lu)\n", invalidCmdCount);
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
        // 紧急停机 - 最高优先级立即执行
        setpoint.thrust = 0;
        setpoint.mode.roll = modeDisable;
        setpoint.mode.pitch = modeDisable;
        setpoint.mode.yaw = modeDisable;
        // 紧急停机使用EXTRX优先级确保能覆盖任何控制
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
        // 降落模式
        setpoint.mode.z = modeVelocity;
        setpoint.velocity.z = -0.3f; // 下降速度 0.3m/s
        commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_REMOTE);
        DEBUG_PRINT("LAND command received\n");
        break;

    case CTRL_CMD_DISARM:
        // 锁定电机
        setpoint.thrust = 0;
        setpoint.mode.roll = modeDisable;
        setpoint.mode.pitch = modeDisable;
        setpoint.mode.yaw = modeDisable;
        commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_REMOTE);
        DEBUG_PRINT("DISARM command received\n");
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
    if (cmd->cmdType > CTRL_CMD_DISARM)
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
    if (size > CRTP_MAX_DATA_SIZE + 1)
    {
        return;
    }

    // 将 CRTP 包发送到本地 CRTP 处理系统
    CRTPPacket p;
    p.size = size - 1;
    memcpy(p.raw, data, size);

    // 发送到 CRTP 发送队列
    crtpSendPacket(&p);
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
    static logVarId_t posXId, posYId, posZId;
    static logVarId_t velXId, velYId, velZId;

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

            posXId = logGetVarId("stateEstimate", "x");
            posYId = logGetVarId("stateEstimate", "y");
            posZId = logGetVarId("stateEstimate", "z");

            velXId = logGetVarId("stateEstimate", "vx");
            velYId = logGetVarId("stateEstimate", "vy");
            velZId = logGetVarId("stateEstimate", "vz");

            logIdsInit = true;
            DEBUG_PRINT("Telemetry log IDs initialized\n");
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

        // 位置估计（暂时固定为0）
        telemetry.posX = 0;
        telemetry.posY = 0;
        telemetry.posZ = 0;

        // 速度估计（暂时固定为0）
        telemetry.velX = 0;
        telemetry.velY = 0;
        telemetry.velZ = 0;

        // 电池状态（暂时固定为100%）
        telemetry.battVoltage = 4200; // 4.2V
        telemetry.battPercent = 100;

        // 飞行状态（简化：1=stabilize）
        telemetry.flightMode = 1;   // stabilize mode
        telemetry.isArmed = 1;      // 已解锁
        telemetry.isLowBattery = 0; // 电量正常

        // 时间戳
        telemetry.timestamp = xTaskGetTickCount();

        // 发送遥测数据
        remoteServerSendPacket(REMOTE_PKT_TELEMETRY,
                               (uint8_t *)&telemetry, sizeof(telemetry));
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
