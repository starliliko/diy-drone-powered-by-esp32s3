/**
 * @file remote_server.h
 * @brief 远程服务器通信模块 - 实现无人机与云端服务器的双向通信
 * 
 * 功能说明：
 * 1. TCP 客户端连接到远程服务器
 * 2. 上行：定期发送遥测数据（姿态、位置、电池等）
 * 3. 下行：接收服务器控制指令，转发到 CRTP 系统
 * 4. 支持断线自动重连
 * 5. 心跳保活机制
 */

#ifndef REMOTE_SERVER_H_
#define REMOTE_SERVER_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 数据包定义
 *===========================================================================*/

// 数据包类型定义
typedef enum {
    REMOTE_PKT_HEARTBEAT     = 0x00,  // 心跳包
    REMOTE_PKT_TELEMETRY     = 0x01,  // 遥测数据（上行）
    REMOTE_PKT_CONTROL       = 0x02,  // 控制指令（下行）
    REMOTE_PKT_CRTP          = 0x03,  // CRTP 透传包
    REMOTE_PKT_ACK           = 0x04,  // 确认包
    REMOTE_PKT_CONFIG        = 0x05,  // 配置包
    REMOTE_PKT_LOG           = 0x06,  // 日志包
} RemotePacketType;

// 通信协议包头 (固定8字节)
typedef struct __attribute__((packed)) {
    uint8_t  magic[2];      // 魔数 0xAB 0xCD
    uint8_t  version;       // 协议版本
    uint8_t  type;          // 包类型 RemotePacketType
    uint16_t seq;           // 序列号
    uint16_t length;        // 数据长度(不含包头)
} RemotePacketHeader;

#define REMOTE_MAGIC_0  0xAB
#define REMOTE_MAGIC_1  0xCD
#define REMOTE_PROTOCOL_VERSION  0x01
#define REMOTE_MAX_PAYLOAD_SIZE  128

// 遥测数据结构（上行）
typedef struct __attribute__((packed)) {
    // 姿态 (单位: 度 * 100)
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    // 角速度 (单位: deg/s * 10)
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    // 加速度 (单位: mg)
    int16_t accX;
    int16_t accY;
    int16_t accZ;
    // 位置估计 (单位: mm)
    int32_t posX;
    int32_t posY;
    int32_t posZ;
    // 速度估计 (单位: mm/s)
    int16_t velX;
    int16_t velY;
    int16_t velZ;
    // 电池 (mV, %)
    uint16_t battVoltage;
    uint8_t  battPercent;
    // 状态标志
    uint8_t  flightMode;
    uint8_t  isArmed;
    uint8_t  isLowBattery;
    // 时间戳 (ms)
    uint32_t timestamp;
} RemoteTelemetryData;

// 控制指令结构（下行）
typedef struct __attribute__((packed)) {
    uint8_t  cmdType;       // 指令类型
    int16_t  roll;          // 横滚角 (度*100) 或 X速度
    int16_t  pitch;         // 俯仰角 (度*100) 或 Y速度
    int16_t  yaw;           // 偏航速率 (deg/s*10) 或 偏航角
    uint16_t thrust;        // 油门 (0-65535)
    uint8_t  mode;          // 控制模式
    uint8_t  reserved[3];   // 保留
} RemoteControlCmd;

// 控制指令类型
typedef enum {
    CTRL_CMD_RPYT       = 0x00,  // Roll/Pitch/Yaw/Thrust 模式
    CTRL_CMD_VELOCITY   = 0x01,  // 速度控制模式
    CTRL_CMD_POSITION   = 0x02,  // 位置控制模式
    CTRL_CMD_HOVER      = 0x03,  // 悬停
    CTRL_CMD_LAND       = 0x04,  // 降落
    CTRL_CMD_EMERGENCY  = 0x05,  // 紧急停机
    CTRL_CMD_ARM        = 0x06,  // 解锁
    CTRL_CMD_DISARM     = 0x07,  // 上锁
} RemoteControlCmdType;

// 连接状态
typedef enum {
    REMOTE_STATE_DISCONNECTED = 0,
    REMOTE_STATE_CONNECTING,
    REMOTE_STATE_CONNECTED,
    REMOTE_STATE_ERROR,
} RemoteConnectionState;

// 状态回调类型
typedef void (*RemoteStateCallback)(RemoteConnectionState state);
typedef void (*RemoteControlCallback)(const RemoteControlCmd* cmd);

/*===========================================================================
 * API 函数
 *===========================================================================*/

/**
 * @brief 初始化远程服务器通信模块（创建资源）
 * @note 在系统初始化阶段调用
 */
void remoteServerInit(void);

/**
 * @brief 启动远程服务器任务
 * @note 需要在 Wi-Fi STA 模式连接成功后调用，由 system.c 统一调度
 */
void remoteServerStart(void);

/**
 * @brief 测试模块是否初始化
 */
bool remoteServerTest(void);

/**
 * @brief 获取当前连接状态
 */
RemoteConnectionState remoteServerGetState(void);

/**
 * @brief 检查是否已连接到服务器
 */
bool remoteServerIsConnected(void);

/**
 * @brief 注册连接状态变化回调
 */
void remoteServerSetStateCallback(RemoteStateCallback cb);

/**
 * @brief 注册控制指令接收回调
 */
void remoteServerSetControlCallback(RemoteControlCallback cb);

/**
 * @brief 发送 CRTP 透传包到服务器
 * @param data CRTP 数据
 * @param size 数据大小
 * @return 是否发送成功
 */
bool remoteServerSendCRTP(const uint8_t* data, uint32_t size);

/**
 * @brief 发送自定义数据包到服务器
 * @param type 包类型
 * @param data 数据
 * @param size 数据大小
 * @return 是否发送成功
 */
bool remoteServerSendPacket(RemotePacketType type, const uint8_t* data, uint16_t size);

/**
 * @brief 手动触发重连
 */
void remoteServerReconnect(void);

/**
 * @brief 获取服务器 IP 信息（用于调试）
 */
const char* remoteServerGetIP(void);

/**
 * @brief 获取连接统计信息
 */
void remoteServerGetStats(uint32_t* txCount, uint32_t* rxCount, 
                          uint32_t* reconnectCount, uint32_t* uptime);

#ifdef __cplusplus
}
#endif

#endif /* REMOTE_SERVER_H_ */
