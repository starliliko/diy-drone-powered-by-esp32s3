#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define MICOLINK_MSG_HEAD 0xEF
#define MICOLINK_MAX_PAYLOAD_LEN 64
#define MICOLINK_MAX_LEN MICOLINK_MAX_PAYLOAD_LEN + 7

/*
    消息ID定义
*/
enum
{
    MICOLINK_MSG_ID_RANGE_SENSOR = 0x51, // 测距传感器
};

/*
    消息结构体定义
*/
typedef struct
{
    uint8_t head;
    uint8_t dev_id;
    uint8_t sys_id;
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[MICOLINK_MAX_PAYLOAD_LEN];
    uint8_t checksum;

    uint8_t status;
    uint8_t payload_cnt;
} MICOLINK_MSG_t;

/*
    数据负载定义
*/
#pragma pack(1)
// 测距传感器
typedef struct
{
    uint32_t time_ms;     // 系统时间 ms
    uint32_t distance;    // 距离(mm) 最小值为10，0表示数据不可用
    uint8_t strength;     // 信号强度
    uint8_t precision;    // 精度
    uint8_t tof_status;   // 状态
    uint8_t reserved1;    // 预留
    int16_t flow_vel_x;   // 光流速度x轴
    int16_t flow_vel_y;   // 光流速度y轴
    uint8_t flow_quality; // 光流质量
    uint8_t flow_status;  // 光流状态
    uint16_t reserved2;   // 预留
} MICOLINK_PAYLOAD_RANGE_SENSOR_t;
#pragma pack()

/**
 * 数据回调函数类型
 * @param payload 解析后的传感器数据
 */
typedef void (*mtf01_data_callback_t)(const MICOLINK_PAYLOAD_RANGE_SENSOR_t *payload);

/**
 * 初始化 MTF01 驱动
 * @param callback 数据接收回调函数
 */
void mtf01_init(mtf01_data_callback_t callback);

/**
 * 处理接收到的串口字节
 * 内部解析 Micolink 协议，解析成功后调用回调函数
 * @param data 接收到的字节
 */
void mtf01_process_byte(uint8_t data);

/**
 * Parse a single byte using Micolink protocol
 * @param msg Message structure to store parsed data
 * @param data Byte to parse
 * @return true if a complete message was parsed successfully
 */
bool micolink_parse_char(MICOLINK_MSG_t *msg, uint8_t data);