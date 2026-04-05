#include "mtf01.h"

/*
MTF01 Micolink 协议驱动 (消息ID: 0x51)

使用方法：
- 使用 mtf01_init() 注册数据回调
- 使用 mtf01_process_byte() 处理串口接收的字节
- 数据解析成功后自动调用回调函数

数据说明：
- 距离值: 单位mm，最小值为2，0表示数据不可用
- tof_status: 1=测距数据可用，0=不可用
- 光流速度: 单位cm/s@1m高度
- flow_status: 1=光流数据可用，0=不可用
- 光流质量: 越大表示可信度越高

计算公式：实际速度(cm/s) = 光流速度 * 高度(m)
*/

/* 内部状态 */
static MICOLINK_MSG_t s_msg;
static mtf01_data_callback_t s_callback = NULL;

/**
 * 初始化 MTF01 驱动
 */
void mtf01_init(mtf01_data_callback_t callback)
{
    s_callback = callback;
    memset(&s_msg, 0, sizeof(s_msg));
}

/**
 * 处理接收到的串口字节
 */
void mtf01_process_byte(uint8_t data)
{
    if (micolink_parse_char(&s_msg, data) == false)
        return;

    switch (s_msg.msg_id)
    {
    case MICOLINK_MSG_ID_RANGE_SENSOR:
    {
        if (s_callback != NULL)
        {
            MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
            memcpy(&payload, s_msg.payload, s_msg.len);
            s_callback(&payload);
        }
        break;
    }

    default:
        break;
    }
}

bool micolink_check_sum(MICOLINK_MSG_t *msg)
{
    uint8_t length = msg->len + 6;
    uint8_t temp[MICOLINK_MAX_LEN];
    uint8_t checksum = 0;

    memcpy(temp, msg, length);

    for (uint8_t i = 0; i < length; i++)
    {
        checksum += temp[i];
    }

    if (checksum == msg->checksum)
        return true;
    else
        return false;
}

bool micolink_parse_char(MICOLINK_MSG_t *msg, uint8_t data)
{
    switch (msg->status)
    {
    case 0: // 帧头
        if (data == MICOLINK_MSG_HEAD)
        {
            msg->head = data;
            msg->status++;
        }
        break;

    case 1: // 设备ID
        msg->dev_id = data;
        msg->status++;
        break;

    case 2: // 系统ID
        msg->sys_id = data;
        msg->status++;
        break;

    case 3: // 消息ID
        msg->msg_id = data;
        msg->status++;
        break;

    case 4: // 包序列
        msg->seq = data;
        msg->status++;
        break;

    case 5: // 负载长度
        msg->len = data;
        if (msg->len == 0)
            msg->status += 2;
        else if (msg->len > MICOLINK_MAX_PAYLOAD_LEN)
            msg->status = 0;
        else
            msg->status++;
        break;

    case 6: // 数据负载接收
        msg->payload[msg->payload_cnt++] = data;
        if (msg->payload_cnt == msg->len)
        {
            msg->payload_cnt = 0;
            msg->status++;
        }
        break;

    case 7: // 帧校验
        msg->checksum = data;
        msg->status = 0;
        if (micolink_check_sum(msg))
        {
            return true;
        }
        break;

    default:
        msg->status = 0;
        msg->payload_cnt = 0;
        break;
    }

    return false;
}
