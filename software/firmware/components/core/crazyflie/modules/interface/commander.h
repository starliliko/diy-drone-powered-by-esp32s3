/**
 *    ||          ____  _ __  ______
 * +------+      / __ )(_) /_/ ____/_________ _____  ___
 * | 0xBC |     / __  / / __/ /    / ___/ __ `/_  / / _	\
 * +------+    / /_/ / / /_/ /___ / /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\____//_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef COMMANDER_H_
#define COMMANDER_H_
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "stabilizer_types.h"

#define DEFAULT_YAW_MODE XMODE

/*===========================================================================
 * 超时配置
 *===========================================================================*/
#define COMMANDER_WDT_TIMEOUT_STABILIZE M2T(500)
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN M2T(2000)
#define COMMANDER_PRIORITY_LOCK_TIME_MS 1000 // 优先级锁定时间（防抖动）

/*===========================================================================
 * 优先级定义（数值越大优先级越高）
 *===========================================================================*/
#define COMMANDER_PRIORITY_DISABLE 0 // 自动模式/超时保护
#define COMMANDER_PRIORITY_CRTP 1    // CRTP协议（Wi-Fi UDP/USB）
#define COMMANDER_PRIORITY_REMOTE 2  // 远程服务器TCP直接控制
#define COMMANDER_PRIORITY_EXTRX 3   // 外部RC接收机（SBUS）最高优先级

/*===========================================================================
 * 安全限制参数
 *===========================================================================*/
#define COMMANDER_MAX_ROLL_DEG 45.0f      // 最大横滚角（度）
#define COMMANDER_MAX_PITCH_DEG 45.0f     // 最大俯仰角（度）
#define COMMANDER_MAX_YAW_RATE_DEG 200.0f // 最大偏航速率（度/秒）
#define COMMANDER_MAX_THRUST 60000        // 最大推力（留10%安全裕度）
#define COMMANDER_MIN_THRUST 0            // 最小推力

/*===========================================================================
 * 通信路径统计结构体
 *===========================================================================*/
typedef struct
{
    uint32_t packetCount;    // 接收的包总数
    uint32_t acceptedCount;  // 被接受的包数
    uint32_t rejectedCount;  // 被拒绝的包数（优先级不足）
    uint32_t invalidCount;   // 无效包数（验证失败）
    uint32_t lastUpdateTime; // 最后更新时间
} CommanderPathStats;

/*===========================================================================
 * API 函数
 *===========================================================================*/
void commanderInit(void);
bool commanderTest(void);
uint32_t commanderGetInactivityTime(void);

void commanderSetSetpoint(setpoint_t *setpoint, int priority);
int commanderGetActivePriority(void);

/**
 * @brief 带验证的setpoint设置（推荐使用）
 * @param setpoint 目标设定点
 * @param priority 优先级
 * @return true=成功 false=验证失败或优先级不足
 */
bool commanderSetSetpointSafe(setpoint_t *setpoint, int priority);

/**
 * @brief 验证setpoint是否在安全范围内
 * @param setpoint 待验证的设定点
 * @return true=安全 false=超出限制
 */
bool commanderValidateSetpoint(const setpoint_t *setpoint);

/**
 * @brief 钳位setpoint到安全范围
 * @param setpoint 要钳位的设定点（就地修改）
 */
void commanderClampSetpoint(setpoint_t *setpoint);

/**
 * @brief 根据当前飞行模式重写 setpoint 的控制语义
 *
 * 将"原始RPYT"(姿态角+油门) setpoint 按飞行模式转换为控制器需要的格式:
 * - STABILIZE: 保持原样 (roll/pitch=角度, yaw=角速度, thrust=直接推力)
 * - ALTITUDE:  thrust → z 速度 (-1~+1), mode.z=modeVelocity
 * - POSITION:  额外将 roll/pitch → xy 速度, mode.x/y=modeVelocity
 *
 * 所有输入路径 (CRTP / SBUS / Remote) 在发送 RPYT 类 setpoint 前都应调用此函数。
 *
 * @param setpoint 已填充好 roll/pitch/yaw/thrust 的 setpoint（就地修改）
 */
void commanderApplyFlightMode(setpoint_t *setpoint);

/**
 * @brief 获取指定路径的统计信息
 * @param priority 优先级/路径ID
 * @return 统计结构体指针（只读）
 */
const CommanderPathStats *commanderGetPathStats(int priority);

/**
 * @brief 重置所有路径统计
 */
void commanderResetStats(void);

/* Inform the commander that streaming setpoints are about to stop.
 * Parameter controls the amount of time the last setpoint will remain valid.
 * This gives the PC time to send the next command, e.g. with the high-level
 * commander, before we enter timeout mode.
 */
void commanderNotifySetpointsStop(int remainValidMillisecs);

void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state);
void commanderGetCurrentSetpoint(setpoint_t *setpoint);

/**
 * @brief 获取当前 setpoint 中的推力值（不触发超时/故障转移逻辑）
 * 用于解锁前油门归零检查。
 */
float commanderGetCurrentThrust(void);

#endif /* COMMANDER_H_ */
