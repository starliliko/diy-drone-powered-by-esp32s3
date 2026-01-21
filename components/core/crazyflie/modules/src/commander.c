/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 *
 */
#include <string.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "commander.h"
#include "crtp_commander.h"
#include "crtp_commander_high_level.h"

#include "cf_math.h"
#include "param.h"
#include "log.h"
#include "stm32_legacy.h"
#include "static_mem.h"

#define DEBUG_MODULE "CMD"
#include "debug_cf.h"

/*===========================================================================
 * 内部变量
 *===========================================================================*/

static bool isInit;
const static setpoint_t nullSetpoint;
static setpoint_t tempSetpoint;
static state_t lastState;
const static int priorityDisable = COMMANDER_PRIORITY_DISABLE;

static uint32_t lastUpdate;
static bool enableHighLevel = false;

static QueueHandle_t setpointQueue;
STATIC_MEM_QUEUE_ALLOC(setpointQueue, 1, sizeof(setpoint_t));
static QueueHandle_t priorityQueue;
STATIC_MEM_QUEUE_ALLOC(priorityQueue, 1, sizeof(int));

// 优先级锁定机制（防止抖动）
static bool priorityLocked = false;
static uint32_t priorityLockExpireTime = 0;
static int lockedPriority = COMMANDER_PRIORITY_DISABLE;

// 通信路径统计（按优先级分类）
#define NUM_PRIORITY_LEVELS 4
static CommanderPathStats pathStats[NUM_PRIORITY_LEVELS];

// 日志变量
static uint8_t logActivePriority = 0;
static uint32_t logAcceptedTotal = 0;
static uint32_t logRejectedTotal = 0;
static uint32_t logInvalidTotal = 0;

/*===========================================================================
 * 辅助宏
 *===========================================================================*/

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/* Public functions */
void commanderInit(void)
{
  setpointQueue = STATIC_MEM_QUEUE_CREATE(setpointQueue);
  ASSERT(setpointQueue);
  xQueueSend(setpointQueue, &nullSetpoint, 0);

  priorityQueue = STATIC_MEM_QUEUE_CREATE(priorityQueue);
  ASSERT(priorityQueue);
  xQueueSend(priorityQueue, &priorityDisable, 0);

  // 初始化路径统计
  memset(pathStats, 0, sizeof(pathStats));

  // 初始化优先级锁定
  priorityLocked = false;
  priorityLockExpireTime = 0;
  lockedPriority = COMMANDER_PRIORITY_DISABLE;

  crtpCommanderInit();              // CRTP指令初始化
  crtpCommanderHighLevelInit();     // 高级CRTP指令初始化
  lastUpdate = xTaskGetTickCount(); // 记录初始化时间

  isInit = true;
  DEBUG_PRINT("Commander initialized with safety limits enabled\n");
}

/*===========================================================================
 * Setpoint 验证和钳位
 *===========================================================================*/

bool commanderValidateSetpoint(const setpoint_t *setpoint)
{
  if (setpoint == NULL)
  {
    return false;
  }

  // 检查姿态角是否在安全范围内
  if (fabsf(setpoint->attitude.roll) > COMMANDER_MAX_ROLL_DEG)
  {
    return false;
  }
  if (fabsf(setpoint->attitude.pitch) > COMMANDER_MAX_PITCH_DEG)
  {
    return false;
  }

  // 检查偏航速率
  if (fabsf(setpoint->attitudeRate.yaw) > COMMANDER_MAX_YAW_RATE_DEG)
  {
    return false;
  }

  // 检查推力范围
  if (setpoint->thrust > COMMANDER_MAX_THRUST)
  {
    return false;
  }

  // 检查NaN和Inf
  if (isnan(setpoint->attitude.roll) || isinf(setpoint->attitude.roll) ||
      isnan(setpoint->attitude.pitch) || isinf(setpoint->attitude.pitch) ||
      isnan(setpoint->attitudeRate.yaw) || isinf(setpoint->attitudeRate.yaw))
  {
    return false;
  }

  return true;
}

void commanderClampSetpoint(setpoint_t *setpoint)
{
  if (setpoint == NULL)
  {
    return;
  }

  // 钳位姿态角
  setpoint->attitude.roll = CLAMP(setpoint->attitude.roll,
                                  -COMMANDER_MAX_ROLL_DEG, COMMANDER_MAX_ROLL_DEG);
  setpoint->attitude.pitch = CLAMP(setpoint->attitude.pitch,
                                   -COMMANDER_MAX_PITCH_DEG, COMMANDER_MAX_PITCH_DEG);

  // 钳位偏航速率
  setpoint->attitudeRate.yaw = CLAMP(setpoint->attitudeRate.yaw,
                                     -COMMANDER_MAX_YAW_RATE_DEG, COMMANDER_MAX_YAW_RATE_DEG);

  // 钳位推力
  if (setpoint->thrust > COMMANDER_MAX_THRUST)
  {
    setpoint->thrust = COMMANDER_MAX_THRUST;
  }

  // 处理NaN/Inf - 替换为安全值
  if (isnan(setpoint->attitude.roll) || isinf(setpoint->attitude.roll))
  {
    setpoint->attitude.roll = 0.0f;
  }
  if (isnan(setpoint->attitude.pitch) || isinf(setpoint->attitude.pitch))
  {
    setpoint->attitude.pitch = 0.0f;
  }
  if (isnan(setpoint->attitudeRate.yaw) || isinf(setpoint->attitudeRate.yaw))
  {
    setpoint->attitudeRate.yaw = 0.0f;
  }
}

/*===========================================================================
 * 优先级锁定机制
 *===========================================================================*/

static bool checkPriorityLock(int priority, uint32_t currentTime)
{
  // 检查锁定是否过期
  if (priorityLocked && currentTime >= priorityLockExpireTime)
  {
    priorityLocked = false;
    DEBUG_PRINT("Priority lock expired\n");
  }

  // 如果锁定中且新优先级低于锁定优先级，拒绝
  if (priorityLocked && priority < lockedPriority)
  {
    return false;
  }

  return true;
}

static void updatePriorityLock(int newPriority, int currentPriority, uint32_t currentTime)
{
  // 如果新优先级高于当前优先级，启动锁定
  if (newPriority > currentPriority)
  {
    priorityLocked = true;
    lockedPriority = newPriority;
    priorityLockExpireTime = currentTime + M2T(COMMANDER_PRIORITY_LOCK_TIME_MS);
  }
}

/*===========================================================================
 * Setpoint 设置函数
 *===========================================================================*/

void commanderSetSetpoint(setpoint_t *setpoint, int priority)
{
  int currentPriority;
  uint32_t currentTime = xTaskGetTickCount();

  // 范围检查优先级
  int statsIndex = (priority >= 0 && priority < NUM_PRIORITY_LEVELS) ? priority : 0;

  // 更新统计
  pathStats[statsIndex].packetCount++;
  pathStats[statsIndex].lastUpdateTime = currentTime;

  const BaseType_t peekResult = xQueuePeek(priorityQueue, &currentPriority, 0);
  ASSERT(peekResult == pdTRUE);

  // 检查优先级锁定
  if (!checkPriorityLock(priority, currentTime))
  {
    pathStats[statsIndex].rejectedCount++;
    logRejectedTotal++;
    return;
  }

  if (priority >= currentPriority)
  {
    // 钳位到安全范围（保守策略：即使无效也尝试修正）
    commanderClampSetpoint(setpoint);

    setpoint->timestamp = currentTime;
    // This is a potential race but without effect on functionality
    xQueueOverwrite(setpointQueue, setpoint);
    xQueueOverwrite(priorityQueue, &priority);

    // 更新优先级锁定
    updatePriorityLock(priority, currentPriority, currentTime);

    // Send the high-level planner to idle so it will forget its current state
    // and start over if we switch from low-level to high-level in the future.
    crtpCommanderHighLevelStop();

    pathStats[statsIndex].acceptedCount++;
    logAcceptedTotal++;
    logActivePriority = priority;
  }
  else
  {
    pathStats[statsIndex].rejectedCount++;
    logRejectedTotal++;
  }
}

bool commanderSetSetpointSafe(setpoint_t *setpoint, int priority)
{
  uint32_t currentTime = xTaskGetTickCount();
  int statsIndex = (priority >= 0 && priority < NUM_PRIORITY_LEVELS) ? priority : 0;

  // 先验证
  if (!commanderValidateSetpoint(setpoint))
  {
    pathStats[statsIndex].invalidCount++;
    logInvalidTotal++;
    DEBUG_PRINT("Invalid setpoint rejected\n");
    return false;
  }

  // 调用普通设置函数（内部会钳位）
  int currentPriority = commanderGetActivePriority();

  if (!checkPriorityLock(priority, currentTime))
  {
    return false;
  }

  if (priority >= currentPriority)
  {
    commanderSetSetpoint(setpoint, priority);
    return true;
  }

  return false;
}

void commanderNotifySetpointsStop(int remainValidMillisecs)
{
  uint32_t currentTime = xTaskGetTickCount();
  int timeSetback = MIN(
      COMMANDER_WDT_TIMEOUT_SHUTDOWN - M2T(remainValidMillisecs),
      currentTime);
  xQueuePeek(setpointQueue, &tempSetpoint, 0);
  tempSetpoint.timestamp = currentTime - timeSetback;
  xQueueOverwrite(setpointQueue, &tempSetpoint);
  crtpCommanderHighLevelTellState(&lastState);
}

void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state)
{
  xQueuePeek(setpointQueue, setpoint, 0);
  lastUpdate = setpoint->timestamp;
  uint32_t currentTime = xTaskGetTickCount();

  if ((currentTime - setpoint->timestamp) > COMMANDER_WDT_TIMEOUT_SHUTDOWN)
  {
    if (enableHighLevel)
    {
      crtpCommanderHighLevelGetSetpoint(setpoint, state);
    }
    if (!enableHighLevel || crtpCommanderHighLevelIsStopped())
    {
      memcpy(setpoint, &nullSetpoint, sizeof(nullSetpoint));
    }
  }
  else if ((currentTime - setpoint->timestamp) > COMMANDER_WDT_TIMEOUT_STABILIZE)
  {
    xQueueOverwrite(priorityQueue, &priorityDisable);
    // Leveling ...
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;
    setpoint->mode.roll = modeAbs;
    setpoint->mode.pitch = modeAbs;
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitude.roll = 0;
    setpoint->attitude.pitch = 0;
    setpoint->attitudeRate.yaw = 0;
    // Keep Z as it is
  }
  // This copying is not strictly necessary because stabilizer.c already keeps
  // a static state_t containing the most recent state estimate. However, it is
  // not accessible by the public interface.
  lastState = *state;
}

bool commanderTest(void)
{
  return isInit;
}

uint32_t commanderGetInactivityTime(void)
{
  return xTaskGetTickCount() - lastUpdate;
}

int commanderGetActivePriority(void)
{
  int priority;

  const BaseType_t peekResult = xQueuePeek(priorityQueue, &priority, 0);
  ASSERT(peekResult == pdTRUE);

  return priority;
}

/*===========================================================================
 * 统计信息
 *===========================================================================*/

const CommanderPathStats *commanderGetPathStats(int priority)
{
  if (priority >= 0 && priority < NUM_PRIORITY_LEVELS)
  {
    return &pathStats[priority];
  }
  return NULL;
}

void commanderResetStats(void)
{
  memset(pathStats, 0, sizeof(pathStats));
  logAcceptedTotal = 0;
  logRejectedTotal = 0;
  logInvalidTotal = 0;
}

/*===========================================================================
 * 参数和日志
 *===========================================================================*/

PARAM_GROUP_START(commander)
PARAM_ADD(PARAM_UINT8, enHighLevel, &enableHighLevel)
PARAM_GROUP_STOP(commander)

LOG_GROUP_START(commander)
LOG_ADD(LOG_UINT8, activePri, &logActivePriority)
LOG_ADD(LOG_UINT32, accepted, &logAcceptedTotal)
LOG_ADD(LOG_UINT32, rejected, &logRejectedTotal)
LOG_ADD(LOG_UINT32, invalid, &logInvalidTotal)
LOG_GROUP_STOP(commander)
