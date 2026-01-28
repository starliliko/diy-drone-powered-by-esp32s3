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

// 自动切换和故障转移机制
typedef struct
{
  uint32_t lastActiveTime; // 最后一次活跃时间
  bool isAlive;            // 是否在线
  uint32_t timeoutCount;   // 超时次数
} ControlSourceHealth;

static ControlSourceHealth sourceHealth[NUM_PRIORITY_LEVELS];

// 日志变量
static uint8_t logActivePriority = 0;
static uint32_t logAcceptedTotal = 0;
static uint32_t logRejectedTotal = 0;
static uint32_t logInvalidTotal = 0;

/*===========================================================================
 * 辅助宏
 *===========================================================================*/

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

// 控制源超时阈值（毫秒）
#define SOURCE_TIMEOUT_MS 1000        // 1秒无数据则认为控制源失联
#define SOURCE_RETRY_INTERVAL_MS 5000 // 5秒后允许重试失联的控制源

/*===========================================================================
 * 控制源健康管理
 *===========================================================================*/

/**
 * 更新控制源健康状态
 */
static void updateSourceHealth(int priority, uint32_t currentTime)
{
  if (priority >= 0 && priority < NUM_PRIORITY_LEVELS)
  {
    sourceHealth[priority].lastActiveTime = currentTime;
    sourceHealth[priority].isAlive = true;
    sourceHealth[priority].timeoutCount = 0;
  }
}

/**
 * 检查控制源是否超时
 */
static bool isSourceTimedOut(int priority, uint32_t currentTime)
{
  if (priority < 0 || priority >= NUM_PRIORITY_LEVELS)
  {
    return true;
  }

  if (!sourceHealth[priority].isAlive)
  {
    return true;
  }

  uint32_t timeSinceActive = currentTime - sourceHealth[priority].lastActiveTime;
  return (timeSinceActive > M2T(SOURCE_TIMEOUT_MS));
}

/**
 * 标记控制源为失联状态
 */
static void markSourceTimeout(int priority)
{
  if (priority >= 0 && priority < NUM_PRIORITY_LEVELS)
  {
    sourceHealth[priority].isAlive = false;
    sourceHealth[priority].timeoutCount++;
    DEBUG_PRINT("Control source %d timed out (count: %lu)\n",
                priority, sourceHealth[priority].timeoutCount);
  }
}

/**
 * 查找最高优先级的可用控制源
 */
static int findBestAvailableSource(uint32_t currentTime)
{
  // 从高优先级到低优先级遍历
  for (int priority = COMMANDER_PRIORITY_EXTRX; priority >= COMMANDER_PRIORITY_CRTP; priority--)
  {
    if (!isSourceTimedOut(priority, currentTime))
    {
      return priority;
    }
  }

  return COMMANDER_PRIORITY_DISABLE;
}

/**
 * 执行控制源故障转移
 */
static bool performSourceFailover(int currentPriority, uint32_t currentTime)
{
  // 检查当前控制源是否超时
  if (!isSourceTimedOut(currentPriority, currentTime))
  {
    return false; // 当前控制源正常，无需切换
  }

  // 标记当前控制源为失联
  markSourceTimeout(currentPriority);

  // 查找次高优先级的可用控制源
  int newPriority = findBestAvailableSource(currentTime);

  if (newPriority != currentPriority)
  {
    DEBUG_PRINT("Failover: %d -> %d\n", currentPriority, newPriority);
    xQueueOverwrite(priorityQueue, &newPriority);
    logActivePriority = newPriority;
    return true;
  }

  return false;
}

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

  // 初始化控制源健康状态
  memset(sourceHealth, 0, sizeof(sourceHealth));
  for (int i = 0; i < NUM_PRIORITY_LEVELS; i++)
  {
    sourceHealth[i].isAlive = false;
  }

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

  // === 更新控制源健康状态 ===
  updateSourceHealth(priority, currentTime);

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

  // === 自动故障转移检查 ===
  int currentPriority = commanderGetActivePriority();

  // 检查当前控制源是否超时，如果超时则尝试切换
  if (currentPriority > COMMANDER_PRIORITY_DISABLE)
  {
    if (isSourceTimedOut(currentPriority, currentTime))
    {
      performSourceFailover(currentPriority, currentTime);
    }
  }

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
    // === 超时但未完全断开：尝试故障转移 ===
    int bestSource = findBestAvailableSource(currentTime);
    if (bestSource != COMMANDER_PRIORITY_DISABLE)
    {
      DEBUG_PRINT("Watchdog failover: %d -> %d\n", currentPriority, bestSource);
      xQueueOverwrite(priorityQueue, &bestSource);
      logActivePriority = bestSource;
    }
    else
    {
      // 所有控制源都失联，进入安全模式
      xQueueOverwrite(priorityQueue, &priorityDisable);
      logActivePriority = COMMANDER_PRIORITY_DISABLE;
    }

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
// 控制源健康状态日志
LOG_ADD(LOG_UINT8, extRxAlive, &sourceHealth[COMMANDER_PRIORITY_EXTRX].isAlive)
LOG_ADD(LOG_UINT8, remoteAlive, &sourceHealth[COMMANDER_PRIORITY_REMOTE].isAlive)
LOG_ADD(LOG_UINT8, crtpAlive, &sourceHealth[COMMANDER_PRIORITY_CRTP].isAlive)
LOG_ADD(LOG_UINT32, extRxTimeout, &sourceHealth[COMMANDER_PRIORITY_EXTRX].timeoutCount)
LOG_ADD(LOG_UINT32, remoteTimeout, &sourceHealth[COMMANDER_PRIORITY_REMOTE].timeoutCount)
LOG_ADD(LOG_UINT32, crtpTimeout, &sourceHealth[COMMANDER_PRIORITY_CRTP].timeoutCount)
LOG_GROUP_STOP(commander)
