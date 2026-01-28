/**
 * @file vehicle_state.c
 * @brief 飞行器状态管理模块实现 - PX4/QGC 风格状态机
 *
 * 实现类似 PX4 的飞行器状态管理，包括：
 * - 解锁状态机（带安全检查）
 * - 飞行模式管理
 * - 飞行阶段检测
 * - 故障安全处理
 */

#include "vehicle_state.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "system.h"
#include "sensors.h"
#include "stabilizer.h"
#include "commander.h"
#include "crtp_commander.h"
#include "pm_esplane.h"
#include "log.h"
#include "param.h"

#define DEBUG_MODULE "VEHICLE"
#include "debug_cf.h"

/*===========================================================================
 * 配置参数
 *===========================================================================*/

// 解锁检查配置
#define ARM_CHECK_ENABLED (ARM_CHECK_SENSORS | ARM_CHECK_BATTERY | ARM_CHECK_THROTTLE_ZERO)
#define BATTERY_LOW_THRESHOLD_MV 3300      // 单节电池低电量阈值 (mV)
#define BATTERY_CRITICAL_THRESHOLD_MV 3100 // 单节电池严重低电量阈值 (mV)
#define ATTITUDE_LEVEL_THRESHOLD_DEG 10.0f // 水平放置检查阈值（度）
#define TAKEOFF_THRUST_THRESHOLD 10000     // 起飞油门阈值
#define LANDED_VELOCITY_THRESHOLD 0.1f     // 降落速度阈值 (m/s)
#define LANDED_TIME_THRESHOLD_MS 1000      // 降落确认时间

/*===========================================================================
 * 状态变量
 *===========================================================================*/

static VehicleState vehicleState = {
    .armingState = ARMING_STATE_INIT,
    .flightMode = FLIGHT_MODE_STABILIZE,
    .flightPhase = FLIGHT_PHASE_ON_GROUND,
    .failsafeState = FAILSAFE_NONE,
    .armFailReason = ARM_FAIL_NONE,
    .armingCheckFlags = ARM_CHECK_ENABLED,
    .armingCheckResult = 0,
    .isArmed = false,
    .isFlying = false,
    .isEmergency = false,
    .isRcConnected = false,
    .isGcsConnected = false,
    .isBatteryLow = false,
    .isSensorHealthy = false,
    .armTime = 0,
    .flightTime = 0,
    .lastStateChange = 0,
};

static SemaphoreHandle_t stateMutex = NULL;
static bool isInit = false;

// 用于记录的变量
static uint8_t logArmingState = 0;
static uint8_t logFlightMode = 0;
static uint8_t logFlightPhase = 0;
static uint8_t logFailsafe = 0;
static uint8_t logIsArmed = 0;
static uint8_t logIsFlying = 0;

/*===========================================================================
 * 私有函数声明
 *===========================================================================*/

static bool runArmingChecks(bool force);
static void updateFlightPhase(void);
static void updateFailsafe(void);
static void syncWithLegacySystem(void);

/*===========================================================================
 * 状态名称字符串
 *===========================================================================*/

static const char *armingStateNames[] = {
    "INIT",
    "STANDBY",
    "ARMED",
    "STANDBY_ERROR",
    "SHUTDOWN",
    "IN_AIR_RESTORE"};

static const char *flightModeNames[] = {
    "MANUAL",
    "STABILIZE",
    "ALTITUDE",
    "POSITION",
    "MISSION",
    "RTL",
    "LAND",
    "TAKEOFF",
    "OFFBOARD"};

static const char *flightPhaseNames[] = {
    "ON_GROUND",
    "TAKEOFF",
    "IN_AIR",
    "LANDING",
    "LANDED"};

static const char *armFailReasonStrs[] = {
    "None",
    "Sensors not ready",
    "Sensors calibrating",
    "RC not connected",
    "Battery low",
    "Throttle not zero",
    "Attitude not level",
    "Already armed",
    "System error",
    "Emergency stop active"};

/*===========================================================================
 * 公共 API 实现
 *===========================================================================*/

void vehicleStateInit(void)
{
    if (isInit)
        return;

    stateMutex = xSemaphoreCreateMutex();
    if (stateMutex == NULL)
    {
        DEBUG_PRINT("Failed to create state mutex!\n");
        return;
    }

    // 等待传感器就绪后切换到 STANDBY
    vehicleState.armingState = ARMING_STATE_INIT;
    vehicleState.lastStateChange = xTaskGetTickCount();

    isInit = true;
    DEBUG_PRINT("Vehicle state manager initialized\n");
}

VehicleState vehicleStateGet(void)
{
    VehicleState copy;
    if (stateMutex && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        copy = vehicleState;
        xSemaphoreGive(stateMutex);
    }
    else
    {
        copy = vehicleState; // 超时时返回非保护副本
    }
    return copy;
}

ArmingState vehicleGetArmingState(void)
{
    return vehicleState.armingState;
}

VehicleFlightMode vehicleGetFlightMode(void)
{
    return vehicleState.flightMode;
}

FlightPhase vehicleGetFlightPhase(void)
{
    return vehicleState.flightPhase;
}

bool vehicleIsArmed(void)
{
    return vehicleState.isArmed;
}

bool vehicleIsFlying(void)
{
    return vehicleState.isFlying;
}

bool vehicleArm(bool force)
{
    if (!isInit)
        return false;

    xSemaphoreTake(stateMutex, portMAX_DELAY);

    // 检查当前状态是否允许解锁
    if (vehicleState.armingState == ARMING_STATE_ARMED)
    {
        vehicleState.armFailReason = ARM_FAIL_ALREADY_ARMED;
        xSemaphoreGive(stateMutex);
        DEBUG_PRINT("Arm failed: already armed\n");
        return false;
    }

    if (vehicleState.isEmergency && !force)
    {
        vehicleState.armFailReason = ARM_FAIL_EMERGENCY_STOP;
        xSemaphoreGive(stateMutex);
        DEBUG_PRINT("Arm failed: emergency stop active\n");
        return false;
    }

    // 运行解锁检查
    if (!runArmingChecks(force))
    {
        xSemaphoreGive(stateMutex);
        DEBUG_PRINT("Arm failed: %s\n", vehicleGetArmFailReasonStr());
        return false;
    }

    // 解锁成功
    vehicleState.armingState = ARMING_STATE_ARMED;
    vehicleState.isArmed = true;
    vehicleState.armTime = xTaskGetTickCount();
    vehicleState.lastStateChange = vehicleState.armTime;
    vehicleState.armFailReason = ARM_FAIL_NONE;

    // 同步到旧系统
    systemSetArmed(true);

    xSemaphoreGive(stateMutex);

    DEBUG_PRINT("=== ARMED ===\n");
    DEBUG_PRINT("Flight Mode: %s\n", vehicleGetFlightModeName(vehicleState.flightMode));

    return true;
}

bool vehicleDisarm(bool force)
{
    if (!isInit)
        return false;

    xSemaphoreTake(stateMutex, portMAX_DELAY);

    // 安全检查：飞行中不允许上锁（除非强制）
    if (vehicleState.isFlying && !force)
    {
        xSemaphoreGive(stateMutex);
        DEBUG_PRINT("Disarm rejected: vehicle is flying\n");
        return false;
    }

    // 上锁
    vehicleState.armingState = ARMING_STATE_STANDBY;
    vehicleState.isArmed = false;
    vehicleState.flightPhase = FLIGHT_PHASE_ON_GROUND;
    vehicleState.isFlying = false;
    vehicleState.lastStateChange = xTaskGetTickCount();

    // 计算飞行时间
    if (vehicleState.armTime > 0)
    {
        vehicleState.flightTime += (xTaskGetTickCount() - vehicleState.armTime);
    }
    vehicleState.armTime = 0;

    // 同步到旧系统
    systemSetArmed(false);

    xSemaphoreGive(stateMutex);

    DEBUG_PRINT("=== DISARMED ===\n");

    return true;
}

ArmingFailReason vehicleGetArmFailReason(void)
{
    return vehicleState.armFailReason;
}

const char *vehicleGetArmFailReasonStr(void)
{
    if (vehicleState.armFailReason < ARM_FAIL_MAX)
    {
        return armFailReasonStrs[vehicleState.armFailReason];
    }
    return "Unknown";
}

bool vehicleSetFlightMode(VehicleFlightMode mode)
{
    if (!isInit || mode >= FLIGHT_MODE_MAX)
        return false;

    xSemaphoreTake(stateMutex, portMAX_DELAY);

    VehicleFlightMode oldMode = vehicleState.flightMode;
    vehicleState.flightMode = mode;
    vehicleState.lastStateChange = xTaskGetTickCount();

    // 同步到旧的 FlightMode 系统
    switch (mode)
    {
    case FLIGHT_MODE_MANUAL:
    case FLIGHT_MODE_STABILIZE:
        setCommandermode(STABILIZE_MODE);
        break;
    case FLIGHT_MODE_ALTITUDE:
        setCommandermode(ALTHOLD_MODE);
        break;
    case FLIGHT_MODE_POSITION:
    case FLIGHT_MODE_OFFBOARD:
        setCommandermode(POSHOLD_MODE);
        break;
    default:
        // 其他模式暂时映射到自稳
        setCommandermode(STABILIZE_MODE);
        break;
    }

    xSemaphoreGive(stateMutex);

    if (oldMode != mode)
    {
        DEBUG_PRINT("Flight mode: %s -> %s\n",
                    vehicleGetFlightModeName(oldMode),
                    vehicleGetFlightModeName(mode));
    }

    return true;
}

void vehicleEmergencyStop(void)
{
    xSemaphoreTake(stateMutex, portMAX_DELAY);

    vehicleState.isEmergency = true;
    vehicleState.isArmed = false;
    vehicleState.armingState = ARMING_STATE_STANDBY_ERROR;
    vehicleState.lastStateChange = xTaskGetTickCount();

    // 触发旧系统的紧急停止
    stabilizerSetEmergencyStop();
    systemSetArmed(false);

    xSemaphoreGive(stateMutex);

    DEBUG_PRINT("!!! EMERGENCY STOP !!!\n");
}

void vehicleResetEmergency(void)
{
    xSemaphoreTake(stateMutex, portMAX_DELAY);

    vehicleState.isEmergency = false;
    vehicleState.armingState = ARMING_STATE_STANDBY;

    // 重置旧系统的紧急停止
    stabilizerResetEmergencyStop();

    xSemaphoreGive(stateMutex);

    DEBUG_PRINT("Emergency stop reset\n");
}

void vehicleStateUpdate(void)
{
    if (!isInit)
        return;

    // 检查传感器状态，更新初始化完成
    if (vehicleState.armingState == ARMING_STATE_INIT)
    {
        if (sensorsAreCalibrated())
        {
            vehicleState.armingState = ARMING_STATE_STANDBY;
            vehicleState.isSensorHealthy = true;
            DEBUG_PRINT("Vehicle ready: STANDBY\n");
        }
    }

    // 更新飞行阶段
    updateFlightPhase();

    // 更新故障安全状态
    updateFailsafe();

    // 同步日志变量
    logArmingState = (uint8_t)vehicleState.armingState;
    logFlightMode = (uint8_t)vehicleState.flightMode;
    logFlightPhase = (uint8_t)vehicleState.flightPhase;
    logFailsafe = (uint8_t)vehicleState.failsafeState;
    logIsArmed = vehicleState.isArmed ? 1 : 0;
    logIsFlying = vehicleState.isFlying ? 1 : 0;
}

void vehicleSetRcConnected(bool connected)
{
    vehicleState.isRcConnected = connected;
}

void vehicleSetGcsConnected(bool connected)
{
    vehicleState.isGcsConnected = connected;
}

const char *vehicleGetArmingStateName(ArmingState state)
{
    if (state < ARMING_STATE_MAX)
    {
        return armingStateNames[state];
    }
    return "UNKNOWN";
}

const char *vehicleGetFlightModeName(VehicleFlightMode mode)
{
    if (mode < FLIGHT_MODE_MAX)
    {
        return flightModeNames[mode];
    }
    return "UNKNOWN";
}

const char *vehicleGetFlightPhaseName(FlightPhase phase)
{
    if (phase < FLIGHT_PHASE_MAX)
    {
        return flightPhaseNames[phase];
    }
    return "UNKNOWN";
}

/*===========================================================================
 * 私有函数实现
 *===========================================================================*/

/**
 * @brief 运行解锁前检查
 */
static bool runArmingChecks(bool force)
{
    vehicleState.armingCheckResult = 0;

    // 1. 传感器检查
    if ((vehicleState.armingCheckFlags & ARM_CHECK_SENSORS) && !force)
    {
        if (!sensorsAreCalibrated())
        {
            vehicleState.armFailReason = ARM_FAIL_SENSORS_CALIBRATING;
            vehicleState.armingCheckResult |= ARM_CHECK_SENSORS;
            return false;
        }
    }

    // 2. 电池检查
    if ((vehicleState.armingCheckFlags & ARM_CHECK_BATTERY) && !force)
    {
        float voltage = pmGetBatteryVoltage();
        if (voltage < (BATTERY_LOW_THRESHOLD_MV / 1000.0f))
        {
            vehicleState.armFailReason = ARM_FAIL_BATTERY_LOW;
            vehicleState.armingCheckResult |= ARM_CHECK_BATTERY;
            vehicleState.isBatteryLow = true;
            return false;
        }
    }

    // 3. 油门归零检查（需要从commander获取当前油门）
    // 这里简化处理，实际应该检查遥控器油门位置
    // if ((vehicleState.armingCheckFlags & ARM_CHECK_THROTTLE_ZERO) && !force) {
    //     // TODO: 检查油门是否归零
    // }

    return true;
}

/**
 * @brief 更新飞行阶段（基于传感器数据）
 */
static void updateFlightPhase(void)
{
    static uint32_t landedStartTime = 0;

    if (!vehicleState.isArmed)
    {
        vehicleState.flightPhase = FLIGHT_PHASE_ON_GROUND;
        vehicleState.isFlying = false;
        return;
    }

    // 简化的飞行阶段检测
    // 实际应该基于加速度、速度、高度等综合判断
    switch (vehicleState.flightPhase)
    {
    case FLIGHT_PHASE_ON_GROUND:
        // 检测起飞：油门超过阈值
        // TODO: 使用实际的油门/高度判断
        vehicleState.isFlying = false;
        break;

    case FLIGHT_PHASE_TAKEOFF:
        // 检测是否完成起飞
        vehicleState.isFlying = true;
        vehicleState.flightPhase = FLIGHT_PHASE_IN_AIR;
        break;

    case FLIGHT_PHASE_IN_AIR:
        vehicleState.isFlying = true;
        // 检测是否开始降落
        if (vehicleState.flightMode == FLIGHT_MODE_LAND)
        {
            vehicleState.flightPhase = FLIGHT_PHASE_LANDING;
        }
        break;

    case FLIGHT_PHASE_LANDING:
        // 检测是否已降落（速度接近零且持续一段时间）
        break;

    case FLIGHT_PHASE_LANDED:
        vehicleState.isFlying = false;
        break;

    default:
        break;
    }
}

/**
 * @brief 更新故障安全状态
 */
static void updateFailsafe(void)
{
    // 检查电池
    float voltage = pmGetBatteryVoltage();
    if (voltage < (BATTERY_CRITICAL_THRESHOLD_MV / 1000.0f))
    {
        vehicleState.failsafeState = FAILSAFE_CRITICAL_BATTERY;
        vehicleState.isBatteryLow = true;
    }
    else if (voltage < (BATTERY_LOW_THRESHOLD_MV / 1000.0f))
    {
        if (vehicleState.failsafeState < FAILSAFE_LOW_BATTERY)
        {
            vehicleState.failsafeState = FAILSAFE_LOW_BATTERY;
        }
        vehicleState.isBatteryLow = true;
    }
    else
    {
        vehicleState.isBatteryLow = false;
        if (vehicleState.failsafeState == FAILSAFE_LOW_BATTERY ||
            vehicleState.failsafeState == FAILSAFE_CRITICAL_BATTERY)
        {
            vehicleState.failsafeState = FAILSAFE_NONE;
        }
    }

    // TODO: 检查遥控器信号
    // TODO: 检查地面站连接
}

/*===========================================================================
 * 日志和参数
 *===========================================================================*/

LOG_GROUP_START(vehicle)
LOG_ADD(LOG_UINT8, armState, &logArmingState)
LOG_ADD(LOG_UINT8, flightMode, &logFlightMode)
LOG_ADD(LOG_UINT8, flightPhase, &logFlightPhase)
LOG_ADD(LOG_UINT8, failsafe, &logFailsafe)
LOG_ADD(LOG_UINT8, isArmed, &logIsArmed)
LOG_ADD(LOG_UINT8, isFlying, &logIsFlying)
LOG_GROUP_STOP(vehicle)

PARAM_GROUP_START(vehicle)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, armState, &logArmingState)
PARAM_ADD(PARAM_UINT8, flightMode, &logFlightMode)
PARAM_GROUP_STOP(vehicle)
