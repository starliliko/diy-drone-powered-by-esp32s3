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

#include <math.h>
#include "sensors.h"
#include "stabilizer.h"
#include "commander.h"
#include "pm_esplane.h"
#include "log.h"
#include "param.h"

/* 估计器高度 / 速度日志变量 ID（用于 flightPhase 物理确认） */
static logVarId_t estZId;
static logVarId_t estVzId;
static bool estLogIdsReady = false;

#define DEBUG_MODULE "VEHICLE"
#include "debug_cf.h"

/*===========================================================================
 * 配置参数
 *===========================================================================*/

// 解锁检查配置
// 注意: 禁用电池检查用于开发测试，生产环境应恢复 ARM_CHECK_BATTERY
// #define ARM_CHECK_ENABLED (ARM_CHECK_SENSORS | ARM_CHECK_BATTERY | ARM_CHECK_THROTTLE_ZERO)
#define ARM_CHECK_ENABLED (ARM_CHECK_SENSORS | ARM_CHECK_THROTTLE_ZERO) // 临时禁用电池检查
#define BATTERY_LOW_THRESHOLD_MV 3300                                   // 单节电池低电量阈值 (mV)
#define BATTERY_CRITICAL_THRESHOLD_MV 3100                              // 单节电池严重低电量阈值 (mV)
#define ATTITUDE_LEVEL_THRESHOLD_DEG 10.0f                              // 水平放置检查阈值（度）
#define TAKEOFF_THRUST_THRESHOLD 10000                                  // 起飞油门阈值
#define LANDED_VELOCITY_THRESHOLD 0.1f                                  // 降落速度阈值 (m/s)
#define LANDED_TIME_THRESHOLD_MS 1000                                   // 降落确认时间

// 估计器物理确认阈值
#define EST_AIRBORNE_HEIGHT_M     0.10f   // 估计高度 > 此值确认离地
#define EST_LANDED_HEIGHT_M       0.08f   // 估计高度 < 此值认为落地
#define EST_LANDED_VELOCITY_M_S   0.15f   // |垂直速度| < 此值认为静止

/*===========================================================================
 * 状态变量
 *===========================================================================*/

#define TAKEOFF_VELOCITY_THRESHOLD 0.15f                                // Vertical takeoff command threshold (m/s)
#define TAKEOFF_ALTITUDE_THRESHOLD 0.15f                                // Absolute-height takeoff threshold (m)
#define LANDING_THRUST_THRESHOLD 2000                                   // Manual landing low-thrust threshold

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
static bool getCurrentSetpoint(setpoint_t *setpoint);
static bool isTakeoffCommandActive(const setpoint_t *setpoint);
static bool isLandingCommandActive(const setpoint_t *setpoint);
static void getEstimatorHeightVelocity(float *height, float *vz);

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

    // 气压计已在启动时自动校准，不再需要手动校准

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

    bool becameReady = false;

    xSemaphoreTake(stateMutex, portMAX_DELAY);

    // 检查传感器状态，更新初始化完成
    if (vehicleState.armingState == ARMING_STATE_INIT)
    {
        if (sensorsAreCalibrated())
        {
            vehicleState.armingState = ARMING_STATE_STANDBY;
            vehicleState.isSensorHealthy = true;
            becameReady = true;
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

    xSemaphoreGive(stateMutex);

    if (becameReady)
    {
        DEBUG_PRINT("Vehicle ready: STANDBY\n");
    }
}

void vehicleSetRcConnected(bool connected)
{
    if (stateMutex && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        vehicleState.isRcConnected = connected;
        xSemaphoreGive(stateMutex);
    }
    else
    {
        vehicleState.isRcConnected = connected;
    }
}

void vehicleSetGcsConnected(bool connected)
{
    if (stateMutex && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        vehicleState.isGcsConnected = connected;
        xSemaphoreGive(stateMutex);
    }
    else
    {
        vehicleState.isGcsConnected = connected;
    }
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
static bool getCurrentSetpoint(setpoint_t *setpoint)
{
    if (setpoint == NULL)
    {
        return false;
    }

    commanderGetCurrentSetpoint(setpoint);
    return true;
}

static bool isTakeoffCommandActive(const setpoint_t *setpoint)
{
    if (setpoint == NULL)
    {
        return false;
    }

    if (setpoint->mode.z == modeVelocity)
    {
        return setpoint->velocity.z > TAKEOFF_VELOCITY_THRESHOLD;
    }

    if (setpoint->mode.z == modeAbs)
    {
        return setpoint->position.z > TAKEOFF_ALTITUDE_THRESHOLD;
    }

    return setpoint->thrust > TAKEOFF_THRUST_THRESHOLD;
}

static bool isLandingCommandActive(const setpoint_t *setpoint)
{
    if (setpoint == NULL)
    {
        return false;
    }

    if (vehicleState.flightMode == FLIGHT_MODE_LAND)
    {
        return true;
    }

    if (setpoint->mode.z == modeVelocity)
    {
        return setpoint->velocity.z < -TAKEOFF_VELOCITY_THRESHOLD;
    }

    if (setpoint->mode.z == modeAbs)
    {
        return setpoint->position.z <= TAKEOFF_ALTITUDE_THRESHOLD;
    }

    return setpoint->thrust < LANDING_THRUST_THRESHOLD;
}

/**
 * @brief 从估计器读取当前高度和垂直速度（通过 LOG 系统）
 *
 * 延迟初始化 log 变量 ID，第一次调用时查找 stateEstimate.z / .vz。
 * 若 log 系统尚未就绪（ID 无效），返回 0.0f 回退到纯意图模式。
 */
static void getEstimatorHeightVelocity(float *height, float *vz)
{
    if (height == NULL || vz == NULL)
    {
        return;
    }

    *height = 0.0f;
    *vz = 0.0f;

    if (!estLogIdsReady || !LOG_VARID_IS_VALID(estZId) || !LOG_VARID_IS_VALID(estVzId))
    {
        estZId  = logGetVarId("stateEstimate", "z");
        estVzId = logGetVarId("stateEstimate", "vz");
        estLogIdsReady = LOG_VARID_IS_VALID(estZId) && LOG_VARID_IS_VALID(estVzId);
    }

    if (!LOG_VARID_IS_VALID(estZId) || !LOG_VARID_IS_VALID(estVzId))
    {
        return;
    }

    *height = logGetFloat(estZId);
    *vz     = logGetFloat(estVzId);
}

static bool runArmingChecks(bool force)
{
    setpoint_t currentSetpoint = {0};
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
    // 当前项目未接入可靠电量测量，避免因无效电压值误判导致无法解锁。
    // 统一视为电池状态正常。
    (void)force;
    vehicleState.isBatteryLow = false;

    // 3. 油门归零检查
    if ((vehicleState.armingCheckFlags & ARM_CHECK_THROTTLE_ZERO) && !force)
    {
        getCurrentSetpoint(&currentSetpoint);
        if (isTakeoffCommandActive(&currentSetpoint))
        {
            vehicleState.armFailReason = ARM_FAIL_THROTTLE_NOT_ZERO;
            vehicleState.armingCheckResult |= ARM_CHECK_THROTTLE_ZERO;
            return false;
        }
    }

    return true;
}

/**
 * @brief 更新飞行阶段（控制意图 + 估计器物理确认）
 *
 * 状态转换逻辑：
 *   ON_GROUND → TAKEOFF : setpoint 表达起飞意图
 *   TAKEOFF   → IN_AIR  : 估计器确认高度 > EST_AIRBORNE_HEIGHT_M
 *                          （若连续 2 秒无确认则强制转入，兜底 STABILIZE 模式等无高度估计场景）
 *   IN_AIR    → LANDING : setpoint 表达降落意图
 *   LANDING   → ON_GROUND : 时间阈值 + 估计器确认高度 & 速度均近零
 */
static void updateFlightPhase(void)
{
    static uint32_t landingStartTime = 0;
    static uint32_t takeoffStartTime = 0;
    setpoint_t currentSetpoint = {0};
    const uint32_t now = xTaskGetTickCount();

    if (!vehicleState.isArmed)
    {
        vehicleState.flightPhase = FLIGHT_PHASE_ON_GROUND;
        vehicleState.isFlying = false;
        landingStartTime = 0;
        takeoffStartTime = 0;
        return;
    }

    // 读取控制意图
    getCurrentSetpoint(&currentSetpoint);
    const bool takeoffCommandActive = isTakeoffCommandActive(&currentSetpoint);
    const bool landingCommandActive = isLandingCommandActive(&currentSetpoint);

    // 读取估计器物理状态
    float estHeight = 0.0f, estVz = 0.0f;
    getEstimatorHeightVelocity(&estHeight, &estVz);

    switch (vehicleState.flightPhase)
    {
    case FLIGHT_PHASE_ON_GROUND:
        vehicleState.isFlying = false;
        landingStartTime = 0;
        takeoffStartTime = 0;
        if (takeoffCommandActive)
        {
            vehicleState.flightPhase = FLIGHT_PHASE_TAKEOFF;
            takeoffStartTime = now;
            DEBUG_PRINT("Flight phase: ON_GROUND -> TAKEOFF\n");
        }
        break;

    case FLIGHT_PHASE_TAKEOFF:
    {
        vehicleState.isFlying = true;
        landingStartTime = 0;

        // 用估计器确认离地；2 秒无确认则强制转入（兜底）
        bool heightConfirmed = (estHeight > EST_AIRBORNE_HEIGHT_M);
        bool timeout = (takeoffStartTime > 0) &&
                       ((now - takeoffStartTime) >= pdMS_TO_TICKS(2000));

        if (heightConfirmed || timeout)
        {
            vehicleState.flightPhase = FLIGHT_PHASE_IN_AIR;
            takeoffStartTime = 0;
            DEBUG_PRINT("Flight phase: TAKEOFF -> IN_AIR (h=%.2f %s)\n",
                        (double)estHeight, heightConfirmed ? "confirmed" : "timeout");
        }
        break;
    }

    case FLIGHT_PHASE_IN_AIR:
        vehicleState.isFlying = true;
        takeoffStartTime = 0;
        if (landingCommandActive)
        {
            vehicleState.flightPhase = FLIGHT_PHASE_LANDING;
            landingStartTime = now;
            DEBUG_PRINT("Flight phase: IN_AIR -> LANDING\n");
        }
        break;

    case FLIGHT_PHASE_LANDING:
        vehicleState.isFlying = true;
        takeoffStartTime = 0;
        if (takeoffCommandActive)
        {
            vehicleState.flightPhase = FLIGHT_PHASE_IN_AIR;
            landingStartTime = 0;
            DEBUG_PRINT("Flight phase: LANDING -> IN_AIR\n");
        }
        else
        {
            if (landingStartTime == 0)
            {
                landingStartTime = now;
            }

            // 时间阈值 + 估计器物理确认（高度近零 & 速度近零）
            bool timeElapsed = (now - landingStartTime) >= pdMS_TO_TICKS(LANDED_TIME_THRESHOLD_MS);
            bool physicallyLanded = (estHeight < EST_LANDED_HEIGHT_M) &&
                                   (fabsf(estVz) < EST_LANDED_VELOCITY_M_S);

            if (timeElapsed && physicallyLanded)
            {
                vehicleState.flightPhase = FLIGHT_PHASE_ON_GROUND;
                vehicleState.isFlying = false;
                landingStartTime = 0;
                DEBUG_PRINT("Flight phase: LANDING -> ON_GROUND (h=%.2f vz=%.2f)\n",
                            (double)estHeight, (double)estVz);
            }
            // 兜底：超过 3 倍时间阈值仍未物理确认，强制落地
            else if ((now - landingStartTime) >= pdMS_TO_TICKS(LANDED_TIME_THRESHOLD_MS * 3))
            {
                vehicleState.flightPhase = FLIGHT_PHASE_ON_GROUND;
                vehicleState.isFlying = false;
                landingStartTime = 0;
                DEBUG_PRINT("Flight phase: LANDING -> ON_GROUND (timeout, h=%.2f vz=%.2f)\n",
                            (double)estHeight, (double)estVz);
            }
        }
        break;

    case FLIGHT_PHASE_LANDED:
        vehicleState.isFlying = false;
        vehicleState.flightPhase = FLIGHT_PHASE_ON_GROUND;
        landingStartTime = 0;
        takeoffStartTime = 0;
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
    // 当前项目未接入可靠电量测量：
    // 1) 不根据电压触发 LOW/CRITICAL_BATTERY failsafe
    // 2) BatteryLow 标志始终清零
    vehicleState.isBatteryLow = false;
    if (vehicleState.failsafeState == FAILSAFE_LOW_BATTERY ||
        vehicleState.failsafeState == FAILSAFE_CRITICAL_BATTERY)
    {
        vehicleState.failsafeState = FAILSAFE_NONE;
    }

    // 检查遥控器信号 - 如果已启用SBUS且信号丢失
    if (!vehicleState.isRcConnected && vehicleState.isArmed)
    {
        // 遥控器信号丢失，且飞机已解锁 - 触发RC_LOSS失控保护
        if (vehicleState.failsafeState < FAILSAFE_RC_LOSS)
        {
            vehicleState.failsafeState = FAILSAFE_RC_LOSS;
            DEBUG_PRINT("FAILSAFE: RC signal lost!\n");
        }
    }
    else if (vehicleState.isRcConnected &&
             vehicleState.failsafeState == FAILSAFE_RC_LOSS)
    {
        // 遥控器信号恢复
        vehicleState.failsafeState = FAILSAFE_NONE;
        DEBUG_PRINT("FAILSAFE: RC signal recovered\n");
    }

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
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, flightMode, &logFlightMode)
PARAM_GROUP_STOP(vehicle)
