/**
 * @file vehicle_state.h
 * @brief 飞行器状态管理模块 - PX4/QGC 风格状态机
 *
 * 本模块实现类似 PX4 的飞行器状态管理：
 * - 解锁状态 (Arming State)
 * - 飞行模式 (Flight Mode)
 * - 飞行阶段 (Flight Phase)
 * - 故障检测 (Failsafe)
 *
 * 状态转换遵循安全规则，防止非法状态切换
 */

#ifndef __VEHICLE_STATE_H__
#define __VEHICLE_STATE_H__

#include <stdint.h>
#include <stdbool.h>

/*===========================================================================
 * 解锁状态枚举 (类似 PX4 arming_state_t)
 *===========================================================================*/
typedef enum
{
    ARMING_STATE_INIT = 0,       // 初始化中
    ARMING_STATE_STANDBY,        // 待机（已上锁，可解锁）
    ARMING_STATE_ARMED,          // 已解锁（电机可运转）
    ARMING_STATE_STANDBY_ERROR,  // 待机错误（传感器故障等）
    ARMING_STATE_SHUTDOWN,       // 关机状态
    ARMING_STATE_IN_AIR_RESTORE, // 空中恢复（紧急情况）
    ARMING_STATE_MAX
} ArmingState;

/*===========================================================================
 * 飞行模式枚举 (类似 PX4 main_state_t / QGC flight_mode)
 *===========================================================================*/
typedef enum
{
    FLIGHT_MODE_MANUAL = 0, // 手动模式（直接油门控制）
    FLIGHT_MODE_STABILIZE,  // 自稳模式（姿态控制）
    FLIGHT_MODE_ALTITUDE,   // 定高模式
    FLIGHT_MODE_POSITION,   // 定点模式
    FLIGHT_MODE_MISSION,    // 任务模式（预留）
    FLIGHT_MODE_RTL,        // 返航模式（预留）
    FLIGHT_MODE_LAND,       // 降落模式
    FLIGHT_MODE_TAKEOFF,    // 起飞模式
    FLIGHT_MODE_OFFBOARD,   // 外部控制模式（遥测控制）
    FLIGHT_MODE_MAX
} VehicleFlightMode;

/*===========================================================================
 * 飞行阶段枚举 (类似 PX4 vehicle_status)
 *===========================================================================*/
typedef enum
{
    FLIGHT_PHASE_ON_GROUND = 0, // 地面待机
    FLIGHT_PHASE_TAKEOFF,       // 起飞中
    FLIGHT_PHASE_IN_AIR,        // 空中飞行
    FLIGHT_PHASE_LANDING,       // 降落中
    FLIGHT_PHASE_LANDED,        // 已降落
    FLIGHT_PHASE_MAX
} FlightPhase;

/*===========================================================================
 * 故障安全状态枚举
 *===========================================================================*/
typedef enum
{
    FAILSAFE_NONE = 0,         // 无故障
    FAILSAFE_LOW_BATTERY,      // 低电量
    FAILSAFE_CRITICAL_BATTERY, // 电量严重不足
    FAILSAFE_RC_LOSS,          // 遥控信号丢失
    FAILSAFE_GCS_LOSS,         // 地面站信号丢失
    FAILSAFE_SENSOR_FAILURE,   // 传感器故障
    FAILSAFE_GEOFENCE,         // 越界（预留）
    FAILSAFE_MAX
} FailsafeState;

/*===========================================================================
 * 解锁检查标志位
 *===========================================================================*/
typedef enum
{
    ARM_CHECK_NONE = 0,
    ARM_CHECK_SENSORS = (1 << 0),       // 传感器检查
    ARM_CHECK_RC = (1 << 1),            // 遥控器检查
    ARM_CHECK_BATTERY = (1 << 2),       // 电池检查
    ARM_CHECK_SAFETY_SWITCH = (1 << 3), // 安全开关检查（预留）
    ARM_CHECK_THROTTLE_ZERO = (1 << 4), // 油门归零检查
    ARM_CHECK_ATTITUDE = (1 << 5),      // 姿态检查（水平放置）
    ARM_CHECK_ALL = 0xFF                // 所有检查
} ArmingCheckFlags;

/*===========================================================================
 * 解锁失败原因
 *===========================================================================*/
typedef enum
{
    ARM_FAIL_NONE = 0,            // 无失败
    ARM_FAIL_SENSORS_NOT_READY,   // 传感器未就绪
    ARM_FAIL_SENSORS_CALIBRATING, // 传感器校准中
    ARM_FAIL_RC_NOT_CONNECTED,    // 遥控器未连接
    ARM_FAIL_BATTERY_LOW,         // 电池电量低
    ARM_FAIL_THROTTLE_NOT_ZERO,   // 油门未归零
    ARM_FAIL_ATTITUDE_NOT_LEVEL,  // 姿态不水平
    ARM_FAIL_ALREADY_ARMED,       // 已经解锁
    ARM_FAIL_SYSTEM_ERROR,        // 系统错误
    ARM_FAIL_EMERGENCY_STOP,      // 紧急停止激活
    ARM_FAIL_MAX
} ArmingFailReason;

/*===========================================================================
 * 飞行器完整状态结构
 *===========================================================================*/
typedef struct
{
    // 主要状态
    ArmingState armingState;      // 解锁状态
    VehicleFlightMode flightMode; // 飞行模式
    FlightPhase flightPhase;      // 飞行阶段
    FailsafeState failsafeState;  // 故障安全状态

    // 解锁相关
    ArmingFailReason armFailReason; // 最近一次解锁失败原因
    uint32_t armingCheckFlags;      // 启用的解锁检查标志
    uint32_t armingCheckResult;     // 解锁检查结果（失败的检查项）

    // 状态标志
    bool isArmed;         // 是否已解锁
    bool isFlying;        // 是否在飞行
    bool isEmergency;     // 是否紧急状态
    bool isRcConnected;   // 遥控器是否连接
    bool isGcsConnected;  // 地面站是否连接
    bool isBatteryLow;    // 电池是否低电量
    bool isSensorHealthy; // 传感器是否健康

    // 时间戳
    uint32_t armTime;         // 解锁时间
    uint32_t flightTime;      // 飞行时间
    uint32_t lastStateChange; // 最后状态变化时间

} VehicleState;

/*===========================================================================
 * API 函数
 *===========================================================================*/

/**
 * @brief 初始化飞行器状态管理模块
 */
void vehicleStateInit(void);

/**
 * @brief 获取当前飞行器状态（只读副本）
 */
VehicleState vehicleStateGet(void);

/**
 * @brief 获取当前解锁状态
 */
ArmingState vehicleGetArmingState(void);

/**
 * @brief 获取当前飞行模式
 */
VehicleFlightMode vehicleGetFlightMode(void);

/**
 * @brief 获取当前飞行阶段
 */
FlightPhase vehicleGetFlightPhase(void);

/**
 * @brief 检查是否已解锁
 */
bool vehicleIsArmed(void);

/**
 * @brief 检查是否在飞行中
 */
bool vehicleIsFlying(void);

/**
 * @brief 尝试解锁
 * @param force 强制解锁（跳过部分检查，仅用于紧急情况）
 * @return 成功返回 true，失败返回 false
 */
bool vehicleArm(bool force);

/**
 * @brief 上锁（锁定电机）
 * @param force 强制上锁（即使在飞行中）
 * @return 成功返回 true
 */
bool vehicleDisarm(bool force);

/**
 * @brief 获取解锁失败原因
 */
ArmingFailReason vehicleGetArmFailReason(void);

/**
 * @brief 获取解锁失败原因的文字描述
 */
const char *vehicleGetArmFailReasonStr(void);

/**
 * @brief 设置飞行模式
 * @param mode 目标飞行模式
 * @return 成功返回 true
 */
bool vehicleSetFlightMode(VehicleFlightMode mode);

/**
 * @brief 触发紧急停止
 */
void vehicleEmergencyStop(void);

/**
 * @brief 重置紧急停止状态
 */
void vehicleResetEmergency(void);

/**
 * @brief 更新飞行器状态（由 stabilizer 周期调用）
 * 检测起飞/降落/飞行状态变化
 */
void vehicleStateUpdate(void);

/**
 * @brief 设置遥控器连接状态
 */
void vehicleSetRcConnected(bool connected);

/**
 * @brief 设置地面站连接状态
 */
void vehicleSetGcsConnected(bool connected);

/**
 * @brief 获取解锁状态名称
 */
const char *vehicleGetArmingStateName(ArmingState state);

/**
 * @brief 获取飞行模式名称
 */
const char *vehicleGetFlightModeName(VehicleFlightMode mode);

/**
 * @brief 获取飞行阶段名称
 */
const char *vehicleGetFlightPhaseName(FlightPhase phase);

#endif /* __VEHICLE_STATE_H__ */
