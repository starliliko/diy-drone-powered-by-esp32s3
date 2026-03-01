
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "sensfusion6.h"
#include "position_controller.h"
#include "controller_pid.h"

#include "log.h"
#include "debug_cf.h"
#include "param.h"
#include "math3d.h"
#include <stdio.h>

#define ATTITUDE_UPDATE_DT (float)(1.0f / ATTITUDE_RATE)

// 油门死区阈值: 低于此值时视为零油门，清零所有PID输出
// 防止SBUS摇杆最低位微小偏移导致PID积分饱和
// SBUS最低位(CH2≈290)经转换后约为2091，需要覆盖此值
#define THRUST_DEAD_ZONE 3000

static bool tiltCompensationEnabled = false;

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

void controllerPidInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
}

bool controllerPidTest(void)
{
  bool pass = true;

  pass &= attitudeControllerTest();
  DEBUG_PRINTD("controller_Pid_Test = %d", pass);

  return pass;
}

static float capAngle(float angle)
{
  float result = angle;

  while (result > 180.0f)
  {
    result -= 360.0f;
  }

  while (result < -180.0f)
  {
    result += 360.0f;
  }

  return result;
}

void controllerPid(control_t *control, setpoint_t *setpoint,
                   const sensorData_t *sensors,
                   const state_t *state,
                   const uint32_t tick)
{
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick))
  {
    // Rate-controled YAW is moving YAW angle setpoint
    if (setpoint->mode.yaw == modeVelocity)
    {
      attitudeDesired.yaw += setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT;
    }
    else
    {
      attitudeDesired.yaw = setpoint->attitude.yaw;
    }

    attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
  }

  if (RATE_DO_EXECUTE(POSITION_RATE, tick))
  {
    positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick))
  {
    // Switch between manual and automatic position control
    if (setpoint->mode.z == modeDisable)
    {
      actuatorThrust = setpoint->thrust;
    }
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable)
    {
      attitudeDesired.roll = setpoint->attitude.roll;
      attitudeDesired.pitch = setpoint->attitude.pitch;
    }

    attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                         attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                         &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

// 外环输出(角速率需求)限幅: 防止大角度误差时内环完全饱和
// 内环饱和阈值 = rpLim/RATE_KP = 12044/250 ≈ 48°/s
// 限制rDes在±100°/s内, 保证内环大部分时间工作在线性区间
#define RATE_DESIRED_LIMIT 100.0f
    if (rateDesired.roll > RATE_DESIRED_LIMIT)
      rateDesired.roll = RATE_DESIRED_LIMIT;
    else if (rateDesired.roll < -RATE_DESIRED_LIMIT)
      rateDesired.roll = -RATE_DESIRED_LIMIT;
    if (rateDesired.pitch > RATE_DESIRED_LIMIT)
      rateDesired.pitch = RATE_DESIRED_LIMIT;
    else if (rateDesired.pitch < -RATE_DESIRED_LIMIT)
      rateDesired.pitch = -RATE_DESIRED_LIMIT;

    // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
    // value. Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later
    if (setpoint->mode.roll == modeVelocity)
    {
      rateDesired.roll = setpoint->attitudeRate.roll;
      attitudeControllerResetRollAttitudePID();
    }
    if (setpoint->mode.pitch == modeVelocity)
    {
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      attitudeControllerResetPitchAttitudePID();
    }

    // gyro 已在驱动层对齐坐标系（sensors_bmi088_spi_ms5611.c），此处直通无需取反。
    attitudeControllerCorrectRatePID(sensors->gyro.x, sensors->gyro.y, sensors->gyro.z,
                                     rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

    attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);

    // 符号链分析 — 基于 sensfusion6 欧拉角定义和四元数严格推导:
    //   四元数推导: nose up 绕Y轴旋转-θ → q=(cosθ/2, 0, -sinθ/2, 0)
    //     → gravX = 2*(qx*qz - qw*qy) = sinθ > 0
    //   pitch = asin(gravX): 鼻子上仰 → pitch为正; 鼻子下压 → pitch为负
    //   roll = atan2(gravY, gravZ): 右倾 → roll为正
    //
    // X构型 mixer (r=R/2, p=P/2): M1=T-r+p-Y, M2=T+r+p+Y, M3=T+r-p-Y, M4=T-r-p+Y
    //
    // Roll:  飞机右倾(roll>0) → PID输出负R → r<0
    //        → 右侧(M1,M4)加速 → 右侧上抬 → roll减小 → 正确负反馈 ✓
    //        → Roll: 不取反
    //
    // Pitch: 鼻子下压(pitch<0) → PID输出正P → p>0
    //        → 前方(M1,M2)加速 → 鼻子上仰 → pitch增大 → 正确负反馈 ✓
    //        → Pitch: 不取反
    //
    // Yaw:   "+"构型CW电机系数+Y, "X"构型CW电机系数-Y → 方向相反
    //        → Yaw: 需要取反
    // Roll: 不取反
    // Pitch: 不取反 (nose up = pitch正, 正p→前方加速→nose up→pitch增大→负反馈)
    control->yaw = -control->yaw; // Yaw: X构型与+构型CW电机系数相反, 需取反

    // 推力比例输出限幅:
    // 在mixer中: M = T ± R/2 ± P/2 ± Y
    // 为保证所有电机 > 0, 需要: |R/2| + |P/2| + |Y| < T
    // 限制: |R| ≤ 0.6T, |P| ≤ 0.6T, |Y| ≤ 0.2T
    // → 最差电机 ≥ T - 0.3T - 0.3T - 0.2T = 0.2T (至少20%推力)
    {
      float thr = actuatorThrust;
      if (thr > THRUST_DEAD_ZONE)
      {
        // 使用int32_t中间变量避免int16_t溢出 (thr*0.6在T>54612时超过32767)
        int32_t rpLim32 = (int32_t)(thr * 0.6f);
        int32_t yLim32 = (int32_t)(thr * 0.2f);
        // 限幅值本身不能超过INT16_MAX
        int16_t rpLim = (rpLim32 > INT16_MAX) ? INT16_MAX : (int16_t)rpLim32;
        int16_t yLim = (yLim32 > INT16_MAX) ? INT16_MAX : (int16_t)yLim32;
        if (control->roll > rpLim)
          control->roll = rpLim;
        else if (control->roll < -rpLim)
          control->roll = -rpLim;
        if (control->pitch > rpLim)
          control->pitch = rpLim;
        else if (control->pitch < -rpLim)
          control->pitch = -rpLim;
        if (control->yaw > yLim)
          control->yaw = yLim;
        else if (control->yaw < -yLim)
          control->yaw = -yLim;
      }
    }

    // 诊断日志：每500次（约500ms@1kHz）打印PID中间变量
    {
      static uint32_t pidDbgCnt = 0;
      if (control->thrust >= THRUST_DEAD_ZONE && ++pidDbgCnt % 500 == 0)
      {
        printf("[PID] att: R=%.1f P=%.1f | des: R=%.1f P=%.1f | "
               "rDes: R=%.1f P=%.1f | gyro: x=%.1f y=%.1f | "
               "out: R=%d P=%d Y=%d\n",
               state->attitude.roll, state->attitude.pitch,
               attitudeDesired.roll, attitudeDesired.pitch,
               rateDesired.roll, rateDesired.pitch,
               sensors->gyro.x, sensors->gyro.y,
               (int)control->roll, (int)control->pitch, (int)control->yaw);
      }
    }

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
    r_roll = radians(sensors->gyro.x);
    r_pitch = radians(sensors->gyro.y);
    r_yaw = radians(sensors->gyro.z);
    accelz = sensors->acc.z;
  }

  if (tiltCompensationEnabled)
  {
    control->thrust = actuatorThrust / sensfusion6GetInvThrustCompensationForTilt();
  }
  else
  {
    control->thrust = actuatorThrust;
  }

  if (control->thrust < THRUST_DEAD_ZONE)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    attitudeControllerResetAllPID();
    positionControllerResetAllPID();

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }
}

LOG_GROUP_START(controller)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
LOG_ADD(LOG_FLOAT, accelz, &accelz)
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
LOG_ADD(LOG_FLOAT, roll, &attitudeDesired.roll)
LOG_ADD(LOG_FLOAT, pitch, &attitudeDesired.pitch)
LOG_ADD(LOG_FLOAT, yaw, &attitudeDesired.yaw)
LOG_ADD(LOG_FLOAT, rollRate, &rateDesired.roll)
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
LOG_ADD(LOG_FLOAT, yawRate, &rateDesired.yaw)
LOG_GROUP_STOP(controller)

PARAM_GROUP_START(controller)
PARAM_ADD(PARAM_UINT8, tiltComp, &tiltCompensationEnabled)
PARAM_GROUP_STOP(controller)
