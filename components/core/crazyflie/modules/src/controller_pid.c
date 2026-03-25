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
#define RATE_UPDATE_DT (float)(1.0f / RATE_MAIN_LOOP)

// Treat very small thrust as zero-throttle and fully reset the controller.
#define THRUST_DEAD_ZONE 3000
#define RATE_DESIRED_LIMIT 100.0f

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
static bool pitchRateSignWarned = false;

void controllerPidInit(void)
{
  attitudeControllerInit(RATE_UPDATE_DT, ATTITUDE_UPDATE_DT);
  positionControllerInit();

  // The Kalman externalized attitude uses the legacy Crazyflie pitch sign
  // convention (.pitch = -pitch * RAD_TO_DEG). Keep the rate loop in the same
  // convention by inverting gyro.y locally in the PID controller path.
  if (!pitchRateSignWarned)
  {
    DEBUG_PRINT("WARNING: PID pitch-rate sign compensation enabled to match attitude pitch convention\n");
    pitchRateSignWarned = true;
  }
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
  // Keep pitch rate in the same sign convention as state->attitude.pitch.
  const float pitchRateActual = -sensors->gyro.y;

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick))
  {
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

  if (setpoint->mode.z == modeDisable)
  {
    actuatorThrust = setpoint->thrust;
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick))
  {
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable)
    {
      attitudeDesired.roll = setpoint->attitude.roll;
      attitudeDesired.pitch = setpoint->attitude.pitch;
    }

    attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                         attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                         &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

    if (rateDesired.roll > RATE_DESIRED_LIMIT)
      rateDesired.roll = RATE_DESIRED_LIMIT;
    else if (rateDesired.roll < -RATE_DESIRED_LIMIT)
      rateDesired.roll = -RATE_DESIRED_LIMIT;

    if (rateDesired.pitch > RATE_DESIRED_LIMIT)
      rateDesired.pitch = RATE_DESIRED_LIMIT;
    else if (rateDesired.pitch < -RATE_DESIRED_LIMIT)
      rateDesired.pitch = -RATE_DESIRED_LIMIT;

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
  }

  // Run the inner rate loop at the main stabilizer rate. The attitude loop
  // refreshes rateDesired at ATTITUDE_RATE and the rate loop reuses the latest
  // setpoint in between updates.
  attitudeControllerCorrectRatePID(sensors->gyro.x, pitchRateActual, sensors->gyro.z,
                                   rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

  attitudeControllerGetActuatorOutput(&control->roll,
                                      &control->pitch,
                                      &control->yaw);

  // The X mixer layout uses the opposite yaw sign compared to the original
  // Crazyflie "+" reference frame.
  control->yaw = -control->yaw;

  {
    float thr = actuatorThrust;
    if (thr > THRUST_DEAD_ZONE)
    {
      int32_t rpLim32 = (int32_t)(thr * 0.6f);
      int32_t yLim32 = (int32_t)(thr * 0.2f);
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
             sensors->gyro.x, pitchRateActual,
             (int)control->roll, (int)control->pitch, (int)control->yaw);
    }
  }

  cmd_thrust = control->thrust;
  cmd_roll = control->roll;
  cmd_pitch = control->pitch;
  cmd_yaw = control->yaw;
  r_roll = radians(sensors->gyro.x);
  r_pitch = radians(pitchRateActual);
  r_yaw = radians(sensors->gyro.z);
  accelz = sensors->acc.z;

  if (tiltCompensationEnabled)
  {
    control->thrust = actuatorThrust / sensfusion6GetInvThrustCompensationForTilt();
  }
  else
  {
    control->thrust = actuatorThrust;
  }

  if (control->thrust >= THRUST_DEAD_ZONE && attitudeControllerAllPidGainsAreZero())
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

    static uint32_t pidZeroWarnCount = 0;
    if (++pidZeroWarnCount % 500 == 0)
    {
      DEBUG_PRINT("Safety interlock: all PID gains are zero, thrust output blocked\n");
    }
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
