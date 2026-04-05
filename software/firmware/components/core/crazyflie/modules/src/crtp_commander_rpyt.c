/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2017 Bitcraze AB
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
#include <math.h>
#include <stdbool.h>

#include "crtp_commander.h"
#include "commander.h"
#include "crtp.h"
#include "param.h"
#include "FreeRTOS.h"
#include "num.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "MODE"
#include "debug_cf.h"

#define MIN_THRUST 1000
#define MAX_THRUST 60000

/**
 * CRTP commander rpyt packet format
 */
struct CommanderCrtpLegacyValues
{
  float roll;  // deg
  float pitch; // deg
  float yaw;   // deg
  uint16_t thrust;
} __attribute__((packed));

/**
 * Stabilization modes for Roll, Pitch, Yaw.
 */
typedef enum
{
  RATE = 0,
  ANGLE = 1,
} RPYType;

/**
 * Yaw flight Modes
 */
typedef enum
{
  CAREFREE = 0, // Yaw is locked to world coordinates thus heading stays the same when yaw rotates
  PLUSMODE = 1, // Plus-mode. Motor M1 is defined as front
  XMODE = 2,    // X-mode. M1 & M4 are defined as front
} YawModeType;

static RPYType stabilizationModeRoll = ANGLE;  // Current stabilization type of roll (rate or angle)
static RPYType stabilizationModePitch = ANGLE; // Current stabilization type of pitch (rate or angle)
static RPYType stabilizationModeYaw = RATE;    // Current stabilization type of yaw (rate or angle)

static YawModeType yawMode = DEFAULT_YAW_MODE; // Yaw mode configuration
static bool carefreeResetFront;                // Reset what is front in carefree mode

static bool thrustLocked = true;

/**
 * Rotate Yaw so that the Crazyflie will change what is considered front.
 *
 * @param yawRad Amount of radians to rotate yaw.
 */
static void rotateYaw(setpoint_t *setpoint, float yawRad)
{
  float cosy = cosf(yawRad);
  float siny = sinf(yawRad);
  float originalRoll = setpoint->attitude.roll;
  float originalPitch = setpoint->attitude.pitch;

  setpoint->attitude.roll = originalRoll * cosy - originalPitch * siny;
  setpoint->attitude.pitch = originalPitch * cosy + originalRoll * siny;
}

/**
 * Update Yaw according to current setting
 */
static void yawModeUpdate(setpoint_t *setpoint)
{
  switch (yawMode)
  {
  case CAREFREE:
    // TODO: Add frame of reference to setpoint
    ASSERT(false);
    break;
  case PLUSMODE:
    rotateYaw(setpoint, 45 * M_PI / 180);
    break;
  case XMODE: // Fall through
  default:
    // Default in x-mode. Do nothing
    break;
  }
}

void crtpCommanderRpytDecodeSetpoint(setpoint_t *setpoint, CRTPPacket *pk)
{
  struct CommanderCrtpLegacyValues *values = (struct CommanderCrtpLegacyValues *)pk->data;

  if (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE)
  {
    thrustLocked = true;
  }
  if (values->thrust == 0)
  {
    thrustLocked = false;
  }

  // Thrust
  uint16_t rawThrust = values->thrust;

  if (thrustLocked || (rawThrust < MIN_THRUST))
  {
    setpoint->thrust = 0;
  }
  else
  {
    setpoint->thrust = fminf(rawThrust, MAX_THRUST);
  }

  // Roll/Pitch (CRTP 支持 RATE/ANGLE 切换)
  if (stabilizationModeRoll == RATE)
  {
    setpoint->mode.roll = modeVelocity;
    setpoint->attitudeRate.roll = values->roll;
    setpoint->attitude.roll = 0;
  }
  else
  {
    setpoint->mode.roll = modeAbs;
    setpoint->attitudeRate.roll = 0;
    setpoint->attitude.roll = values->roll;
  }

  if (stabilizationModePitch == RATE)
  {
    setpoint->mode.pitch = modeVelocity;
    setpoint->attitudeRate.pitch = values->pitch;
    setpoint->attitude.pitch = 0;
  }
  else
  {
    setpoint->mode.pitch = modeAbs;
    setpoint->attitudeRate.pitch = 0;
    setpoint->attitude.pitch = values->pitch;
  }

  // Yaw
  if (stabilizationModeYaw == RATE)
  {
    // legacy rate input is inverted
    setpoint->attitudeRate.yaw = -values->yaw;
    yawModeUpdate(setpoint);
    setpoint->mode.yaw = modeVelocity;
  }
  else
  {
    setpoint->mode.yaw = modeAbs;
    setpoint->attitudeRate.yaw = 0;
    setpoint->attitude.yaw = values->yaw;
  }

  // 统一飞行模式 → setpoint 语义（定高/定点模式会覆写 z/x/y mode）
  commanderApplyFlightMode(setpoint);
}

// Params for stabilization modes (flight mode now managed by vehicle_state)
PARAM_GROUP_START(flightmode)
PARAM_ADD(PARAM_UINT8, yawMode, &yawMode)
PARAM_ADD(PARAM_UINT8, yawRst, &carefreeResetFront)
PARAM_ADD(PARAM_UINT8, stabModeRoll, &stabilizationModeRoll)
PARAM_ADD(PARAM_UINT8, stabModePitch, &stabilizationModePitch)
PARAM_ADD(PARAM_UINT8, stabModeYaw, &stabilizationModeYaw)
PARAM_GROUP_STOP(flightmode)
