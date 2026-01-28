/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2013 Bitcraze AB
 * Copyright (C) 2024 ESP-Drone Project (SBUS support)
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
 * extrx.c - Module to handle external receiver inputs (SBUS protocol)
 */

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "config.h"
#include "system.h"
#include "commander.h"
#include "crtp_commander.h"
#include "vehicle_state.h"
#include "static_mem.h"
#include "sdkconfig.h"

#ifdef CONFIG_ENABLE_SBUS
#include "sbus.h"
#endif

#define DEBUG_MODULE "EXTRX"
#include "debug_cf.h"
#include "log.h"

/* Enable logging */
#define ENABLE_EXTRX_LOG

/* Number of channels to process (IA-10B supports 10 channels) */
#define EXTRX_NR_CHANNELS 10

/* Channel mapping - use Kconfig values or defaults */

#define EXTRX_CH_THRUST CONFIG_SBUS_CH_THROTTLE
#define EXTRX_CH_ROLL CONFIG_SBUS_CH_ROLL
#define EXTRX_CH_PITCH CONFIG_SBUS_CH_PITCH
#define EXTRX_CH_YAW CONFIG_SBUS_CH_YAW
#define EXTRX_CH_ARM CONFIG_SBUS_CH_ARM
#define EXTRX_CH_MODE CONFIG_SBUS_CH_MODE

/* Control direction signs (adjust if controls are reversed) */
#define EXTRX_SIGN_ROLL (-1)
#define EXTRX_SIGN_PITCH (-1)
#define EXTRX_SIGN_YAW (-1)

/* Control scaling (degrees for roll/pitch, degrees/second for yaw) */
#define EXTRX_SCALE_ROLL (20.0f)  // degrees
#define EXTRX_SCALE_PITCH (20.0f) // degrees
#define EXTRX_SCALE_YAW (150.0f)  // degrees/second

/* Arm switch threshold (SBUS value 0-2047, center ~992) */
#define EXTRX_ARM_THRESHOLD 1500

/* Mode switch thresholds for 3-position switch */
/* Low position: ~240-600, Mid position: ~600-1400, High position: ~1400-1800 */
#define EXTRX_MODE_LOW_THRESHOLD 600
#define EXTRX_MODE_HIGH_THRESHOLD 1400

/* Module state */
static setpoint_t extrxSetpoint;
static uint16_t ch[EXTRX_NR_CHANNELS];
static bool isInit = false;
static bool isArmed = false;
static FlightMode currentMode = STABILIZE_MODE; // 当前飞行模式
static FlightMode lastMode = STABILIZE_MODE;    // 上次飞行模式，用于检测变化

/* Private function declarations */
static void extRxTask(void *param);
static void extRxDecodeChannels(void);
static void extRxUpdateFlightMode(uint16_t modeChannel);

STATIC_MEM_TASK_ALLOC(extRxTask, EXTRX_TASK_STACKSIZE);

/**
 * @brief Initialize external receiver module
 */
void extRxInit(void)
{
  if (isInit)
  {
    return;
  }

  /* Initialize setpoint modes */
  extrxSetpoint.mode.roll = modeAbs;
  extrxSetpoint.mode.pitch = modeAbs;
  extrxSetpoint.mode.yaw = modeVelocity;

#ifdef CONFIG_ENABLE_SBUS
  /* Initialize SBUS driver */
  sbusInit();
  DEBUG_PRINT("SBUS receiver enabled\n");
#else
  DEBUG_PRINT("No external receiver configured\n");
  return;
#endif

  /* Create the external receiver task */
  STATIC_MEM_TASK_CREATE(extRxTask, extRxTask, EXTRX_TASK_NAME, NULL, EXTRX_TASK_PRI);

  isInit = true;
}

/**
 * @brief Check if external receiver is initialized
 */
bool extRxTest(void)
{
  return isInit;
}

/**
 * @brief Check if receiver signal is available
 */
bool extRxIsAvailable(void)
{
#ifdef CONFIG_ENABLE_SBUS
  return sbusIsAvailable();
#else
  return false;
#endif
}

/**
 * @brief Check if system is armed via external receiver
 */
bool extRxIsArmed(void)
{
  return isArmed;
}

/**
 * @brief External receiver task - reads SBUS frames and updates commander
 */
static void extRxTask(void *param)
{
#ifdef CONFIG_ENABLE_SBUS
  sbusFrame_t frame;
  static uint32_t loopCount = 0;
#endif

  /* Wait for the system to be fully started */
  systemWaitStart();

  printf("[EXTRX] External receiver task started, waiting for SBUS data...\n");

  while (true)
  {
#ifdef CONFIG_ENABLE_SBUS
    loopCount++;

    // /* Print status every 1000 loops (~20 seconds) */
    // if (loopCount % 1000 == 0)
    // {
    //   printf("[EXTRX] Active: loops=%lu, armed=%d, thr=%d\n",
    //          loopCount, isArmed, extrxSetpoint.thrust);
    // }

    /* Read SBUS frame (blocking with timeout) */
    if (sbusReadFrame(&frame, 20) == pdTRUE)
    {
      /* Copy channel values */
      for (int i = 0; i < EXTRX_NR_CHANNELS && i < SBUS_NUM_CHANNELS; i++)
      {
        ch[i] = frame.channels[i];
      }

      /* Check arm switch and update vehicle state */
      bool armSwitchOn = (frame.channels[EXTRX_CH_ARM] > EXTRX_ARM_THRESHOLD);

      /* Handle arming state transition */
      if (armSwitchOn && !isArmed)
      {
        /* Arm switch turned ON - attempt to arm */
        if (vehicleArm(false))
        {
          isArmed = true;
          DEBUG_PRINT("RC ARM: SUCCESS\n");
        }
        else
        {
          DEBUG_PRINT("RC ARM: FAILED - %s\n", vehicleGetArmFailReasonStr());
        }
      }
      else if (!armSwitchOn && isArmed)
      {
        /* Arm switch turned OFF - disarm */
        vehicleDisarm(false);
        isArmed = false;
        DEBUG_PRINT("RC DISARM\n");
      }

      /* Update RC connection status */
      vehicleSetRcConnected(true);

      /* Update flight mode based on mode switch */
      extRxUpdateFlightMode(frame.channels[EXTRX_CH_MODE]);

      /* Decode channels to setpoint */
      extRxDecodeChannels();

      // /* Output VOFA FireWater format: Raw channels + Converted values */
      // if (loopCount % 10 == 0) /* Output every 10 loops (~200ms) */
      // {
      //   printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.0f,%.2f,%.2f,%.2f\n",
      //          ch[0], ch[1], ch[2], ch[3], ch[4], // Raw CH0-CH4
      //          ch[5], ch[6], ch[7], ch[8], ch[9], // Raw CH5-CH9
      //          extrxSetpoint.thrust,              // Thrust (float, 0-65535)
      //          extrxSetpoint.attitude.roll,       // Roll (degrees)
      //          extrxSetpoint.attitude.pitch,      // Pitch (degrees)
      //          extrxSetpoint.attitude.yaw);       // Yaw (deg/s)
      // }
    }
    else if (!sbusIsAvailable())
    {
      /* No signal - disarm and zero controls, mark RC disconnected */
      if (isArmed)
      {
        vehicleDisarm(true); // Force disarm on signal loss
        isArmed = false;
        DEBUG_PRINT("RC SIGNAL LOST - DISARMED\\n");
      }
      vehicleSetRcConnected(false);
      extrxSetpoint.thrust = 0;
    }
#else
    /* No receiver configured, just delay */
    vTaskDelay(pdMS_TO_TICKS(100));
#endif
  }
}

/**
 * @brief Update flight mode based on mode switch channel
 *
 * 根据遥控器模式开关位置切换飞行模式:
 * - 低位 (Low):  STABILIZE_MODE - 自稳模式，仅姿态控制
 * - 中位 (Mid):  ALTHOLD_MODE   - 定高模式，自动保持高度
 * - 高位 (High): POSHOLD_MODE   - 定点模式，自动保持位置（需要光流/GPS）
 *
 * @param modeChannel Mode switch channel value (0-2047)
 */
static void extRxUpdateFlightMode(uint16_t modeChannel)
{
#ifdef CONFIG_ENABLE_SBUS
  VehicleFlightMode newMode;

  /* Determine flight mode based on switch position */
  if (modeChannel < EXTRX_MODE_LOW_THRESHOLD)
  {
    /* Low position: Stabilize mode */
    currentMode = STABILIZE_MODE;
    newMode = FLIGHT_MODE_STABILIZE;
  }
  else if (modeChannel < EXTRX_MODE_HIGH_THRESHOLD)
  {
    /* Mid position: Altitude hold mode */
    currentMode = ALTHOLD_MODE;
    newMode = FLIGHT_MODE_ALTITUDE;
  }
  else
  {
    /* High position: Position hold mode */
    currentMode = POSHOLD_MODE;
    newMode = FLIGHT_MODE_POSITION;
  }

  /* Only update if mode changed (avoid repeated calls) */
  if (currentMode != lastMode)
  {
    // 使用新的vehicle_state系统设置飞行模式
    vehicleSetFlightMode(newMode);
    DEBUG_PRINT("Flight mode changed: %s (switch=%d)\n",
                vehicleGetFlightModeName(newMode), modeChannel);
    lastMode = currentMode;
  }
#endif
}

/**
 * @brief Decode SBUS channel values to setpoint
 */
static void extRxDecodeChannels(void)
{
#ifdef CONFIG_ENABLE_SBUS
  /* Only send commands if armed */
  if (!isArmed)
  {
    extrxSetpoint.thrust = 0.0f;
    extrxSetpoint.attitude.roll = 0.0f;
    extrxSetpoint.attitude.pitch = 0.0f;
    extrxSetpoint.attitude.yaw = 0.0f;
    return;
  }

  /* Convert SBUS values to setpoint */
  /* Thrust: 0-65535 as float */
  extrxSetpoint.thrust = (float)sbusConvertToUint16(ch[EXTRX_CH_THRUST]);

  /* Roll: -SCALE to +SCALE degrees */
  extrxSetpoint.attitude.roll = EXTRX_SIGN_ROLL *
                                sbusConvertToRange(ch[EXTRX_CH_ROLL], -EXTRX_SCALE_ROLL, EXTRX_SCALE_ROLL);

  /* Pitch: -SCALE to +SCALE degrees */
  extrxSetpoint.attitude.pitch = EXTRX_SIGN_PITCH *
                                 sbusConvertToRange(ch[EXTRX_CH_PITCH], -EXTRX_SCALE_PITCH, EXTRX_SCALE_PITCH);

  /* Yaw: -SCALE to +SCALE degrees/second */
  extrxSetpoint.attitude.yaw = EXTRX_SIGN_YAW *
                               sbusConvertToRange(ch[EXTRX_CH_YAW], -EXTRX_SCALE_YAW, EXTRX_SCALE_YAW);

  /* Send setpoint to commander */
  commanderSetSetpoint(&extrxSetpoint, COMMANDER_PRIORITY_EXTRX);
#endif
}

/* Loggable variables */
#ifdef ENABLE_EXTRX_LOG
LOG_GROUP_START(extrx)
LOG_ADD(LOG_UINT16, ch0, &ch[0])
LOG_ADD(LOG_UINT16, ch1, &ch[1])
LOG_ADD(LOG_UINT16, ch2, &ch[2])
LOG_ADD(LOG_UINT16, ch3, &ch[3])
LOG_ADD(LOG_UINT16, ch4, &ch[4])
LOG_ADD(LOG_UINT16, ch5, &ch[5])
LOG_ADD(LOG_UINT16, ch6, &ch[6])
LOG_ADD(LOG_UINT16, ch7, &ch[7])
LOG_ADD(LOG_UINT16, ch8, &ch[8])
LOG_ADD(LOG_UINT16, ch9, &ch[9])
LOG_ADD(LOG_UINT16, thrust, &extrxSetpoint.thrust)
LOG_ADD(LOG_FLOAT, roll, &extrxSetpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &extrxSetpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &extrxSetpoint.attitude.yaw)
LOG_ADD(LOG_UINT8, armed, &isArmed)
LOG_ADD(LOG_UINT8, mode, &currentMode)
LOG_GROUP_STOP(extrx)
#endif
