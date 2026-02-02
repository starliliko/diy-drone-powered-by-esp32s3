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
#define EXTRX_MODE_HYSTERESIS 100 /* 迟滞值，防止边界跳变 */

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

  /* Signal quality detection - use ratio of valid/invalid frames */
  static int16_t prevStickChannels[4] = {0, 0, 0, 0}; /* Roll, Pitch, Yaw, Throttle */
  static uint32_t validFrameCount = 0;                /* Valid frames in current window */
  static uint32_t invalidFrameCount = 0;              /* Invalid frames in current window */
  static bool signalLostDetected = true;              /* Start as lost until link is qualified */
  static uint8_t goodWindowStreak = 0;                /* Consecutive good windows */
  static bool firstFrameReceived = false;             /* Skip first frame to initialize prevStickChannels */
#define STICK_CHANGE_THRESHOLD 5                      /* Minimum change to consider "active" */
#define WINDOW_SIZE 25                                /* Check signal quality every 25 frames (~0.5 second) */
#define SIGNAL_LOST_THRESHOLD 20                      /* If >20 invalid frames out of 25, signal is lost (80%) */
#define SIGNAL_RECOVER_WINDOWS 1                      /* Require N consecutive good windows before recovery */
#define CHANNEL_VALID_MIN 180                         /* Minimum valid channel value (below = failsafe) */
#define CHANNEL_VALID_MAX 1850                        /* Maximum valid channel value */
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
      /* Debug: Frame-level debug disabled for cleaner output */
      // static uint32_t frameDebugCount = 0;
      // frameDebugCount++;
      // if (frameDebugCount % 250 == 0)
      // {
      //   printf("[EXTRX] Frame#%lu: CH0=%d CH1=%d CH2=%d CH3=%d | FS=%d FL=%d | frozen=%lu sigLost=%d\n",
      //          frameDebugCount, frame.channels[0], frame.channels[1],
      //          frame.channels[2], frame.channels[3],
      //          frame.failsafe, frame.frameLost, invalidFrameCount, signalLostDetected);
      // }

      /* Check if this frame is valid or invalid */
      bool frameValid = true;

      /* Check failsafe and frame lost flags */
      if (frame.failsafe || frame.frameLost)
      {
        frameValid = false;
      }
      else
      {
        /* Check if channel values are in valid range */
        int16_t stickChannels[4] = {
            frame.channels[EXTRX_CH_ROLL],
            frame.channels[EXTRX_CH_PITCH],
            frame.channels[EXTRX_CH_YAW],
            frame.channels[EXTRX_CH_THRUST]};
        for (int i = 0; i < 4; i++)
        {
          if (stickChannels[i] < CHANNEL_VALID_MIN || stickChannels[i] > CHANNEL_VALID_MAX)
          {
            frameValid = false;
            break;
          }
        }
      }

      /* Count valid/invalid frames */
      if (frameValid)
      {
        validFrameCount++;
      }
      else
      {
        invalidFrameCount++;
      }

      /* Check signal quality every WINDOW_SIZE frames */
      uint32_t totalFrames = validFrameCount + invalidFrameCount;
      if (totalFrames >= WINDOW_SIZE)
      {
        if (invalidFrameCount >= SIGNAL_LOST_THRESHOLD)
        {
          /* Too many invalid frames - signal lost */
          goodWindowStreak = 0;
          if (!signalLostDetected)
          {
            signalLostDetected = true;
            printf("[EXTRX] RC SIGNAL LOST (invalid=%lu/%lu frames)\n",
                   invalidFrameCount, totalFrames);
            if (isArmed)
            {
              vehicleDisarm(true);
              isArmed = false;
              printf("[EXTRX] DISARMED due to signal loss\n");
            }
            vehicleSetRcConnected(false);
            extrxSetpoint.thrust = 0;
          }
        }
        else
        {
          /* Good signal quality */
          if (signalLostDetected)
          {
            goodWindowStreak++;
            if (goodWindowStreak >= SIGNAL_RECOVER_WINDOWS)
            {
              signalLostDetected = false;
              vehicleSetRcConnected(true);
              printf("[EXTRX] RC signal RECOVERED (valid=%lu/%lu frames)\n",
                     validFrameCount, totalFrames);
            }
          }
          else
          {
            /* Keep link qualified */
            goodWindowStreak = SIGNAL_RECOVER_WINDOWS;
          }
        }

        /* Reset counters for next window */
        validFrameCount = 0;
        invalidFrameCount = 0;
      }

      /* If signal is lost, skip processing */
      if (signalLostDetected)
      {
        vTaskDelay(pdMS_TO_TICKS(20));
        continue;
      }

      /* Only process valid frames */
      if (!frameValid)
      {
        vTaskDelay(pdMS_TO_TICKS(20));
        continue;
      }

      /* Activity detection: Check if stick channels are frozen */
      int16_t stickChannels[4] = {
          frame.channels[EXTRX_CH_ROLL],
          frame.channels[EXTRX_CH_PITCH],
          frame.channels[EXTRX_CH_YAW],
          frame.channels[EXTRX_CH_THRUST]};

      /* First valid frame: just initialize prevStickChannels, don't detect activity */
      if (!firstFrameReceived)
      {
        for (int i = 0; i < 4; i++)
        {
          prevStickChannels[i] = stickChannels[i];
        }
        firstFrameReceived = true;
        printf("[EXTRX] First SBUS frame received, activity detection started\n");
      }
      else
      {
        bool stickActivity = false;
        for (int i = 0; i < 4; i++)
        {
          int16_t diff = stickChannels[i] - prevStickChannels[i];
          if (diff < 0)
            diff = -diff; /* abs() */
          if (diff > STICK_CHANGE_THRESHOLD)
          {
            stickActivity = true;
            break;
          }
        }

        /* Update previous values */
        for (int i = 0; i < 4; i++)
        {
          prevStickChannels[i] = stickChannels[i];
        }

        /* stickActivity not used currently - frozen detection handled by frame ratio */
        (void)stickActivity;
      }

      /* Copy channel values */
      for (int i = 0; i < EXTRX_NR_CHANNELS && i < SBUS_NUM_CHANNELS; i++)
      {
        ch[i] = frame.channels[i];
      }

      /* Check arm switch and update vehicle state */
      bool armSwitchOn = (frame.channels[EXTRX_CH_ARM] > EXTRX_ARM_THRESHOLD);

      /* Debug: Print arm switch status periodically (every 5 seconds) */
      static uint32_t lastArmDebug = 0;
      if (loopCount - lastArmDebug >= 250) /* Every 5 seconds (~20ms * 250) */
      {
        printf("[EXTRX] ARM:sw=%d ch6=%d>%d | MODE:ch5=%d mode=%d | armed=%d | FS=%d FL=%d | inv=%lu sigLost=%d\n",
               armSwitchOn, frame.channels[EXTRX_CH_ARM], EXTRX_ARM_THRESHOLD,
               frame.channels[EXTRX_CH_MODE], currentMode, isArmed,
               frame.failsafe, frame.frameLost, invalidFrameCount, signalLostDetected);
        lastArmDebug = loopCount;
      }

      /* Handle arming state transition */
      static bool armFailLogged = false; /* 移到外部以便多处使用 */

      if (armSwitchOn && !isArmed)
      {
        /* Arm switch turned ON - attempt to arm */
        if (vehicleArm(false))
        {
          isArmed = true;
          printf("[EXTRX] RC ARM SUCCESS\n");
          armFailLogged = false; /* 成功后重置 */
        }
        else
        {
          /* Only print failure once per attempt */
          if (!armFailLogged)
          {
            printf("[EXTRX] RC ARM FAILED: %s\n", vehicleGetArmFailReasonStr());
            armFailLogged = true;
          }
        }
      }
      else if (!armSwitchOn && isArmed)
      {
        /* Arm switch turned OFF - disarm */
        vehicleDisarm(false);
        isArmed = false;
        printf("[EXTRX] RC DISARM\n");
        armFailLogged = false; /* 解锁开关关闭时重置 */
      }
      else if (!armSwitchOn)
      {
        /* Reset arm failure flag when switch is off */
        armFailLogged = false;
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
      /* No signal (timeout or failsafe) - disarm and zero controls */
      static bool wasConnected = true;
      if (wasConnected)
      {
        DEBUG_PRINT("RC SIGNAL LOST\n");
        wasConnected = false;
      }

      if (isArmed)
      {
        vehicleDisarm(true); // Force disarm on signal loss
        isArmed = false;
        DEBUG_PRINT("RC SIGNAL LOST - DISARMED\n");
      }
      vehicleSetRcConnected(false);

      /* Send zero setpoint to commander so it knows SBUS is inactive */
      /* This will cause the control source to timeout and failover */
      extrxSetpoint.thrust = 0;
      extrxSetpoint.attitude.roll = 0.0f;
      extrxSetpoint.attitude.pitch = 0.0f;
      extrxSetpoint.attitude.yaw = 0.0f;
      /* Note: We intentionally DON'T call commanderSetSetpoint here */
      /* so that the control source will timeout and switch away */

      /* Small delay before retry */
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    else
    {
      /* SBUS available but no new frame yet */
      static bool wasConnected = false;
      if (!wasConnected)
      {
        DEBUG_PRINT("RC SIGNAL RESTORED\n");
        wasConnected = true;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
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
  static uint16_t lowThreshold = EXTRX_MODE_LOW_THRESHOLD;
  static uint16_t highThreshold = EXTRX_MODE_HIGH_THRESHOLD;

  /* 使用迟滞逻辑判断飞行模式，防止边界跳变 */
  /* 根据当前模式调整阈值 */
  switch (currentMode)
  {
  case STABILIZE_MODE:
    /* 当前是STABILIZE，需要超过 LOW+HYSTERESIS 才切换到 ALTHOLD */
    lowThreshold = EXTRX_MODE_LOW_THRESHOLD + EXTRX_MODE_HYSTERESIS;
    highThreshold = EXTRX_MODE_HIGH_THRESHOLD;
    break;
  case ALTHOLD_MODE:
    /* 当前是ALTHOLD，需要低于 LOW-HYSTERESIS 才切换到 STABILIZE */
    /* 需要高于 HIGH+HYSTERESIS 才切换到 POSHOLD */
    lowThreshold = EXTRX_MODE_LOW_THRESHOLD - EXTRX_MODE_HYSTERESIS;
    highThreshold = EXTRX_MODE_HIGH_THRESHOLD + EXTRX_MODE_HYSTERESIS;
    break;
  case POSHOLD_MODE:
  case POSSET_MODE:
  default:
    /* 当前是POSHOLD，需要低于 HIGH-HYSTERESIS 才切换到 ALTHOLD */
    lowThreshold = EXTRX_MODE_LOW_THRESHOLD;
    highThreshold = EXTRX_MODE_HIGH_THRESHOLD - EXTRX_MODE_HYSTERESIS;
    break;
  }

  /* Determine flight mode based on switch position with hysteresis */
  if (modeChannel < lowThreshold)
  {
    /* Low position: Stabilize mode */
    currentMode = STABILIZE_MODE;
    newMode = FLIGHT_MODE_STABILIZE;
  }
  else if (modeChannel >= highThreshold)
  {
    /* High position: Position hold mode */
    currentMode = POSHOLD_MODE;
    newMode = FLIGHT_MODE_POSITION;
  }
  else
  {
    /* Mid position: Altitude hold mode */
    currentMode = ALTHOLD_MODE;
    newMode = FLIGHT_MODE_ALTITUDE;
  }

  /* Only update if mode changed (avoid repeated calls) */
  if (currentMode != lastMode)
  {
    // 使用新的vehicle_state系统设置飞行模式
    vehicleSetFlightMode(newMode);
    printf("[EXTRX] MODE CHANGED: %d->%d ch5=%d (thresholds: low=%d high=%d)\n",
           lastMode, currentMode, modeChannel, lowThreshold, highThreshold);
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

  /* If not armed, zero thrust for safety but still send to commander */
  /* This allows commander to track SBUS as active control source */
  if (!isArmed)
  {
    extrxSetpoint.thrust = 0.0f;
  }

  /* Send setpoint to commander (always, to maintain control source status) */
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
