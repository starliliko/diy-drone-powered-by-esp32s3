/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
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
 * system.c - Top level module implementation
 */

#include <stdbool.h>
#include <inttypes.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "version.h"
#include "config.h"
#include "param.h"
#include "log.h"
#include "ledseq.h"
#include "adc_esp32.h"
#include "pm_esplane.h"
#include "config.h"
#include "system.h"
#include "platform.h"
#include "configblock.h"
#include "worker.h"
#include "freeRTOSdebug.h"
#include "wifi_esp32.h"
#include "comm.h"
#include "stabilizer.h"
#include "commander.h"
#include "console.h"
#include "wifilink.h"
#include "mem.h"
#include "queuemonitor.h"
#include "buzzer.h"
#include "sound.h"
#include "sysload.h"
#include "estimator_kalman.h"
#include "app.h"
#include "stm32_legacy.h"
#include "extrx.h"
#include "flow_mtf01.h"
#include "esp_log.h"
#define DEBUG_MODULE "SYS"
#include "debug_cf.h"
#include "static_mem.h"
#include "cfassert.h"
#include "extrx.h"

#ifndef START_DISARMED
#define ARM_INIT true
#else
#define ARM_INIT false
#endif

/* Private variable */
static bool selftestPassed;
static bool canFly;
static bool armed = ARM_INIT;
static bool forceArm;
static bool isInit;

STATIC_MEM_TASK_ALLOC(systemTask, SYSTEM_TASK_STACKSIZE);

// 系统范围的同步
xSemaphoreHandle canStartMutex;
static StaticSemaphore_t canStartMutexBuffer;

/* Private functions */
static void systemTask(void *arg);

/* Public functions */
void systemLaunch(void)
{
  STATIC_MEM_TASK_CREATE(systemTask, systemTask, SYSTEM_TASK_NAME, NULL, SYSTEM_TASK_PRI);
  // 启动系统任务
}

// This must be the first module to be initialized!
// 这必须是第一个被初始化的模块!
void systemInit(void)
{
  if (isInit)
    return;

  DEBUG_PRINT_LOCAL("----------------------------\n");
  // DEBUG_PRINT_LOCAL("%s is up and running!\n", platformConfigGetDeviceTypeName());
  char name[20] = "diy——drone";
  DEBUG_PRINT_LOCAL("%s is up and running!\n", name);

  canStartMutex = xSemaphoreCreateMutexStatic(&canStartMutexBuffer);
  xSemaphoreTake(canStartMutex, portMAX_DELAY);
  // 分配空间创建互斥信号量，并立即获取它以阻止系统启动，直到所有初始化完成

  wifilinkInit(); // wifi链接初始化
  sysLoadInit();  // 系统负载初始化

  /* Initialized here so that DEBUG_PRINT (buffered) can be used early */
  // 中文注释: 这里初始化以便可以早期使用缓冲的DEBUG_PRINT

  debugInit();   // debug初始化
  crtpInit();    // crtp初始化
  consoleInit(); // 控制台初始化 //位置奇怪

  /* DEBUG_PRINT("----------------------------\n");
  DEBUG_PRINT("%s is up and running!\n", platformConfigGetDeviceTypeName());

  if (V_PRODUCTION_RELEASE) {
    DEBUG_PRINT("Production release %s\n", V_STAG);
  } else {
    DEBUG_PRINT("Build %s:%s (%s) %s\n", V_SLOCAL_REVISION,
                V_SREVISION, V_STAG, (V_MODIFIED)?"MODIFIED":"CLEAN");
  }
  DEBUG_PRINT("I am 0x%08X%08X%08X and I have %dKB of flash!\n",
              *((int*)(MCU_ID_ADDRESS+8)), *((int*)(MCU_ID_ADDRESS+4)),
              *((int*)(MCU_ID_ADDRESS+0)), *((short*)(MCU_FLASH_SIZE_ADDRESS)));*/

  configblockInit(); // 从Flash或 eeprom获得初始化
  // storageInit(); //持久化存储初始化
  workerInit(); // 工作队列初始化
  adcInit();    // ADC初始化
  ledseqInit(); // LED序列初始化 //LED动画
  pmInit();     // 电源管理初始化
  buzzerInit(); // 蜂鸣器初始化
  //  peerLocalizationInit();  //多机协同定位

#ifdef APP_ENABLED
  appInit();
#endif

  isInit = true;
}

bool systemTest()
{
  bool pass = isInit;

  pass &= ledseqTest();
  pass &= pmTest();
  DEBUG_PRINTI("pmTest = %d", pass);
  pass &= workerTest();
  DEBUG_PRINTI("workerTest = %d", pass);
  pass &= buzzerTest();
  return pass;
}

/* Private functions implementation */

void systemTask(void *arg)
{
  bool pass = true;

  // 底层外设初始化
  ledInit();          // led初始化
  ledSet(CHG_LED, 1); // 充电指示灯亮起
  wifiInit();         // wifi初始化

  vTaskDelay(M2T(500));
#ifdef DEBUG_QUEUE_MONITOR // 启动队列监视器
  queueMonitorInit();
#endif

  // 初始化高级模块
  systemInit();    // 系统初始化
  commInit();      // 通信初始化
  commanderInit(); // 通信指令初始化

#ifdef CONFIG_ENABLE_SBUS
  extRxInit(); // 外部接收器(SBUS)初始化
#endif

  // 选择估计器：
  // - complementaryEstimator: 使用Mahony AHRS，加速度计直接校正姿态，不依赖外部传感器
  // - kalmanEstimator: 需要TOF/光流等外部传感器校正，否则会漂移
#ifdef CONFIG_ENABLE_MTF01
  StateEstimatorType estimator = kalmanEstimator; // 有光流模块，使用卡尔曼
  estimatorKalmanTaskInit();
#else
  StateEstimatorType estimator = complementaryEstimator; // 无光流，使用互补滤波
#endif
  // deckInit();
  // estimator = deckGetRequiredEstimator();
  stabilizerInit(estimator); // 稳定器初始化
  // if (deckGetRequiredLowInterferenceRadioMode() && platformConfigPhysicalLayoutAntennasAreClose())
  // {
  //   platformSetLowInterferenceRadioMode();
  // } //无用

  soundInit(); // 蜂鸣器音效初始化
  memInit();   // 通过CRTP协议管理内存初始化

#ifdef PROXIMITY_ENABLED
  proximityInit(); // 接近/距离传感器初始化
#endif

#ifdef CONFIG_ENABLE_MTF01
  mtf01TaskInit(); // MTF01 光流/测距模块初始化
#endif

  // 测试每个模块
  pass &= wifiTest();
  DEBUG_PRINTI("wifilinkTest = %d ", pass);
  pass &= systemTest();
  DEBUG_PRINTI("systemTest = %d ", pass);
  pass &= configblockTest();
  DEBUG_PRINTI("configblockTest = %d ", pass);
  // pass &= storageTest();
  pass &= commTest();
  DEBUG_PRINTI("commTest = %d ", pass);
  pass &= commanderTest();
  DEBUG_PRINTI("commanderTest = %d ", pass);
  pass &= stabilizerTest();
  DEBUG_PRINTI("stabilizerTest = %d ", pass);
  // 互补滤波器模式下不测试卡尔曼估计器
#ifdef CONFIG_ENABLE_MTF01
  pass &= estimatorKalmanTaskTest();
  DEBUG_PRINTI("estimatorKalmanTaskTest = %d ", pass);
#endif
  // pass &= deckTest();
  // 可选测试：sound、mem、extrx 失败不影响主功能
  // bool optionalPass = true;
  // optionalPass &= soundTest();
  // DEBUG_PRINTI("soundTest = %d ", optionalPass);
  // optionalPass &= memTest();
  // DEBUG_PRINTI("memTest = %d ", optionalPass);
// pass &= watchdogNormalStartTest();
#ifdef CONFIG_ENABLE_SBUS
  optionalPass &= extRxTest();
  DEBUG_PRINTI("extRxTest = %d ", optionalPass);
#endif
#ifdef CONFIG_ENABLE_MTF01
  pass &= mtf01TaskTest();
  DEBUG_PRINTI("mtf01TaskTest = %d ", pass);
#endif
  pass &= cfAssertNormalStartTest();
  //  pass &= peerLocalizationTest();

  ESP_LOGI("SYS", "[SYSTEM] All tests completed, pass=%d", pass);

  // 系统自检
  if (pass)
  {
    ESP_LOGI("SYS", "[SYSTEM] Tests passed, calling systemStart()...");
    selftestPassed = 1;
    systemStart(); // 启动系统
    ESP_LOGI("SYS", "[SYSTEM] systemStart() returned");
    DEBUG_PRINTI("systemStart ! selftestPassed = %d", selftestPassed);
    soundSetEffect(SND_STARTUP); // 播放启动音效
    ledseqRun(&seq_alive);       // 运行存活LED灯效果
    ledseqRun(&seq_testPassed);
  }
  else
  {
    selftestPassed = 0;
    if (systemTest()) // 测试系统情况
    {
      while (1)
      {
        ledseqRun(&seq_testFailed);
        vTaskDelay(M2T(2000));
        // 系统可以通过从cfclient将参数设置为1来强制启动
        if (selftestPassed)
        {
          DEBUG_PRINT("Start forced.\n");
          systemStart();
          break;
        }
      }
    }
    else
    {
      ledInit();
      ledSet(SYS_LED, true); // 永远点亮系统LED表示严重错误
    }
  }
  DEBUG_PRINT("Free heap: %" PRIu32 " bytes\n", xPortGetFreeHeapSize());

  workerLoop(); // 工作队列，轻量级的异步任务调度和执行机制

  // Should never reach this point!
  // 永远不应该到达这里!
  while (1)
    vTaskDelay(portMAX_DELAY);
}

/* Global system variables */
void systemStart()
{
  ESP_LOGI("SYS", "[systemStart] Releasing canStartMutex...");
  xSemaphoreGive(canStartMutex);
  ESP_LOGI("SYS", "[systemStart] canStartMutex released");
#ifndef DEBUG_EP2
  // watchdogInit();
#endif
}

void systemWaitStart(void)
{
  // This permits to guarantee that the system task is initialized before other
  // tasks waits for the start event.
  while (!isInit)
    vTaskDelay(2);

  xSemaphoreTake(canStartMutex, portMAX_DELAY);
  xSemaphoreGive(canStartMutex);
}

void systemSetCanFly(bool val)
{
  canFly = val;
}

bool systemCanFly(void)
{
  return canFly;
}

void systemSetArmed(bool val)
{
  armed = val;
}

bool systemIsArmed()
{

  return armed || forceArm;
}
// void vApplicationIdleHook( void )
// {
//   static uint32_t tickOfLatestWatchdogReset = M2T(0);

//   portTickType tickCount = xTaskGetTickCount();

//   if (tickCount - tickOfLatestWatchdogReset > M2T(WATCHDOG_RESET_PERIOD_MS))
//   {
//     tickOfLatestWatchdogReset = tickCount;
//     watchdogReset();
//   }

//   // Enter sleep mode. Does not work when debugging chip with SWD.
//   // Currently saves about 20mA STM32F405 current consumption (~30%).
// #ifndef DEBUG
//   { __asm volatile ("wfi"); }
// #endif
// }

/*System parameters (mostly for test, should be removed from here) */
/*PARAM_GROUP_START(cpu)
PARAM_ADD(PARAM_UINT16 | PARAM_RONLY, flash, MCU_FLASH_SIZE_ADDRESS)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, id0, MCU_ID_ADDRESS+0)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, id1, MCU_ID_ADDRESS+4)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, id2, MCU_ID_ADDRESS+8)
PARAM_GROUP_STOP(cpu)*/

PARAM_GROUP_START(system)
PARAM_ADD(PARAM_INT8 | PARAM_RONLY, selftestPassed, &selftestPassed)
PARAM_ADD(PARAM_INT8, forceArm, &forceArm)
PARAM_GROUP_STOP(sytem)

/* Loggable variables */
LOG_GROUP_START(sys)
LOG_ADD(LOG_INT8, canfly, &canFly)
LOG_ADD(LOG_INT8, armed, &armed)
LOG_GROUP_STOP(sys)
