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
 * motor_test.c - 电机测试模块
 *
 * 用于飞行前电机输出测试：
 * 1. 单个电机测试
 * 2. 顺序测试所有电机
 * 3. 自定义推力测试
 * 4. 急停功能
 * 5. 电机输出LOG记录
 */

#define DEBUG_MODULE "MOTOR_TEST"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "debug_cf.h"
#include "param.h"
#include "log.h"
#include "motors.h"
#include "motor_test.h"
#include "static_mem.h"

/*===========================================================================
 * 内部常量定义
 *===========================================================================*/

#define MOTOR_TEST_TASK_STACKSIZE (2 * configMINIMAL_STACK_SIZE)
#define MOTOR_TEST_TASK_PRI (tskIDLE_PRIORITY + 1)
#define MOTOR_TEST_TASK_NAME "MOTOR_TEST"

#define SEQUENTIAL_TEST_THRUST (uint16_t)(0.10 * 65535) // 顺序测试推力：10%
#define SEQUENTIAL_TEST_DURATION_MS 1000                // 每个电机运行时长
#define SEQUENTIAL_TEST_INTERVAL_MS 500                 // 电机间隔时长

/*===========================================================================
 * 内部变量
 *===========================================================================*/

static bool isInit = false;

// 测试参数
static uint8_t testMode = 0;      // 0=停止, 1=顺序测试, 2=单电机测试, 3=全部电机同时测试
static uint8_t motorId = 0;       // 要测试的电机ID (0-3)
static uint16_t motorThrust = 0;  // 电机推力 (0-65535)
static uint8_t emergencyStop = 0; // 急停标志 (使用uint8_t以匹配PARAM_UINT8)
static bool testRunning = false;  // 测试运行标志

// 电机输出记录（用于LOG导出到地面站）
NO_DMA_CCM_SAFE_ZERO_INIT static uint16_t motorOutput[4];     // 电机实时输出 (0-65535)
NO_DMA_CCM_SAFE_ZERO_INIT static float motorOutputPercent[4]; // 百分比

/*===========================================================================
 * 私有函数声明
 *===========================================================================*/

static void motorTestTask(void *param);

STATIC_MEM_TASK_ALLOC(motorTestTask, MOTOR_TEST_TASK_STACKSIZE);

/*===========================================================================
 * 私有函数实现
 *===========================================================================*/

/**
 * 更新电机输出记录
 */
static void updateMotorOutput(uint8_t motor, uint16_t thrust)
{
    if (motor < NBR_OF_MOTORS)
    {
        motorOutput[motor] = thrust;
        motorOutputPercent[motor] = (float)thrust * 100.0f / 65535.0f;
        motorsSetRatio(motor, thrust);
    }
}

/*===========================================================================
 * 公共函数实现
 *===========================================================================*/

void motorTestInit(void)
{
    if (isInit)
    {
        return;
    }

    // 初始化电机输出数组
    memset(motorOutput, 0, sizeof(motorOutput));
    memset(motorOutputPercent, 0, sizeof(motorOutputPercent));

    // 创建电机测试任务（使用静态内存分配）
    STATIC_MEM_TASK_CREATE(motorTestTask, motorTestTask, MOTOR_TEST_TASK_NAME, NULL, MOTOR_TEST_TASK_PRI);

    isInit = true;

    DEBUG_PRINT("Motor test module initialized\n");
}

bool motorTestTest(void)
{
    return isInit;
}

/*===========================================================================
 * 任务实现
 *===========================================================================*/

static void motorTestTask(void *param)
{
    static uint32_t lastTestTime = 0;
    static uint8_t currentMotor = 0;
    static bool motorActive = false;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        // 检查急停
        if (emergencyStop)
        {
            for (int i = 0; i < NBR_OF_MOTORS; i++)
            {
                updateMotorOutput(i, 0);
            }
            testMode = 0;
            testRunning = false;
            motorActive = false;
            emergencyStop = 0;
            DEBUG_PRINT("Emergency stop activated!\n");
        }

        switch (testMode)
        {
        case 0: // 停止模式
            if (testRunning)
            {
                // 停止所有电机
                for (int i = 0; i < NBR_OF_MOTORS; i++)
                {
                    updateMotorOutput(i, 0);
                }
                testRunning = false;
                motorActive = false;
                DEBUG_PRINT("Motor test stopped\n");
            }
            break;

        case 1: // 顺序测试模式
            if (!testRunning)
            {
                testRunning = true;
                currentMotor = 0;
                motorActive = false;
                lastTestTime = xTaskGetTickCount();
                DEBUG_PRINT("Starting sequential motor test...\n");
            }

            if (testRunning)
            {
                uint32_t currentTime = xTaskGetTickCount();
                uint32_t elapsed = currentTime - lastTestTime;

                if (!motorActive)
                {
                    // 启动当前电机
                    DEBUG_PRINT("Testing Motor M%d at %d%% thrust\n",
                                currentMotor + 1,
                                (int)(SEQUENTIAL_TEST_THRUST * 100 / 65535));
                    updateMotorOutput(currentMotor, SEQUENTIAL_TEST_THRUST);
                    motorActive = true;
                    lastTestTime = currentTime;
                }
                else if (elapsed >= pdMS_TO_TICKS(SEQUENTIAL_TEST_DURATION_MS))
                {
                    // 停止当前电机
                    updateMotorOutput(currentMotor, 0);
                    DEBUG_PRINT("Motor M%d test completed\n", currentMotor + 1);
                    motorActive = false;
                    lastTestTime = currentTime;
                    currentMotor++;

                    if (currentMotor >= NBR_OF_MOTORS)
                    {
                        // 所有电机测试完成
                        DEBUG_PRINT("Sequential motor test completed\n");
                        testMode = 0;
                        testRunning = false;
                    }
                    else
                    {
                        // 等待间隔后测试下一个电机
                        vTaskDelay(pdMS_TO_TICKS(SEQUENTIAL_TEST_INTERVAL_MS));
                    }
                }
            }
            break;

        case 2: // 单电机测试模式
            if (!testRunning)
            {
                testRunning = true;
                DEBUG_PRINT("Testing single motor M%d at thrust %d\n",
                            motorId + 1, motorThrust);
            }

            if (testRunning)
            {
                // 确保其他电机停止
                for (int i = 0; i < NBR_OF_MOTORS; i++)
                {
                    if (i != motorId)
                    {
                        updateMotorOutput(i, 0);
                    }
                }

                // 设置目标电机推力
                if (motorId < NBR_OF_MOTORS)
                {
                    updateMotorOutput(motorId, motorThrust);
                }
            }
            break;

        case 3: // 全部电机同时测试
            if (!testRunning)
            {
                testRunning = true;
                DEBUG_PRINT("Testing all motors at thrust %d\n", motorThrust);
            }

            if (testRunning)
            {
                // 所有电机设置相同推力
                for (int i = 0; i < NBR_OF_MOTORS; i++)
                {
                    updateMotorOutput(i, motorThrust);
                }
            }
            break;

        default:
            testMode = 0;
            break;
        }

        // 50Hz 更新频率
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}

/*===========================================================================
 * 参数系统
 *===========================================================================*/

/**
 * 参数组定义
 * 使用方法：
 * - motortest.mode: 设置测试模式 (0=停止, 1=顺序测试, 2=单电机, 3=全部电机)
 * - motortest.id: 选择要测试的电机 (0-3 对应 M1-M4)
 * - motortest.thrust: 设置推力值 (0-65535, 建议从 3276 (5%) 开始测试)
 * - motortest.estop: 设置为 1 触发急停
 */
PARAM_GROUP_START(motortest)
PARAM_ADD(PARAM_UINT8, mode, &testMode)
PARAM_ADD(PARAM_UINT8, id, &motorId)
PARAM_ADD(PARAM_UINT16, thrust, &motorThrust)
PARAM_ADD(PARAM_UINT8, estop, &emergencyStop)
PARAM_GROUP_STOP(motortest)

/*===========================================================================
 * 日志系统
 *===========================================================================*/

/**
 * 日志组定义 - 导出到地面站
 * 地面站可以订阅这些LOG来实时显示电机输出
 */
LOG_GROUP_START(motors)
LOG_ADD(LOG_UINT16, m1, &motorOutput[0])
LOG_ADD(LOG_UINT16, m2, &motorOutput[1])
LOG_ADD(LOG_UINT16, m3, &motorOutput[2])
LOG_ADD(LOG_UINT16, m4, &motorOutput[3])
LOG_ADD(LOG_FLOAT, m1_pct, &motorOutputPercent[0])
LOG_ADD(LOG_FLOAT, m2_pct, &motorOutputPercent[1])
LOG_ADD(LOG_FLOAT, m3_pct, &motorOutputPercent[2])
LOG_ADD(LOG_FLOAT, m4_pct, &motorOutputPercent[3])
LOG_GROUP_STOP(motors)
