/**
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
 * motors.c - Motor driver (MCPWM for Brushless ESC)
 *
 */

#include <stdbool.h>

// FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ESP-IDF MCPWM driver
#include "driver/mcpwm_prelude.h"

#include "stm32_legacy.h"
#include "motors.h"
#include "pm_esplane.h"
#include "log.h"
#define DEBUG_MODULE "MOTORS"
#include "debug_cf.h"

/******** MCPWM ESC Configuration ********/
// ESC标准PWM参数：400Hz，脉宽1000~2000us
#define ESC_TIMEBASE_RESOLUTION_HZ 1000000 // 1MHz，每tick为1us
#define ESC_TIMEBASE_PERIOD 2500           // 2500 ticks = 2.5ms（400Hz）
#define ESC_PULSE_MIN_US 1000              // 最小脉宽1000us（0%油门）
#define ESC_PULSE_MAX_US 2000              // 最大脉宽2000us（100%油门）

// 电机GPIO数组
static const int motor_gpios[NBR_OF_MOTORS] = {
    MOTOR1_GPIO,
    MOTOR2_GPIO,
    MOTOR3_GPIO,
    MOTOR4_GPIO};

// MCPWM句柄
static mcpwm_timer_handle_t motor_timers[NBR_OF_MOTORS] = {NULL};
static mcpwm_oper_handle_t motor_operators[NBR_OF_MOTORS] = {NULL};
static mcpwm_cmpr_handle_t motor_comparators[NBR_OF_MOTORS] = {NULL};
static mcpwm_gen_handle_t motor_generators[NBR_OF_MOTORS] = {NULL};

// 电机当前脉宽（us），用于读取和日志
static uint32_t motor_pulse_us[NBR_OF_MOTORS] = {ESC_PULSE_MIN_US, ESC_PULSE_MIN_US, ESC_PULSE_MIN_US, ESC_PULSE_MIN_US};

// 电机推力比例（0~65535），用于日志
uint32_t motor_ratios[NBR_OF_MOTORS] = {0, 0, 0, 0};

void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);
static void motorsUnlockESC(void);
const MotorPerifDef **motorMap; /* Current map configuration */

const uint32_t MOTORS[] = {MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4};

const uint16_t testsound[NBR_OF_MOTORS] = {0, 0, 0, 0};

static bool isInit = false;

/* Private functions */

/**
 * 将0~65535的推力值转换为1000~2000us脉宽
 */
static uint32_t thrustToPulseWidth(uint16_t thrust)
{
    // 线性映射：0 -> 1000us, 65535 -> 2000us
    return ESC_PULSE_MIN_US + ((uint32_t)thrust * (ESC_PULSE_MAX_US - ESC_PULSE_MIN_US)) / 65535;
}

/**
 * 将1000~2000us脉宽转换为0~65535的推力值
 */
static uint16_t pulseWidthToThrust(uint32_t pulse_us)
{
    if (pulse_us <= ESC_PULSE_MIN_US)
        return 0;
    if (pulse_us >= ESC_PULSE_MAX_US)
        return 65535;
    return (uint16_t)(((pulse_us - ESC_PULSE_MIN_US) * 65535) / (ESC_PULSE_MAX_US - ESC_PULSE_MIN_US));
}

/* Public functions */

// Initialization. Will set all motors ratio to 0% (1000us pulse)
void motorsInit(const MotorPerifDef **motorMapSelect)
{
    if (isInit)
    {
        return;
    }

    motorMap = motorMapSelect;

    // 为每个电机创建独立的MCPWM资源
    for (int i = 0; i < NBR_OF_MOTORS; i++)
    {
        // 创建定时器（使用两个MCPWM组，每组2个电机）
        mcpwm_timer_config_t timer_config = {
            .group_id = i / 2, // 电机0,1用组0；电机2,3用组1
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = ESC_TIMEBASE_RESOLUTION_HZ,
            .period_ticks = ESC_TIMEBASE_PERIOD,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        };
        ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &motor_timers[i]));

        // 创建操作器
        mcpwm_operator_config_t operator_config = {
            .group_id = i / 2,
        };
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &motor_operators[i]));

        // 连接定时器和操作器
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(motor_operators[i], motor_timers[i]));

        // 创建比较器
        mcpwm_comparator_config_t comparator_config = {
            .flags.update_cmp_on_tez = true,
        };
        ESP_ERROR_CHECK(mcpwm_new_comparator(motor_operators[i], &comparator_config, &motor_comparators[i]));

        // 创建生成器
        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = motor_gpios[i],
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(motor_operators[i], &generator_config, &motor_generators[i]));

        // 设置初始比较值为最小油门（1000us）
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor_comparators[i], ESC_PULSE_MIN_US));

        // 设置生成器动作：计数器为空时输出高电平，达到比较值时输出低电平
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(motor_generators[i],
                                                                  MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(motor_generators[i],
                                                                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor_comparators[i], MCPWM_GEN_ACTION_LOW)));

        // 启用并启动定时器
        ESP_ERROR_CHECK(mcpwm_timer_enable(motor_timers[i]));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(motor_timers[i], MCPWM_TIMER_START_NO_STOP));

        motor_pulse_us[i] = ESC_PULSE_MIN_US;
        motor_ratios[i] = 0;
    }

    DEBUG_PRINT("MCPWM ESC motors initialized (400Hz, 1000-2000us)\n");

    vTaskDelay(pdMS_TO_TICKS(50)); // 等待PWM稳定
    motorsUnlockESC();             // 电调解锁
    motorsSetRatio(MOTOR_M1, 0);   // 确保所有电机推力为0
    motorsSetRatio(MOTOR_M2, 0);
    motorsSetRatio(MOTOR_M3, 0);
    motorsSetRatio(MOTOR_M4, 0);
    isInit = true;
}

void motorsDeInit(const MotorPerifDef **motorMapSelect)
{
    for (int i = 0; i < NBR_OF_MOTORS; i++)
    {
        if (motor_timers[i] != NULL)
        {
            mcpwm_timer_start_stop(motor_timers[i], MCPWM_TIMER_STOP_EMPTY);
            mcpwm_timer_disable(motor_timers[i]);
            mcpwm_del_generator(motor_generators[i]);
            mcpwm_del_comparator(motor_comparators[i]);
            mcpwm_del_operator(motor_operators[i]);
            mcpwm_del_timer(motor_timers[i]);
            motor_timers[i] = NULL;
        }
    }
    isInit = false;
}

bool motorsTest(void)
{
    int i;

    for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
    {
        // 无刷电机测试：短暂给一点油门
        motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
        vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
        motorsSetRatio(MOTORS[i], 0);
        vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
    }

    return isInit;
}

// ithrust: 0~65535 映射到 1000~2000us 脉宽
void motorsSetRatio(uint32_t id, uint16_t ithrust)
{
    if (isInit)
    {
        ASSERT(id < NBR_OF_MOTORS);

        // 转换推力值为脉宽
        uint32_t pulse_us = thrustToPulseWidth(ithrust);

        // 设置比较值
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor_comparators[id], pulse_us));

        // 保存当前状态
        motor_pulse_us[id] = pulse_us;
        motor_ratios[id] = ithrust;

#ifdef DEBUG_EP2
        DEBUG_PRINT_LOCAL("Motor %d: thrust=%d, pulse=%ldus\n", id, ithrust, pulse_us);
#endif
        // 添加实时输出
        printf("M%d:%d ", id + 1, ithrust);
        if (id == 3)
            printf("\n"); // 4个电机输出完换行
    }
}

int motorsGetRatio(uint32_t id)
{
    ASSERT(id < NBR_OF_MOTORS);
    // 从保存的脉宽转换回推力值
    return pulseWidthToThrust(motor_pulse_us[id]);
}

// 电调解锁
// 发送最大油门2秒，再发送最小油门2秒，实际上也做了电调校准
static void motorsUnlockESC(void)
{
    // 步骤1：最大油门
    motorsSetRatio(MOTOR_M1, 65535);
    motorsSetRatio(MOTOR_M2, 65535);
    motorsSetRatio(MOTOR_M3, 65535);
    motorsSetRatio(MOTOR_M4, 65535);

    vTaskDelay(pdMS_TO_TICKS(2000));
    // 步骤2：最小油门
    motorsSetRatio(MOTOR_M1, 0);
    motorsSetRatio(MOTOR_M2, 0);
    motorsSetRatio(MOTOR_M3, 0);
    motorsSetRatio(MOTOR_M4, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));
}

// 无刷电机+ESC不支持蜂鸣功能，保留空函数
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio)
{
    // 无刷电调不支持通过PWM频率变化发声
    // 如需蜂鸣功能，请使用支持蜂鸣的ESC或外置蜂鸣器
    (void)id;
    (void)enable;
    (void)frequency;
    (void)ratio;
}

LOG_GROUP_START(pwm)
LOG_ADD(LOG_UINT32, m1_pwm, &motor_ratios[0])
LOG_ADD(LOG_UINT32, m2_pwm, &motor_ratios[1])
LOG_ADD(LOG_UINT32, m3_pwm, &motor_ratios[2])
LOG_ADD(LOG_UINT32, m4_pwm, &motor_ratios[3])
LOG_GROUP_STOP(pwm)
