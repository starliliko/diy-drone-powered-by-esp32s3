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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_attr.h"
#include "esp_system.h"

#include "driver/mcpwm_prelude.h"

#include "stm32_legacy.h"
#include "motors.h"
#include "pm_esplane.h"
#include "log.h"
#include "param.h"
#define DEBUG_MODULE "MOTORS"
#include "debug_cf.h"

/******** MCPWM ESC Configuration ********/
#define ESC_TIMEBASE_RESOLUTION_HZ 1000000
#define ESC_TIMEBASE_PERIOD 2500
#define ESC_PULSE_MIN_US 1000
#define ESC_PULSE_MAX_US 2000

static const int motor_gpios[NBR_OF_MOTORS] = {
    MOTOR1_GPIO,
    MOTOR2_GPIO,
    MOTOR3_GPIO,
    MOTOR4_GPIO};

static mcpwm_timer_handle_t motor_timers[NBR_OF_MOTORS] = {NULL};
static mcpwm_oper_handle_t motor_operators[NBR_OF_MOTORS] = {NULL};
static mcpwm_cmpr_handle_t motor_comparators[NBR_OF_MOTORS] = {NULL};
static mcpwm_gen_handle_t motor_generators[NBR_OF_MOTORS] = {NULL};

static uint8_t escCalibRequest = 0;
static bool escCalibActive = false;

static uint32_t motor_pulse_us[NBR_OF_MOTORS] = {ESC_PULSE_MIN_US, ESC_PULSE_MIN_US, ESC_PULSE_MIN_US, ESC_PULSE_MIN_US};
uint32_t motor_ratios[NBR_OF_MOTORS] = {0, 0, 0, 0};

void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);
static void motorsHandleEscCalibCommand(void);
static uint16_t pulseWidthToThrust(uint32_t pulse_us);
static inline void motorSetPulseWidthRaw(uint32_t id, uint32_t pulse_us);
static inline void motorsSetAllPulseWidthRaw(uint32_t pulse_us);
const MotorPerifDef **motorMap;

const uint32_t MOTORS[] = {MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4};

const uint16_t testsound[NBR_OF_MOTORS] = {0, 0, 0, 0};

static bool isInit = false;

static inline void motorSetPulseWidthRaw(uint32_t id, uint32_t pulse_us)
{
    ASSERT(id < NBR_OF_MOTORS);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor_comparators[id], pulse_us));
    motor_pulse_us[id] = pulse_us;
    motor_ratios[id] = pulseWidthToThrust(pulse_us);
}

static inline void motorsSetAllPulseWidthRaw(uint32_t pulse_us)
{
    for (uint32_t i = 0; i < NBR_OF_MOTORS; i++)
    {
        motorSetPulseWidthRaw(i, pulse_us);
    }
}

static uint32_t thrustToPulseWidth(uint16_t thrust)
{
    return ESC_PULSE_MIN_US + ((uint32_t)thrust * (ESC_PULSE_MAX_US - ESC_PULSE_MIN_US)) / 65535;
}

static uint16_t pulseWidthToThrust(uint32_t pulse_us)
{
    if (pulse_us <= ESC_PULSE_MIN_US)
        return 0;
    if (pulse_us >= ESC_PULSE_MAX_US)
        return 65535;
    return (uint16_t)(((pulse_us - ESC_PULSE_MIN_US) * 65535) / (ESC_PULSE_MAX_US - ESC_PULSE_MIN_US));
}

void motorsInit(const MotorPerifDef **motorMapSelect)
{
    if (isInit)
    {
        return;
    }

    motorMap = motorMapSelect;

    for (int i = 0; i < NBR_OF_MOTORS; i++)
    {
        mcpwm_timer_config_t timer_config = {
            .group_id = i / 2,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = ESC_TIMEBASE_RESOLUTION_HZ,
            .period_ticks = ESC_TIMEBASE_PERIOD,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        };
        ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &motor_timers[i]));

        mcpwm_operator_config_t operator_config = {
            .group_id = i / 2,
        };
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &motor_operators[i]));

        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(motor_operators[i], motor_timers[i]));

        mcpwm_comparator_config_t comparator_config = {
            .flags.update_cmp_on_tez = true,
        };
        ESP_ERROR_CHECK(mcpwm_new_comparator(motor_operators[i], &comparator_config, &motor_comparators[i]));

        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = motor_gpios[i],
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(motor_operators[i], &generator_config, &motor_generators[i]));

        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor_comparators[i], ESC_PULSE_MIN_US));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            motor_generators[i],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            motor_generators[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor_comparators[i], MCPWM_GEN_ACTION_LOW)));

        ESP_ERROR_CHECK(mcpwm_timer_enable(motor_timers[i]));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(motor_timers[i], MCPWM_TIMER_START_NO_STOP));

        motor_pulse_us[i] = ESC_PULSE_MIN_US;
        motor_ratios[i] = 0;
    }

    DEBUG_PRINT("MCPWM ESC motors initialized (400Hz, 1000-2000us)\\n");
    DEBUG_PRINT("All motors set to MIN throttle (1000us), waiting for ESC arm...\\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    DEBUG_PRINT("ESC arm complete.\\n");

    isInit = true;
}

void motorsDeInit(const MotorPerifDef **motorMapSelect)
{
    (void)motorMapSelect;

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
    escCalibActive = false;
}

bool motorsTest(void)
{
    int i;

    for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
    {
        motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
        vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
        motorsSetRatio(MOTORS[i], 0);
        vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
    }

    return isInit;
}

void motorsSetRatio(uint32_t id, uint16_t ithrust)
{
    if (isInit)
    {
        motorsHandleEscCalibCommand();

        ASSERT(id < NBR_OF_MOTORS);

        if (escCalibActive)
        {
            // Block normal outputs while ESC calibration is active.
            return;
        }

        uint32_t pulse_us = thrustToPulseWidth(ithrust);

        motorSetPulseWidthRaw(id, pulse_us);
        motor_ratios[id] = ithrust;

#ifdef DEBUG_EP2
        DEBUG_PRINT_LOCAL("Motor %d: thrust=%d, pulse=%ldus\n", id, ithrust, pulse_us);
#endif
    }
}

int motorsGetRatio(uint32_t id)
{
    ASSERT(id < NBR_OF_MOTORS);
    return pulseWidthToThrust(motor_pulse_us[id]);
}

// ESC calibration command handling (manual power-on workflow):
// request=1: hold MAX throttle and wait for user to power on ESC
// request=2: switch to MIN throttle and finish calibration
// request=3: abort calibration and force MIN throttle
static void motorsHandleEscCalibCommand(void)
{
    if (escCalibRequest == 0)
    {
        return;
    }

    const uint8_t cmd = escCalibRequest;
    escCalibRequest = 0;

    if (cmd == 1)
    {
        escCalibActive = true;
        motorsSetAllPulseWidthRaw(ESC_PULSE_MAX_US);
        DEBUG_PRINT("ESC calibration STEP1: MAX throttle active. Power on ESC manually now.\\n");
        return;
    }

    if (cmd == 2)
    {
        if (!escCalibActive)
        {
            DEBUG_PRINT("ESC calibration STEP2 ignored: STEP1 not active.\\n");
            return;
        }

        motorsSetAllPulseWidthRaw(ESC_PULSE_MIN_US);
        DEBUG_PRINT("ESC calibration STEP2: MIN throttle active, waiting ESC confirm tones...\\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
        escCalibActive = false;
        DEBUG_PRINT("ESC calibration complete.\\n");
        return;
    }

    if (cmd == 3)
    {
        escCalibActive = false;
        motorsSetAllPulseWidthRaw(ESC_PULSE_MIN_US);
        DEBUG_PRINT("ESC calibration aborted, forced MIN throttle.\\n");
        return;
    }

    DEBUG_PRINT("ESC calibration unknown cmd=%d (use 1=start,2=finish,3=abort)\\n", cmd);
}

void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio)
{
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

PARAM_GROUP_START(escCalib)
PARAM_ADD(PARAM_UINT8, request, &escCalibRequest)
PARAM_GROUP_STOP(escCalib)
