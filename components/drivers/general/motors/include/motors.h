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
 * Motors.h - Motor driver header file
 *
 */
#ifndef __MOTORS_H__
#define __MOTORS_H__

#include <stdint.h>
#include <stdbool.h>

#include "driver/ledc.h"

#include "config.h"
#include "stm32_legacy.h"

/******** Defines ********/

#define MOTORS_PWM_BITS LEDC_TIMER_8_BIT
#define MOTORS_PWM_PERIOD ((1 << MOTORS_PWM_BITS) - 1)
#define MOTORS_TIM_BEEP_CLK_FREQ 4000000

// 电机数量定义
#define NBR_OF_MOTORS 4

// 电机ID定义
#define MOTOR_M1 0
#define MOTOR_M2 1
#define MOTOR_M3 2
#define MOTOR_M4 3

// 电机GPIO 定义
#define MOTOR1_GPIO CONFIG_MOTOR01_PIN // M1 for ESP32S3
#define MOTOR2_GPIO CONFIG_MOTOR02_PIN // M2 for ESP32S3
#define MOTOR3_GPIO CONFIG_MOTOR03_PIN // M3 for ESP32S3
#define MOTOR4_GPIO CONFIG_MOTOR04_PIN // M4 for ESP32S3

// 电机PWM 通道定义
#define MOT_PWM_CH1 4 // Motor M1 pwmchannel
#define MOT_PWM_CH2 5 // Motor M2 pwmchannel
#define MOT_PWM_CH3 6 // Motor M3 pwmchannel
#define MOT_PWM_CH4 7 // Motor M4 pwmchannel

// 测试用占空比
#define MOTORS_TEST_RATIO (uint16_t)(0.05 * (1 << 16)) // 5% 推力
#define MOTORS_TEST_ON_TIME_MS 200
#define MOTORS_TEST_DELAY_TIME_MS 200

typedef enum
{
    BRUSHED,
    BRUSHLESS
} motorsDrvType;

typedef struct
{
    motorsDrvType drvType;

} MotorPerifDef;

// 电机映射配置

extern const MotorPerifDef *motorMapDefaultBrushed[NBR_OF_MOTORS];

/**
 测试音符 弃用
 */
extern const uint16_t testsound[NBR_OF_MOTORS];

void motorsInit(const MotorPerifDef **motorMapSelect);

/**
 * DeInitialisation. Reset to default
 */
void motorsDeInit(const MotorPerifDef **motorMapSelect);

/**
 * Test of the motor modules. The test will spin each motor very short in
 * the sequence M1 to M4.
 */
bool motorsTest(void);

/**
 * Set the PWM ratio of the motor 'id'
 */
void motorsSetRatio(uint32_t id, uint16_t ratio);

/**
 * Get the PWM ratio of the motor 'id'. Return -1 if wrong ID.
 */
int motorsGetRatio(uint32_t id);

/**
 * Make the motor 'id' beep with given frequency and ratio.
 */
// 空函数
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);

#endif /* __MOTORS_H__ */
