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
 * motor_test.h - 电机测试模块头文件
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 初始化电机测试模块
 *
 * 创建电机测试任务，初始化参数和日志系统。
 * 提供四个电机的独立控制和顺序测试功能。
 */
void motorTestInit(void);

/**
 * @brief 测试电机测试模块是否初始化
 *
 * @return true 模块已初始化
 * @return false 模块未初始化
 */
bool motorTestTest(void);

/**
 * @brief 查询电机测试是否正在运行
 *
 * @return true 电机测试正在运行 (testMode != 0)
 * @return false 电机测试未运行
 */
bool motorTestIsRunning(void);
