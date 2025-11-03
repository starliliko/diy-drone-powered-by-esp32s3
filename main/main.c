/*
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
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "nvs_flash.h"

#include "stm32_legacy.h"
#include "platform.h"
#include "system.h"
#define DEBUG_MODULE "APP_MAIN"
#include "debug_cf.h"

#include "jy901_i2c_driver.h" // 包含JY901驱动头文件

static const char *TAG = "JY901_MAIN"; // 日志TAG定义

static const char *ERR = "ERR"; // 错误日志提示

static jy901_handle_t jy901_sensor; // 全局JY901传感器句柄
void app_main()
{
    /*
     * 初始化平台并启动系统任务
     * app_main 将初始化并启动所有功能
     */

    /* 初始化 NVS flash 为 Wi-Fi 做准备 */
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    /* 初始化平台 */
    if (platformInit() == false)
    {
        while (1)
            ESP_LOGW(ERR, "ERROR: Platform Init Failed");
        ; // 如果固件运行在错误的硬件上，停止运行
    }

    /* 启动系统任务 - 这个很重要！*/
    systemLaunch();

    // // 等待系统完全初始化（很重要！）
    // vTaskDelay(pdMS_TO_TICKS(2000)); // 增加到2秒
    // jy901_config_t config;
    // jy901_angle_t angle;
    // jy901_press_t press; // 添加气压高度数据结构

    // // 初始化新的JY901驱动
    // jy901_get_default_config(&config);

    // jy901_init(&jy901_sensor, &config);

    // // 设置输出频率为200Hz
    // jy901_set_output_rate(&jy901_sensor, RRATE_200HZ);
    // // 高度置零
    // jy901_reset_height_zero(&jy901_sensor);

    // while (1)
    // {

    //     // 同时读取角度数据和气压高度数据（使用校准后的高度）
    //     int angle_ret = jy901_read_angle(&jy901_sensor, &angle);
    //     int press_ret = jy901_read_pressure_height_calibrated(&jy901_sensor, &press);

    //     if (angle_ret == JY901_OK)
    //     {
    //         // 输出角度数据
    //         printf("%.3f,%.3f,%.3f,", angle.roll, angle.pitch, angle.yaw);

    //         // 输出气压和校准后的高度数据
    //         if (press_ret == JY901_OK)
    //         {
    //             printf("%.0f,%.2f\n", press.pressure, press.height);
    //         }
    //         else
    //         {
    //             // 气压高度读取失败时输出占位符
    //             printf("0,0.00\n");
    //         }
    //     }
    //     else
    //     {
    //         // 角度数据读取失败
    //         ESP_LOGW(TAG, "Read angle failed: %d", angle_ret);

    //         // 仍然尝试输出气压高度数据
    //         if (press_ret == JY901_OK)
    //         {
    //             printf("0.000,0.000,0.000,%.0f,%.2f\n", press.pressure, press.height);
    //         }
    //         else
    //         {
    //             printf("0.000,0.000,0.000,0,0.00\n");
    //         }
    //     }

    //     vTaskDelay(pdMS_TO_TICKS(5)); // 5ms延时 - 200Hz读取频率
    // }
}

/*
 * Initialize the platform and Launch the system task
 * app_main will initialize and start everything
 */

/* initialize nvs flash prepare for Wi-Fi */
// esp_err_t ret = nvs_flash_init();

// if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
// {
//     ESP_ERROR_CHECK(nvs_flash_erase());
//     ret = nvs_flash_init();
// }

// ESP_ERROR_CHECK(ret);

// /*Initialize the platform.*/
// if (platformInit() == false) {
//     while (1);//if  firmware is running on the wrong hardware, Halt
// }

// /*launch the system task */
// systemLaunch();
