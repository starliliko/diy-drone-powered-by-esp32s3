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
#include "esp_log.h"
#include "bmi088_spi.h"
#include "bmi088.h"
#include "spi_drv.h"
#include "sbus.h"
#include "i2cdev.h"
#define DEBUG_MODULE "APP_MAIN"
#include "debug_cf.h"

// 声明BMI088配置函数
extern bool bmi088_init_with_default_config(bmi088_dev_t *dev);

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
            DEBUG_PRINT("ERROR: Platform Init Failed\n");
        ; // 如果固件运行在错误的硬件上,停止运行
    }

    /* BMI088硬件测试 - 直接读取数据验证 */
    ESP_LOGI("APP_MAIN", "=== BMI088 Hardware Test ===");

    // 初始化SPI
    spiDrvInit(SPI_DRV_HOST_DEFAULT);
    vTaskDelay(pdMS_TO_TICKS(100));

    // 创建BMI088设备结构体
    bmi088_dev_t bmi088_dev = {0};

    // 使用默认配置初始化BMI088
    if (bmi088_init_with_default_config(&bmi088_dev))
    {
        ESP_LOGI("APP_MAIN", "BMI088 init SUCCESS");

        // 读取10次数据验证
        for (int i = 0; i < 10; i++)
        {
            bmi088_sensor_data_t gyro_data, acc_data;

            if (bmi088_spi_read_gyro_data(&bmi088_dev, &gyro_data) &&
                bmi088_spi_read_acc_data(&bmi088_dev, &acc_data))
            {
                ESP_LOGI("APP_MAIN", "[Test %d] Gyro: X=%d Y=%d Z=%d | Accel: X=%d Y=%d Z=%d",
                         i + 1, gyro_data.x, gyro_data.y, gyro_data.z,
                         acc_data.x, acc_data.y, acc_data.z);
            }
            else
            {
                ESP_LOGE("APP_MAIN", "[Test %d] Failed to read sensor data", i + 1);
            }

            vTaskDelay(pdMS_TO_TICKS(100));
        }

        ESP_LOGI("APP_MAIN", "=== BMI088 Test Complete ===");
    }
    else
    {
        ESP_LOGE("APP_MAIN", "BMI088 init FAILED!");
    }

    /* SBUS硬件测试 - 初始化并等待数据 */
    ESP_LOGI("APP_MAIN", "=== SBUS Hardware Test ===");

    sbusInit();

    if (sbusTest())
    {
        ESP_LOGI("APP_MAIN", "SBUS init SUCCESS, waiting for receiver data...");

        sbusFrame_t frame;
        int framesReceived = 0;

        // 尝试读取10次数据，每次等待500ms
        for (int i = 0; i < 10; i++)
        {
            if (sbusReadFrame(&frame, 500) == pdTRUE)
            {
                framesReceived++;
                ESP_LOGI("APP_MAIN", "[Test %d] CH1=%4d CH2=%4d CH3=%4d CH4=%4d | FS=%d FL=%d",
                         framesReceived,
                         frame.channels[0], frame.channels[1],
                         frame.channels[2], frame.channels[3],
                         frame.failsafe, frame.frameLost);

                // 如果已经收到3帧有效数据，测试通过
                if (framesReceived >= 3)
                {
                    ESP_LOGI("APP_MAIN", "SBUS receiving data OK!");
                    break;
                }
            }
            else
            {
                ESP_LOGW("APP_MAIN", "[Attempt %d] No SBUS data received (timeout 500ms)", i + 1);
            }
        }

        if (framesReceived == 0)
        {
            ESP_LOGW("APP_MAIN", "SBUS: No data received - check receiver connection");
            ESP_LOGW("APP_MAIN", "  - Verify receiver is powered and bound to transmitter");
            ESP_LOGW("APP_MAIN", "  - Verify SBUS wire connected to RX pin");
        }

        ESP_LOGI("APP_MAIN", "=== SBUS Test Complete (frames: %d) ===", framesReceived);
    }
    else
    {
        ESP_LOGE("APP_MAIN", "SBUS init FAILED!");
    }

    /* 启动系统任务（QMC5883P 已移至 sensorsDeviceInit 统一初始化） */
    systemLaunch();
}
