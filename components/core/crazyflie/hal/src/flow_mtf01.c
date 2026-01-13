/**
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020 Espressif Systems (Shanghai)
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
 * mtf01_task.c - MTF01 optical flow & rangefinder task implementation
 *
 * MTF01 sensor outputs:
 * - Distance (ToF): 10-2000mm
 * - Optical flow velocity: cm/s @ 1m height
 * - Actual velocity = flow_velocity * height(m)
 */

#include "sdkconfig.h"

#ifdef CONFIG_ENABLE_MTF01

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "driver/uart.h"
#include "esp_log.h"

#include "config.h"
#include "system.h"
#include "static_mem.h"
#include "stm32_legacy.h"
#include "estimator.h"
#include "log.h"
#include "param.h"
#include "usec_time.h"

#include "mtf01.h"
#include "flow_mtf01.h"

#define DEBUG_MODULE "MTF01"

static const char *TAG = "MTF01";

/* Task configuration */
#define MTF01_TASK_FREQ 100 // 100Hz task frequency
#define MTF01_UART_BUF_SIZE 256
#define MTF01_READ_TIMEOUT_MS 10

/* UART configuration from Kconfig */
#define MTF01_UART_NUM CONFIG_MTF01_UART_NUM
#define MTF01_RX_PIN CONFIG_MTF01_RX_PIN
#define MTF01_TX_PIN CONFIG_MTF01_TX_PIN
#define MTF01_BAUDRATE CONFIG_MTF01_BAUDRATE
#define MTF01_MIN_DISTANCE CONFIG_MTF01_MIN_DISTANCE
#define MTF01_MAX_DISTANCE CONFIG_MTF01_MAX_DISTANCE

/* Module state */
static bool isInit = false;

/* Sensor data */
static uint32_t distance_mm = 0; // Distance in mm
static uint8_t tofStrength = 0;  // ToF signal strength
static uint8_t tofPrecision = 0; // ToF precision
static uint8_t tofStatus = 0;    // ToF status
static int16_t flowVelX = 0;     // Flow velocity X (cm/s @ 1m)
static int16_t flowVelY = 0;     // Flow velocity Y (cm/s @ 1m)
static uint8_t flowQuality = 0;  // Flow quality
static uint8_t flowStatus = 0;   // Flow status

/* Validity flags */
static bool distanceValid = false;
static bool flowValid = false;

/* Statistics */
static uint32_t rxCount = 0;

/* Estimator data submission */
static float flowStdDev = 0.25f; // Standard deviation for flow
static float tofStdDev = 0.01f;  // Standard deviation for ToF (1cm)

/* Task stack allocation */
STATIC_MEM_TASK_ALLOC(mtf01Task, FLOW_TASK_STACKSIZE);

/**
 * Initialize MTF01 UART
 */
static bool mtf01UartInit(void)
{
    uart_config_t uart_config = {
        .baud_rate = MTF01_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(MTF01_UART_NUM, MTF01_UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        return false;
    }

    err = uart_param_config(MTF01_UART_NUM, &uart_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(err));
        return false;
    }

    err = uart_set_pin(MTF01_UART_NUM, MTF01_TX_PIN, MTF01_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "UART%d initialized: TX=%d, RX=%d, BAUD=%d",
             MTF01_UART_NUM, MTF01_TX_PIN, MTF01_RX_PIN, MTF01_BAUDRATE);

    return true;
}

/**
 * Process parsed MTF01 data and submit to estimator
 * 数据回调函数 - 由驱动层解析完成后调用
 */
static void mtf01DataCallback(const MICOLINK_PAYLOAD_RANGE_SENSOR_t *payload)
{
    uint32_t timestamp = usecTimestamp();

    // Update ToF data
    distance_mm = payload->distance;
    tofStrength = payload->strength;
    tofPrecision = payload->precision;
    tofStatus = payload->tof_status;

    // Validate distance
    distanceValid = (distance_mm >= MTF01_MIN_DISTANCE) &&
                    (distance_mm <= MTF01_MAX_DISTANCE) &&
                    (tofStatus == 0);

    // Update optical flow data
    flowVelX = payload->flow_vel_x;
    flowVelY = payload->flow_vel_y;
    flowQuality = payload->flow_quality;
    flowStatus = payload->flow_status;

    // Validate flow
    flowValid = (flowStatus == 0) && (flowQuality > 0);

    rxCount++;

    // // Debug output
    // printf("%d, %d, %d ,%d, %d,%d\n",
    //        distance_mm, tofStatus, flowVelX, flowVelY, flowStatus, flowQuality);

    // Submit ToF data to estimator
    if (distanceValid)
    {
        tofMeasurement_t tofData = {
            .timestamp = timestamp,
            .distance = (float)distance_mm / 1000.0f, // Convert mm to m
            .stdDev = tofStdDev,
        };
        estimatorEnqueueTOF(&tofData);
    }

    // Submit optical flow data to estimator
    if (flowValid && distanceValid)
    {
        // Convert flow velocity to accumulated pixels
        // MTF01 gives velocity in cm/s @ 1m, need to convert to dpixel format
        // dpixel = velocity * dt / height_scale
        // For now, use velocity directly as dpixel (simplified model)
        float dt = 1.0f / MTF01_TASK_FREQ; // Time between samples
        float height_m = (float)distance_mm / 1000.0f;

        flowMeasurement_t flowData = {
            .timestamp = timestamp,
            .dpixelx = (float)flowVelX * dt * height_m / 100.0f, // Convert cm/s to m/s then to dpixel
            .dpixely = (float)flowVelY * dt * height_m / 100.0f,
            .stdDevX = flowStdDev,
            .stdDevY = flowStdDev,
            .dt = dt,
        };
        estimatorEnqueueFlow(&flowData);
    }
}

/**
 * MTF01 sensor task - reads UART and submits data to estimator
 */
static void mtf01Task(void *param)
{
    uint8_t rxBuffer[MTF01_UART_BUF_SIZE];

    ESP_LOGI(TAG, "Task started");

    // Wait for system to be ready
    systemWaitStart();

    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1)
    {
        // Read available UART data
        int len = uart_read_bytes(MTF01_UART_NUM, rxBuffer,
                                  sizeof(rxBuffer),
                                  pdMS_TO_TICKS(MTF01_READ_TIMEOUT_MS));

        if (len > 0)
        {
            // 使用驱动层 API 处理每个字节
            for (int i = 0; i < len; i++)
            {
                mtf01_process_byte(rxBuffer[i]);
            }
        }

        // Maintain task frequency
        vTaskDelayUntil(&lastWakeTime, M2T(1000 / MTF01_TASK_FREQ));
    }
}

/**
 * Initialize MTF01 task
 */
void mtf01TaskInit(void)
{
    if (isInit)
    {
        return;
    }

    ESP_LOGI(TAG, "Initializing...");

    // 初始化驱动层，注册数据回调
    mtf01_init(mtf01DataCallback);

    // Initialize UART
    if (!mtf01UartInit())
    {
        ESP_LOGE(TAG, "UART initialization failed");
        return;
    }

    // Create task
    STATIC_MEM_TASK_CREATE(mtf01Task, mtf01Task, FLOW_TASK_NAME, NULL, FLOW_TASK_PRI);

    isInit = true;
    ESP_LOGI(TAG, "Initialized successfully");
}

/**
 * Test MTF01 initialization
 */
bool mtf01TaskTest(void)
{
    return isInit;
}

/**
 * Get latest distance measurement
 */
uint32_t mtf01GetDistance(void)
{
    return distance_mm;
}

/**
 * Get flow velocity X
 */
int16_t mtf01GetFlowVelX(void)
{
    return flowVelX;
}

/**
 * Get flow velocity Y
 */
int16_t mtf01GetFlowVelY(void)
{
    return flowVelY;
}

/**
 * Get flow quality
 */
uint8_t mtf01GetFlowQuality(void)
{
    return flowQuality;
}

/**
 * Check distance validity
 */
bool mtf01IsDistanceValid(void)
{
    return distanceValid;
}

/**
 * Check flow validity
 */
bool mtf01IsFlowValid(void)
{
    return flowValid;
}

/* Logging - expose data for cfclient */
LOG_GROUP_START(mtf01)
LOG_ADD(LOG_UINT32, distance, &distance_mm)
LOG_ADD(LOG_INT16, flowVelX, &flowVelX)
LOG_ADD(LOG_INT16, flowVelY, &flowVelY)
LOG_ADD(LOG_UINT8, flowQuality, &flowQuality)
LOG_ADD(LOG_UINT8, tofStatus, &tofStatus)
LOG_ADD(LOG_UINT8, flowStatus, &flowStatus)
LOG_ADD(LOG_UINT32, rxCount, &rxCount)
LOG_GROUP_STOP(mtf01)

/* Parameters - runtime tuning */
PARAM_GROUP_START(mtf01)
PARAM_ADD(PARAM_FLOAT, flowStdDev, &flowStdDev)
PARAM_ADD(PARAM_FLOAT, tofStdDev, &tofStdDev)
PARAM_GROUP_STOP(mtf01)

#else /* CONFIG_ENABLE_MTF01 not defined */

#include "mtf01_task.h"

void mtf01TaskInit(void) {}
bool mtf01TaskTest(void) { return false; }
uint32_t mtf01GetDistance(void) { return 0; }
int16_t mtf01GetFlowVelX(void) { return 0; }
int16_t mtf01GetFlowVelY(void) { return 0; }
uint8_t mtf01GetFlowQuality(void) { return 0; }
bool mtf01IsDistanceValid(void) { return false; }
bool mtf01IsFlowValid(void) { return false; }

#endif /* CONFIG_ENABLE_MTF01 */
