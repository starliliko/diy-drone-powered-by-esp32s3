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
#include <math.h>
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

/* 滤波后的光流速度 */
static float filteredFlowVelX = 0.0f;
static float filteredFlowVelY = 0.0f;

/* 滤波参数 */
#define FLOW_DEADZONE 15    // 死区阈值 (cm/s @ 1m)，消除静止噪声
#define FLOW_LPF_ALPHA 0.3f // 低通滤波系数 (0~1, 越小越平滑)
#define FLOW_MAX_CHANGE 100 // 最大变化率限制 (cm/s @ 1m per sample)

/* Estimator data submission (defined in kalman_core.c) */
extern float flowStdDev; // Standard deviation for flow
extern float tofStdDev;  // Standard deviation for ToF (1cm)

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
    // MTF01 tof_status: 0=无效/初始化中, 1=数据有效, 2+=错误
    // 只要距离在有效范围内且 status 不是严重错误即可使用
    distanceValid = (distance_mm >= MTF01_MIN_DISTANCE) &&
                    (distance_mm <= MTF01_MAX_DISTANCE) &&
                    (tofStatus == 1);

    // Update optical flow data
    flowVelX = payload->flow_vel_x;
    flowVelY = payload->flow_vel_y;
    flowQuality = payload->flow_quality;
    flowStatus = payload->flow_status;

    // Validate flow
    // MTF01 flow_status: 0=无效/初始化中, 1=数据有效, 2+=错误
    flowValid = (flowStatus == 1) && (flowQuality > 0);

    // ========== 光流滤波处理 ==========
    if (flowValid)
    {
        float rawX = (float)flowVelX;
        float rawY = (float)flowVelY;

        // 1. 死区滤波 - 消除静止时的小噪声
        if (fabsf(rawX) < FLOW_DEADZONE)
            rawX = 0.0f;
        if (fabsf(rawY) < FLOW_DEADZONE)
            rawY = 0.0f;

        // 2. 限幅滤波 - 限制最大变化率，消除突变尖峰
        float deltaX = rawX - filteredFlowVelX;
        float deltaY = rawY - filteredFlowVelY;
        if (deltaX > FLOW_MAX_CHANGE)
            deltaX = FLOW_MAX_CHANGE;
        if (deltaX < -FLOW_MAX_CHANGE)
            deltaX = -FLOW_MAX_CHANGE;
        if (deltaY > FLOW_MAX_CHANGE)
            deltaY = FLOW_MAX_CHANGE;
        if (deltaY < -FLOW_MAX_CHANGE)
            deltaY = -FLOW_MAX_CHANGE;

        // 3. 低通滤波 - 平滑输出
        filteredFlowVelX = filteredFlowVelX + FLOW_LPF_ALPHA * deltaX;
        filteredFlowVelY = filteredFlowVelY + FLOW_LPF_ALPHA * deltaY;

        // 对滤波结果也应用死区，确保静止时输出为0
        if (fabsf(filteredFlowVelX) < FLOW_DEADZONE * 0.5f)
            filteredFlowVelX = 0.0f;
        if (fabsf(filteredFlowVelY) < FLOW_DEADZONE * 0.5f)
            filteredFlowVelY = 0.0f;
    }

    rxCount++;

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

    // Submit velocity data directly to estimator
    // 使用滤波后的光流速度
    // 坐标轴映射: MTF01 坐标系与飞控坐标系的对应关系
    // 测试结果: 向前移动 flowVelY 为负，向左移动 flowVelX 为负
    // 正确映射 (交换+取反):
    // - 飞控 velX (前进正) = -flowVelY (向前flowVelY负 -> velX正)
    // - 飞控 velY (向左正) = -flowVelX (向左flowVelX负 -> velY正)
    if (flowValid && distanceValid)
    {
        float height_m = (float)distance_mm / 1000.0f;

        velocityMeasurement_t velData = {
            .timestamp = timestamp,
            .velX = -filteredFlowVelY * height_m / 100.0f, // 交换并取反
            .velY = -filteredFlowVelX * height_m / 100.0f, // 交换并取反
            .stdDev = flowStdDev,
        };
        estimatorEnqueueVelocity(&velData);
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

#else /* CONFIG_ENABLE_MTF01 not defined */

#include "flow_mtf01.h"

void mtf01TaskInit(void) {}
bool mtf01TaskTest(void) { return false; }
uint32_t mtf01GetDistance(void) { return 0; }
int16_t mtf01GetFlowVelX(void) { return 0; }
int16_t mtf01GetFlowVelY(void) { return 0; }
uint8_t mtf01GetFlowQuality(void) { return 0; }
bool mtf01IsDistanceValid(void) { return false; }
bool mtf01IsFlowValid(void) { return false; }

#endif /* CONFIG_ENABLE_MTF01 */
