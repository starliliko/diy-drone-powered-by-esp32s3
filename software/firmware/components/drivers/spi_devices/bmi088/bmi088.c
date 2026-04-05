#include "bmi088_spi.h"
#include "bmi088_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DEBUG_MODULE "BMI088"
#include "debug_cf.h"

// 全局BMI088设备实例
static bmi088_dev_t g_bmi088_dev;
static bool g_is_initialized = false;

bool bmi088_init(void)
{
    if (g_is_initialized)
    {
        return true;
    }

    // 使用初始化接口
    if (!bmi088_init_with_default_config(&g_bmi088_dev))
    {
        DEBUG_PRINTE("BMI088 initialization failed");
        return false;
    }

    g_is_initialized = true;
    DEBUG_PRINTI("BMI088 initialized successfully");
    return true;
}

bool bmi088_test(void)
{
    if (!g_is_initialized)
    {
        DEBUG_PRINTE("BMI088 not initialized");
        return false;
    }

    return bmi088_spi_self_test(&g_bmi088_dev);
}

bool bmi088_get_accel_data(int16_t *x, int16_t *y, int16_t *z)
{
    if (!g_is_initialized || !x || !y || !z)
    {
        return false;
    }

    bmi088_sensor_data_t acc_data;
    if (!bmi088_spi_read_acc_data(&g_bmi088_dev, &acc_data))
    {
        return false;
    }

    *x = acc_data.x;
    *y = acc_data.y;
    *z = acc_data.z;
    return true;
}

bool bmi088_get_gyro_data(int16_t *x, int16_t *y, int16_t *z)
{
    if (!g_is_initialized || !x || !y || !z)
    {
        return false;
    }

    bmi088_sensor_data_t gyro_data;
    if (!bmi088_spi_read_gyro_data(&g_bmi088_dev, &gyro_data))
    {
        return false;
    }

    *x = gyro_data.x;
    *y = gyro_data.y;
    *z = gyro_data.z;
    return true;
}

bool bmi088_get_both_data(int16_t acc[3], int16_t gyro[3])
{
    if (!g_is_initialized || !acc || !gyro)
    {
        return false;
    }

    bmi088_sensor_data_t acc_data, gyro_data;
    if (!bmi088_spi_read_both_data(&g_bmi088_dev, &acc_data, &gyro_data))
    {
        return false;
    }

    acc[0] = acc_data.x;
    acc[1] = acc_data.y;
    acc[2] = acc_data.z;

    gyro[0] = gyro_data.x;
    gyro[1] = gyro_data.y;
    gyro[2] = gyro_data.z;

    return true;
}

float bmi088_get_temperature(void)
{
    if (!g_is_initialized)
    {
        return 0.0f;
    }

    float temperature;
    if (!bmi088_spi_read_temperature(&g_bmi088_dev, &temperature))
    {
        return 0.0f;
    }

    return temperature;
}

// 获取量程转换因子
float bmi088_get_accel_scale(void)
{
    if (!g_is_initialized)
    {
        return 0.0f;
    }

    return bmi088_spi_get_acc_scale(g_bmi088_dev.config.acc_range);
}

float bmi088_get_gyro_scale(void)
{
    if (!g_is_initialized)
    {
        return 0.0f;
    }

    return bmi088_spi_get_gyro_scale(g_bmi088_dev.config.gyro_range);
}

bool bmi088_soft_reset(void)
{
    if (!g_is_initialized)
    {
        return false;
    }

    return bmi088_spi_soft_reset(&g_bmi088_dev);
}

void bmi088_deinit(void)
{
    if (g_is_initialized)
    {
        bmi088_spi_deinit(&g_bmi088_dev);
        g_is_initialized = false;
        DEBUG_PRINTI("BMI088 deinitialized");
    }
}
