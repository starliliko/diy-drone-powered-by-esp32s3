#include "bmi088_spi.h"
#include "bmi088_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "BMI088";

// 全局BMI088设备实例
static bmi088_dev_t g_bmi088_dev;
static bool g_is_initialized = false;

// 兼容现有API的包装函数
bool bmi088_init(void)
{
    if (g_is_initialized)
    {
        return true;
    }

    // 使用默认配置初始化
    if (!bmi088_init_default(&g_bmi088_dev))
    {
        ESP_LOGE(TAG, "BMI088 initialization failed");
        return false;
    }

    g_is_initialized = true;
    ESP_LOGI(TAG, "BMI088 initialized successfully");
    return true;
}

bool bmi088_test(void)
{
    if (!g_is_initialized)
    {
        ESP_LOGE(TAG, "BMI088 not initialized");
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

// 配置相关函数
bool bmi088_set_accel_range(bmi088_acc_range_t range)
{
    if (!g_is_initialized)
    {
        return false;
    }

    return bmi088_spi_set_acc_config(&g_bmi088_dev, range,
                                     g_bmi088_dev.config.acc_odr,
                                     g_bmi088_dev.config.acc_bwp);
}

bool bmi088_set_gyro_range(bmi088_gyro_range_t range)
{
    if (!g_is_initialized)
    {
        return false;
    }

    return bmi088_spi_set_gyro_config(&g_bmi088_dev, range,
                                      g_bmi088_dev.config.gyro_bw);
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

// 调试和诊断函数
void bmi088_print_config(void)
{
    if (!g_is_initialized)
    {
        ESP_LOGW(TAG, "BMI088 not initialized");
        return;
    }

    bmi088_full_config_t full_config;
    // 重新构建完整配置用于显示
    full_config.bus_config = *g_bmi088_dev.acc_spi.bus_config;
    full_config.acc_device_config = g_bmi088_dev.acc_spi.device_config;
    full_config.gyro_device_config = g_bmi088_dev.gyro_spi.device_config;
    full_config.sensor_config = g_bmi088_dev.config;

    bmi088_config_print(&full_config);
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
        ESP_LOGI(TAG, "BMI088 deinitialized");
    }
}
