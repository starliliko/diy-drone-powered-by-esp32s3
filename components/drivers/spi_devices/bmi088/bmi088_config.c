#include "bmi088_config.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "BMI088_CONFIG";

// 默认SPI总线配置
static void init_default_bus_config(spi_drv_bus_config_t *bus_config)
{
    bus_config->host_id = SPI_DRV_HOST_PRIMARY;
    bus_config->miso_pin = 11; // 根据实际硬件调整
    bus_config->mosi_pin = 13; // 根据实际硬件调整
    bus_config->sclk_pin = 12; // 根据实际硬件调整
    bus_config->use_dma = true;
    bus_config->max_transfer_sz = 4096;
    bus_config->is_bus_initialized = false;
}

bool bmi088_config_init_default(bmi088_full_config_t *full_config)
{
    if (!full_config)
    {
        return false;
    }

    // 初始化默认总线配置
    init_default_bus_config(&full_config->bus_config);

    // 加速度计SPI设备配置
    full_config->acc_device_config.cs_pin = BMI088_DEFAULT_ACC_CS_PIN;
    full_config->acc_device_config.clock_speed_hz = BMI088_DEFAULT_SPI_FREQ_HZ;
    full_config->acc_device_config.mode = BMI088_DEFAULT_SPI_MODE;
    full_config->acc_device_config.queue_size = BMI088_DEFAULT_SPI_QUEUE_SIZE;

    // 陀螺仪SPI设备配置
    full_config->gyro_device_config.cs_pin = BMI088_DEFAULT_GYRO_CS_PIN;
    full_config->gyro_device_config.clock_speed_hz = BMI088_DEFAULT_SPI_FREQ_HZ;
    full_config->gyro_device_config.mode = BMI088_DEFAULT_SPI_MODE;
    full_config->gyro_device_config.queue_size = BMI088_DEFAULT_SPI_QUEUE_SIZE;

    // 默认传感器配置
    full_config->sensor_config.acc_range = BMI088_ACC_RANGE_6G;
    full_config->sensor_config.acc_odr = BMI088_ACC_ODR_800_HZ;
    full_config->sensor_config.acc_bwp = BMI088_ACC_BWP_NORMAL;
    full_config->sensor_config.acc_power = BMI088_ACC_PWR_ACTIVE;
    full_config->sensor_config.gyro_range = BMI088_GYRO_RANGE_1000_DPS;
    full_config->sensor_config.gyro_bw = BMI088_GYRO_BW_116_ODR_1000_HZ;
    full_config->sensor_config.gyro_power = BMI088_GYRO_PWR_NORMAL;

    return true;
}

bool bmi088_config_validate(const bmi088_full_config_t *full_config)
{
    if (!full_config)
    {
        return false;
    }

    // 验证SPI引脚配置
    if (full_config->bus_config.miso_pin < 0 ||
        full_config->bus_config.mosi_pin < 0 ||
        full_config->bus_config.sclk_pin < 0)
    {
        ESP_LOGE(TAG, "Invalid SPI bus pins");
        return false;
    }

    // 验证CS引脚配置
    if (full_config->acc_device_config.cs_pin < 0 ||
        full_config->gyro_device_config.cs_pin < 0)
    {
        ESP_LOGE(TAG, "Invalid CS pins");
        return false;
    }

    // 验证CS引脚不重复
    if (full_config->acc_device_config.cs_pin == full_config->gyro_device_config.cs_pin)
    {
        ESP_LOGE(TAG, "Accelerometer and gyroscope cannot use same CS pin");
        return false;
    }

    return true;
}

bool bmi088_init_default(bmi088_dev_t *dev)
{
    bmi088_full_config_t full_config;

    if (!bmi088_config_init_default(&full_config))
    {
        ESP_LOGE(TAG, "Failed to initialize default configuration");
        return false;
    }

    if (!bmi088_config_validate(&full_config))
    {
        ESP_LOGE(TAG, "Invalid configuration");
        return false;
    }

    ESP_LOGI(TAG, "Initializing BMI088 with default configuration");

    return bmi088_spi_init(dev, &full_config.bus_config,
                           &full_config.acc_device_config,
                           &full_config.gyro_device_config);
}

void bmi088_config_print(const bmi088_full_config_t *full_config)
{
    if (!full_config)
    {
        return;
    }

    ESP_LOGI(TAG, "BMI088 Configuration:");
    ESP_LOGI(TAG, "  SPI Bus:");
    ESP_LOGI(TAG, "    Host: %d", full_config->bus_config.host_id);
    ESP_LOGI(TAG, "    MISO: GPIO%d", full_config->bus_config.miso_pin);
    ESP_LOGI(TAG, "    MOSI: GPIO%d", full_config->bus_config.mosi_pin);
    ESP_LOGI(TAG, "    SCLK: GPIO%d", full_config->bus_config.sclk_pin);
    ESP_LOGI(TAG, "    DMA: %s", full_config->bus_config.use_dma ? "Enabled" : "Disabled");

    ESP_LOGI(TAG, "  Accelerometer:");
    ESP_LOGI(TAG, "    CS Pin: GPIO%d", full_config->acc_device_config.cs_pin);
    ESP_LOGI(TAG, "    SPI Freq: %d Hz", full_config->acc_device_config.clock_speed_hz);
    ESP_LOGI(TAG, "    Range: %d", full_config->sensor_config.acc_range);
    ESP_LOGI(TAG, "    ODR: %d", full_config->sensor_config.acc_odr);

    ESP_LOGI(TAG, "  Gyroscope:");
    ESP_LOGI(TAG, "    CS Pin: GPIO%d", full_config->gyro_device_config.cs_pin);
    ESP_LOGI(TAG, "    SPI Freq: %d Hz", full_config->gyro_device_config.clock_speed_hz);
    ESP_LOGI(TAG, "    Range: %d", full_config->sensor_config.gyro_range);
    ESP_LOGI(TAG, "    BW: %d", full_config->sensor_config.gyro_bw);
}
