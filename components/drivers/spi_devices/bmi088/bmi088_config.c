#include "bmi088_config.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "BMI088_CONFIG";

bool bmi088_init_with_default_config(bmi088_dev_t *dev)
{
    if (!dev)
    {
        ESP_LOGE(TAG, "Invalid device pointer");
        return false;
    }

    // 获取SPI配置
    spi_drv_bus_config_t *bus_config = get_spi_bus_config();
    const spi_drv_device_config_t *acc_config = get_bmi088_accel_spi_config();
    const spi_drv_device_config_t *gyro_config = get_bmi088_gyro_spi_config();

    if (!bus_config || !acc_config || !gyro_config)
    {
        ESP_LOGE(TAG, "Failed to get SPI configuration");
        return false;
    }

    ESP_LOGI(TAG, "Initializing BMI088 with centralized configuration");

    return bmi088_spi_init(dev, bus_config, acc_config, gyro_config);
}

void bmi088_print_device_config(const bmi088_dev_t *dev)
{
    if (!dev || !dev->is_initialized)
    {
        ESP_LOGW(TAG, "BMI088 device not initialized");
        return;
    }

    ESP_LOGI(TAG, "BMI088 Device Configuration:");
    ESP_LOGI(TAG, "  SPI Bus:");
    ESP_LOGI(TAG, "    Host: %d", dev->acc_spi.bus_config->host_id);
    ESP_LOGI(TAG, "    MISO: GPIO%d", dev->acc_spi.bus_config->miso_pin);
    ESP_LOGI(TAG, "    MOSI: GPIO%d", dev->acc_spi.bus_config->mosi_pin);
    ESP_LOGI(TAG, "    SCLK: GPIO%d", dev->acc_spi.bus_config->sclk_pin);
    ESP_LOGI(TAG, "    DMA: %s", dev->acc_spi.bus_config->use_dma ? "Enabled" : "Disabled");

    ESP_LOGI(TAG, "  Accelerometer:");
    ESP_LOGI(TAG, "    CS Pin: GPIO%d", dev->acc_spi.device_config.cs_pin);
    ESP_LOGI(TAG, "    SPI Freq: %d Hz", dev->acc_spi.device_config.clock_speed_hz);
    ESP_LOGI(TAG, "    Range: %d", dev->config.acc_range);
    ESP_LOGI(TAG, "    ODR: %d", dev->config.acc_odr);

    ESP_LOGI(TAG, "  Gyroscope:");
    ESP_LOGI(TAG, "    CS Pin: GPIO%d", dev->gyro_spi.device_config.cs_pin);
    ESP_LOGI(TAG, "    SPI Freq: %d Hz", dev->gyro_spi.device_config.clock_speed_hz);
    ESP_LOGI(TAG, "    Range: %d", dev->config.gyro_range);
    ESP_LOGI(TAG, "    BW: %d", dev->config.gyro_bw);
}
