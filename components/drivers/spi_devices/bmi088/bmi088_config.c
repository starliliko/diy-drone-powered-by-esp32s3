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
