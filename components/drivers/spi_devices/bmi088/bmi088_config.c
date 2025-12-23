#include "bmi088_config.h"
#include "spi_config.h"
#include <string.h>
#include "esp_log.h"

#define DEBUG_MODULE "BMI088_CFG"
#include "debug_cf.h"

static const char *TAG = "BMI088_CFG";

bool bmi088_init_with_default_config(bmi088_dev_t *dev)
{
    if (!dev)
    {
        DEBUG_PRINT("Invalid device pointer\n");
        return false;
    }

    const spi_drv_device_config_t *acc_config = get_bmi088_accel_spi_config();
    const spi_drv_device_config_t *gyro_config = get_bmi088_gyro_spi_config();

    if (!acc_config || !gyro_config)
    {
        DEBUG_PRINT("Failed to get SPI device configuration\n");
        return false;
    }

    DEBUG_PRINT("Initializing BMI088 with default configuration\n");
    DEBUG_PRINT("ACC CS: %d, GYRO CS: %d, Freq: %d Hz\n",
                acc_config->cs_pin, gyro_config->cs_pin, acc_config->clock_speed_hz);

    // 使用NULL作为总线配置，由驱动层使用全局配置
    bool result = bmi088_spi_init(dev, NULL, acc_config, gyro_config);

    if (!result)
    {
        DEBUG_PRINT("bmi088_spi_init returned false\n");
    }

    return result;
}
