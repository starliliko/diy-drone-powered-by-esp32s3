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
        return false;
    }

    const spi_drv_device_config_t *acc_config = get_bmi088_accel_spi_config();
    const spi_drv_device_config_t *gyro_config = get_bmi088_gyro_spi_config();

    if (!acc_config || !gyro_config)
    {
        return false;
    }

    return bmi088_spi_init(dev, NULL, acc_config, gyro_config);
}
