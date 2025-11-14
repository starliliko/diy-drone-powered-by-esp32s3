#ifndef __BMI088_CONFIG_H__
#define __BMI088_CONFIG_H__

#include "bmi088_spi.h"
#include "spi_drv.h"

#ifdef __cplusplus
extern "C"
{
#endif

// ESP32S3上BMI088的默认SPI配置
#define BMI088_DEFAULT_SPI_FREQ_HZ SPI_BAUDRATE_10MHZ
#define BMI088_DEFAULT_SPI_MODE SPI_MODE_0
#define BMI088_DEFAULT_SPI_QUEUE_SIZE 8

// 默认引脚配置 (可根据硬件设计修改)
#define BMI088_DEFAULT_ACC_CS_PIN 10  // 加速度计CS引脚
#define BMI088_DEFAULT_GYRO_CS_PIN 11 // 陀螺仪CS引脚

    // BMI088完整配置结构
    typedef struct
    {
        // SPI总线配置
        spi_drv_bus_config_t bus_config;

        // 设备SPI配置
        spi_drv_device_config_t acc_device_config;
        spi_drv_device_config_t gyro_device_config;

        // 传感器配置
        bmi088_config_t sensor_config;
    } bmi088_full_config_t;

    // 配置管理函数
    bool bmi088_config_init_default(bmi088_full_config_t *full_config);
    bool bmi088_config_validate(const bmi088_full_config_t *full_config);

    // 便捷初始化函数
    bool bmi088_init_default(bmi088_dev_t *dev);

    // 配置辅助函数
    void bmi088_config_print(const bmi088_full_config_t *full_config);

#ifdef __cplusplus
}
#endif

#endif /* __BMI088_CONFIG_H__ */
