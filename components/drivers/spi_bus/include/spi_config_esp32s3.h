#ifndef __SPI_CONFIG_ESP32S3_H__
#define __SPI_CONFIG_ESP32S3_H__

#include "spi_drv.h"

/*
 * ESP32S3 SPI外设引脚配置
 *
 * 注意：GPIO26-37专用于SPI0/SPI1(Flash/PSRAM)，不可用于外设
 * 外设只能使用SPI2和SPI3，引脚可灵活配置到其他GPIO
 */

// ESP32S3外设SPI推荐引脚配置
#define ESP32S3_SPI_SCLK_PIN 12 // GPIO12 - 时钟
#define ESP32S3_SPI_MOSI_PIN 13 // GPIO13 - 主机输出
#define ESP32S3_SPI_MISO_PIN 11 // GPIO11 - 主机输入

// 外设CS引脚分配（按重要性排序）
#define ESP32S3_BMI088_ACCEL_CS 10 // GPIO10 - BMI088加速度计
#define ESP32S3_BMI088_GYRO_CS 21  // GPIO21 - BMI088陀螺仪
#define ESP32S3_BMP388_CS 47       // GPIO47 - BMP388气压计
#define ESP32S3_PMW3901_CS 48      // GPIO48 - PMW3901光流

// 预留扩展CS引脚
#define ESP32S3_EXPANSION_CS_1 14 // GPIO14
#define ESP32S3_EXPANSION_CS_2 15 // GPIO15

// ESP32S3 SPI2配置（推荐用于外设）
static const spi_drv_bus_config_t esp32s3_spi2_bus_config = {
    .host_id = SPI2_HOST,
    .miso_pin = ESP32S3_SPI_MISO_PIN,
    .mosi_pin = ESP32S3_SPI_MOSI_PIN,
    .sclk_pin = ESP32S3_SPI_SCLK_PIN,
    .use_dma = true,
    .max_transfer_sz = 4096,
    .is_bus_initialized = false};

// BMI088加速度计配置
static const spi_drv_device_config_t esp32s3_bmi088_accel_config = {
    .cs_pin = ESP32S3_BMI088_ACCEL_CS,
    .clock_speed_hz = SPI_BAUDRATE_10MHZ,
    .mode = SPI_MODE_0,
    .queue_size = 8};

// BMI088陀螺仪配置
static const spi_drv_device_config_t esp32s3_bmi088_gyro_config = {
    .cs_pin = ESP32S3_BMI088_GYRO_CS,
    .clock_speed_hz = SPI_BAUDRATE_10MHZ,
    .mode = SPI_MODE_0,
    .queue_size = 8};

// BMP388气压传感器配置
static const spi_drv_device_config_t esp32s3_bmp388_config = {
    .cs_pin = ESP32S3_BMP388_CS,
    .clock_speed_hz = SPI_BAUDRATE_10MHZ,
    .mode = SPI_MODE_0,
    .queue_size = 4};

// PMW3901光流传感器配置
static const spi_drv_device_config_t esp32s3_pmw3901_config = {
    .cs_pin = ESP32S3_PMW3901_CS,
    .clock_speed_hz = SPI_BAUDRATE_2MHZ,
    .mode = SPI_MODE_3,
    .queue_size = 4};

// 获取配置的辅助函数
inline spi_drv_bus_config_t *get_esp32s3_spi2_bus_config(void)
{
    return (spi_drv_bus_config_t *)&esp32s3_spi2_bus_config;
}

inline const spi_drv_device_config_t *get_esp32s3_bmi088_accel_config(void)
{
    return &esp32s3_bmi088_accel_config;
}

inline const spi_drv_device_config_t *get_esp32s3_bmi088_gyro_config(void)
{
    return &esp32s3_bmi088_gyro_config;
}

inline const spi_drv_device_config_t *get_esp32s3_bmp388_config(void)
{
    return &esp32s3_bmp388_config;
}

inline const spi_drv_device_config_t *get_esp32s3_pmw3901_config(void)
{
    return &esp32s3_pmw3901_config;
}

#endif /* __SPI_CONFIG_ESP32S3_H__ */
