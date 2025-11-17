#ifndef __SPI_CONFIG_H__
#define __SPI_CONFIG_H__

#include "spi_drv.h"

#ifdef __cplusplus
extern "C"
{
#endif

// ======================== SPI总线引脚配置 ========================
// SPI2总线引脚配置（推荐用于外设）
#define SPI_BUS_SCLK_PIN 12 // GPIO12 - 时钟线
#define SPI_BUS_MOSI_PIN 13 // GPIO13 - 主机输出
#define SPI_BUS_MISO_PIN 11 // GPIO11 - 主机输入

// SPI主机选择
#define SPI_BUS_HOST_ID SPI2_HOST
#define SPI_BUS_USE_DMA true
#define SPI_BUS_MAX_TRANSFER 4096

// ======================== CS片选引脚配置 ========================
// BMI088传感器CS引脚
#define BMI088_ACCEL_CS_PIN 10 // GPIO10 - BMI088加速度计CS
#define BMI088_GYRO_CS_PIN 21  // GPIO21 - BMI088陀螺仪CS

// ======================== SPI传输速率配置 ========================
// BMI088传输速率
#define BMI088_SPI_FREQ_HZ SPI_BAUDRATE_10MHZ
#define BMI088_SPI_MODE SPI_MODE_0
#define BMI088_SPI_QUEUE_SIZE 8

    // ======================== 配置获取函数 ========================
    /**
     * @brief 获取SPI总线配置
     * @return SPI总线配置指针
     * @note 使用静态变量避免多次初始化
     */
    static inline spi_drv_bus_config_t *get_spi_bus_config(void)
    {
        static spi_drv_bus_config_t config = {
            .host_id = SPI_BUS_HOST_ID,
            .miso_pin = SPI_BUS_MISO_PIN,
            .mosi_pin = SPI_BUS_MOSI_PIN,
            .sclk_pin = SPI_BUS_SCLK_PIN,
            .use_dma = SPI_BUS_USE_DMA,
            .max_transfer_sz = SPI_BUS_MAX_TRANSFER,
            .is_bus_initialized = false};
        return &config;
    }

    /**
     * @brief 获取BMI088加速度计SPI设备配置
     * @return 设备配置指针
     */
    static inline const spi_drv_device_config_t *get_bmi088_accel_spi_config(void)
    {
        static const spi_drv_device_config_t config = {
            .cs_pin = BMI088_ACCEL_CS_PIN,
            .clock_speed_hz = BMI088_SPI_FREQ_HZ,
            .mode = BMI088_SPI_MODE,
            .queue_size = BMI088_SPI_QUEUE_SIZE};
        return &config;
    }

    /**
     * @brief 获取BMI088陀螺仪SPI设备配置
     * @return 设备配置指针
     */
    static inline const spi_drv_device_config_t *get_bmi088_gyro_spi_config(void)
    {
        static const spi_drv_device_config_t config = {
            .cs_pin = BMI088_GYRO_CS_PIN,
            .clock_speed_hz = BMI088_SPI_FREQ_HZ,
            .mode = BMI088_SPI_MODE,
            .queue_size = BMI088_SPI_QUEUE_SIZE};
        return &config;
    }

#ifdef __cplusplus
}
#endif

#endif /* __SPI_CONFIG_H__ */
