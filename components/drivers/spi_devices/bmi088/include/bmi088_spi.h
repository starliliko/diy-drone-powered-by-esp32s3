#ifndef __BMI088_SPI_H__
#define __BMI088_SPI_H__

#include <stdint.h>
#include <stdbool.h>
#include "../../spi_bus/include/spi_drv.h"

#ifdef __cplusplus
extern "C"
{
#endif

// BMI088加速度计寄存器地址
#define BMI088_ACC_CHIP_ID_REG 0x00
#define BMI088_ACC_ERR_REG 0x02
#define BMI088_ACC_STATUS_REG 0x03
#define BMI088_ACC_X_LSB_REG 0x12
#define BMI088_ACC_X_MSB_REG 0x13
#define BMI088_ACC_Y_LSB_REG 0x14
#define BMI088_ACC_Y_MSB_REG 0x15
#define BMI088_ACC_Z_LSB_REG 0x16
#define BMI088_ACC_Z_MSB_REG 0x17
#define BMI088_ACC_SENSORTIME_0_REG 0x18
#define BMI088_ACC_SENSORTIME_1_REG 0x19
#define BMI088_ACC_SENSORTIME_2_REG 0x1A
#define BMI088_ACC_INT_STAT_1_REG 0x1D
#define BMI088_ACC_TEMP_MSB_REG 0x22
#define BMI088_ACC_TEMP_LSB_REG 0x23
#define BMI088_ACC_CONF_REG 0x40
#define BMI088_ACC_RANGE_REG 0x41
#define BMI088_ACC_INT1_IO_CTRL_REG 0x53
#define BMI088_ACC_INT_MAP_DATA_REG 0x58
#define BMI088_ACC_PWR_CONF_REG 0x7C
#define BMI088_ACC_PWR_CTRL_REG 0x7D
#define BMI088_ACC_SOFTRESET_REG 0x7E

// BMI088陀螺仪寄存器地址
#define BMI088_GYRO_CHIP_ID_REG 0x00
#define BMI088_GYRO_X_LSB_REG 0x02
#define BMI088_GYRO_X_MSB_REG 0x03
#define BMI088_GYRO_Y_LSB_REG 0x04
#define BMI088_GYRO_Y_MSB_REG 0x05
#define BMI088_GYRO_Z_LSB_REG 0x06
#define BMI088_GYRO_Z_MSB_REG 0x07
#define BMI088_GYRO_INT_STAT_1_REG 0x0A
#define BMI088_GYRO_RANGE_REG 0x0F
#define BMI088_GYRO_BANDWIDTH_REG 0x10
#define BMI088_GYRO_LPM1_REG 0x11
#define BMI088_GYRO_SOFTRESET_REG 0x14
#define BMI088_GYRO_INT_CTRL_REG 0x15
#define BMI088_GYRO_INT3_INT4_IO_CONF_REG 0x16
#define BMI088_GYRO_INT3_INT4_IO_MAP_REG 0x18

// 芯片ID值
#define BMI088_ACC_CHIP_ID_VALUE 0x1E
#define BMI088_GYRO_CHIP_ID_VALUE 0x0F

    // 加速度计量程配置
    typedef enum
    {
        BMI088_ACC_RANGE_3G = 0x00,
        BMI088_ACC_RANGE_6G = 0x01,
        BMI088_ACC_RANGE_12G = 0x02,
        BMI088_ACC_RANGE_24G = 0x03
    } bmi088_acc_range_t;

    // 加速度计输出数据率配置
    typedef enum
    {
        BMI088_ACC_ODR_12_5_HZ = 0x05,
        BMI088_ACC_ODR_25_HZ = 0x06,
        BMI088_ACC_ODR_50_HZ = 0x07,
        BMI088_ACC_ODR_100_HZ = 0x08,
        BMI088_ACC_ODR_200_HZ = 0x09,
        BMI088_ACC_ODR_400_HZ = 0x0A,
        BMI088_ACC_ODR_800_HZ = 0x0B,
        BMI088_ACC_ODR_1600_HZ = 0x0C
    } bmi088_acc_odr_t;

    // 加速度计带宽配置
    typedef enum
    {
        BMI088_ACC_BWP_OSR4 = 0x00,
        BMI088_ACC_BWP_OSR2 = 0x01,
        BMI088_ACC_BWP_NORMAL = 0x02
    } bmi088_acc_bwp_t;

    // 陀螺仪量程配置
    typedef enum
    {
        BMI088_GYRO_RANGE_2000_DPS = 0x00,
        BMI088_GYRO_RANGE_1000_DPS = 0x01,
        BMI088_GYRO_RANGE_500_DPS = 0x02,
        BMI088_GYRO_RANGE_250_DPS = 0x03,
        BMI088_GYRO_RANGE_125_DPS = 0x04
    } bmi088_gyro_range_t;

    // 陀螺仪输出数据率和带宽配置
    typedef enum
    {
        BMI088_GYRO_BW_532_ODR_2000_HZ = 0x00,
        BMI088_GYRO_BW_230_ODR_2000_HZ = 0x01,
        BMI088_GYRO_BW_116_ODR_1000_HZ = 0x02,
        BMI088_GYRO_BW_47_ODR_400_HZ = 0x03,
        BMI088_GYRO_BW_23_ODR_200_HZ = 0x04,
        BMI088_GYRO_BW_12_ODR_100_HZ = 0x05,
        BMI088_GYRO_BW_64_ODR_200_HZ = 0x06,
        BMI088_GYRO_BW_32_ODR_100_HZ = 0x07
    } bmi088_gyro_bw_t;

    // 电源模式
    typedef enum
    {
        BMI088_ACC_PWR_ACTIVE = 0x00,
        BMI088_ACC_PWR_SUSPEND = 0x03
    } bmi088_acc_power_t;

    typedef enum
    {
        BMI088_GYRO_PWR_NORMAL = 0x00,
        BMI088_GYRO_PWR_SUSPEND = 0x80,
        BMI088_GYRO_PWR_DEEP_SUSPEND = 0x20
    } bmi088_gyro_power_t;

    // BMI088传感器数据结构
    typedef struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
    } bmi088_sensor_data_t;

    // BMI088配置结构
    typedef struct
    {
        // 加速度计配置
        bmi088_acc_range_t acc_range;
        bmi088_acc_odr_t acc_odr;
        bmi088_acc_bwp_t acc_bwp;
        bmi088_acc_power_t acc_power;

        // 陀螺仪配置
        bmi088_gyro_range_t gyro_range;
        bmi088_gyro_bw_t gyro_bw;
        bmi088_gyro_power_t gyro_power;
    } bmi088_config_t;

    // BMI088驱动结构
    typedef struct
    {
        spi_drv_t acc_spi;      // 加速度计SPI实例
        spi_drv_t gyro_spi;     // 陀螺仪SPI实例
        bmi088_config_t config; // 传感器配置
        bool is_initialized;    // 初始化标志
    } bmi088_dev_t;

    // BMI088驱动函数声明
    bool bmi088_spi_init(bmi088_dev_t *dev,
                         spi_drv_bus_config_t *bus_config,
                         const spi_drv_device_config_t *acc_device_config,
                         const spi_drv_device_config_t *gyro_device_config);

    bool bmi088_spi_deinit(bmi088_dev_t *dev);

    // 基础读写函数
    bool bmi088_acc_read_reg(bmi088_dev_t *dev, uint8_t reg_addr, uint8_t *data, size_t length);
    bool bmi088_acc_write_reg(bmi088_dev_t *dev, uint8_t reg_addr, const uint8_t *data, size_t length);
    bool bmi088_gyro_read_reg(bmi088_dev_t *dev, uint8_t reg_addr, uint8_t *data, size_t length);
    bool bmi088_gyro_write_reg(bmi088_dev_t *dev, uint8_t reg_addr, const uint8_t *data, size_t length);

    // 配置函数
    bool bmi088_spi_configure(bmi088_dev_t *dev, const bmi088_config_t *config);
    bool bmi088_spi_set_acc_config(bmi088_dev_t *dev, bmi088_acc_range_t range,
                                   bmi088_acc_odr_t odr, bmi088_acc_bwp_t bwp);
    bool bmi088_spi_set_gyro_config(bmi088_dev_t *dev, bmi088_gyro_range_t range, bmi088_gyro_bw_t bw);
    bool bmi088_spi_set_power_mode(bmi088_dev_t *dev, bmi088_acc_power_t acc_power, bmi088_gyro_power_t gyro_power);

    // 数据读取函数
    bool bmi088_spi_read_acc_data(bmi088_dev_t *dev, bmi088_sensor_data_t *acc_data);
    bool bmi088_spi_read_gyro_data(bmi088_dev_t *dev, bmi088_sensor_data_t *gyro_data);
    bool bmi088_spi_read_both_data(bmi088_dev_t *dev, bmi088_sensor_data_t *acc_data, bmi088_sensor_data_t *gyro_data);

    // 芯片ID检测
    bool bmi088_spi_check_chip_id(bmi088_dev_t *dev);

    // 软复位
    bool bmi088_spi_soft_reset(bmi088_dev_t *dev);

    // 自测试
    bool bmi088_spi_self_test(bmi088_dev_t *dev);

    // 温度读取
    bool bmi088_spi_read_temperature(bmi088_dev_t *dev, float *temperature);

    // 获取量程比例因子
    float bmi088_spi_get_acc_scale(bmi088_acc_range_t range);
    float bmi088_spi_get_gyro_scale(bmi088_gyro_range_t range);

#ifdef __cplusplus
}
#endif

#endif /* __BMI088_SPI_H__ */
