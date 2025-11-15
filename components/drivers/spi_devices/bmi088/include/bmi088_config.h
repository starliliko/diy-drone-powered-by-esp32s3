#ifndef __BMI088_CONFIG_H__
#define __BMI088_CONFIG_H__

#include "bmi088_spi.h"
#include "../../spi_bus/include/spi_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

// ======================== BMI088传感器配置参数 ========================
// 加速度计配置参数 - 修改这里即可改变整个系统的BMI088配置
#define BMI088_CONFIG_ACC_RANGE BMI088_ACC_RANGE_6G
#define BMI088_CONFIG_ACC_ODR BMI088_ACC_ODR_800_HZ
#define BMI088_CONFIG_ACC_BWP BMI088_ACC_BWP_NORMAL
#define BMI088_CONFIG_ACC_POWER BMI088_ACC_PWR_ACTIVE

// 陀螺仪配置参数 - 修改这里即可改变整个系统的BMI088配置
#define BMI088_CONFIG_GYRO_RANGE BMI088_GYRO_RANGE_1000_DPS
#define BMI088_CONFIG_GYRO_BW BMI088_GYRO_BW_116_ODR_1000_HZ
#define BMI088_CONFIG_GYRO_POWER BMI088_GYRO_PWR_NORMAL

    // ======================== 配置管理函数 ========================
    // 使用集中配置初始化BMI088设备
    bool bmi088_init_with_default_config(bmi088_dev_t *dev);

    // 配置辅助函数
    void bmi088_print_device_config(const bmi088_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* __BMI088_CONFIG_H__ */
