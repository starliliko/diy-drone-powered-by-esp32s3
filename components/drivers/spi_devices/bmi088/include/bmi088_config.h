#ifndef __BMI088_CONFIG_H__
#define __BMI088_CONFIG_H__

#include "bmi088_spi.h"
#include "../../spi_bus/include/spi_config.h"

#ifdef __cplusplus
extern "C"
{
#endif
// BMI088外部中断引脚定义
#define BMI088_INT1_PIN CONFIG_BMI088_INT1_PIN // GPIO4 - BMI088加速度计INT1
#define BMI088_INT3_PIN CONFIG_BMI088_INT3_PIN // GPIO5 - BMI088陀螺仪INT3

// BMI088传感器配置参数
#define BMI088_CONFIG_ACC_RANGE BMI088_ACC_RANGE_6G   // 6G量程：精度最高
#define BMI088_CONFIG_ACC_ODR BMI088_ACC_ODR_1600_HZ  // 1600Hz (支持1kHz控制循环)
#define BMI088_CONFIG_ACC_BWP BMI088_ACC_BWP_OSR4     // 优先精度和低噪声
#define BMI088_CONFIG_ACC_POWER BMI088_ACC_PWR_ACTIVE // 激活模式

// 优先精度和低噪声
#define BMI088_CONFIG_GYRO_RANGE BMI088_GYRO_RANGE_2000_DPS  // 2000DPS：避免快速转动时饱和
#define BMI088_CONFIG_GYRO_BW BMI088_GYRO_BW_116_ODR_1000_HZ // 1000Hz ODR，116Hz滤波
#define BMI088_CONFIG_GYRO_POWER BMI088_GYRO_PWR_NORMAL      // 正常功耗模式
    bool bmi088_init_with_default_config(bmi088_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* __BMI088_CONFIG_H__ */
