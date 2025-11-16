#ifndef __BMI088_H__
#define __BMI088_H__

#include "bmi088_spi.h"
#include "bmi088_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // ======================== BMI088高级接口函数声明 ========================
    bool bmi088_init(void);
    bool bmi088_test(void);

    bool bmi088_get_accel_data(int16_t *x, int16_t *y, int16_t *z);
    bool bmi088_get_gyro_data(int16_t *x, int16_t *y, int16_t *z);
    float bmi088_get_temperature(void);

    float bmi088_get_accel_scale(void);
    float bmi088_get_gyro_scale(void);

    // 调试和诊断函数
    bool bmi088_soft_reset(void);
    void bmi088_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* __BMI088_H__ */
