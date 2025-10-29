#ifndef __JY901_I2C_DRIVER_H
#define __JY901_I2C_DRIVER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdio.h>
#include "driver/i2c.h"
#include "jy901_registers.h"

// 错误代码定义
#define JY901_OK 0
#define JY901_ERROR -1
#define JY901_TIMEOUT -2
#define JY901_NOT_FOUND -3
#define JY901_INVALID_ADDR -4

// 默认I2C配置
#define JY901_DEFAULT_ADDR 0x50
#define JY901_I2C_FREQ 400000 // 400kHz - 提升通讯速率
#define JY901_TIMEOUT_MS 1000

    // 角度数据结构
    typedef struct
    {
        float roll;  // 横滚角 (度)
        float pitch; // 俯仰角 (度)
        float yaw;   // 偏航角 (度)
    } jy901_angle_t;

    // 加速度计数据结构
    typedef struct
    {
        float ax; // X轴加速度 (g)
        float ay; // Y轴加速度 (g)
        float az; // Z轴加速度 (g)
    } jy901_acc_t;

    // 陀螺仪数据结构
    typedef struct
    {
        float gx; // X轴角速度 (°/s)
        float gy; // Y轴角速度 (°/s)
        float gz; // Z轴角速度 (°/s)
    } jy901_gyro_t;

    // 磁力计数据结构
    typedef struct
    {
        float hx; // X轴磁场强度
        float hy; // Y轴磁场强度
        float hz; // Z轴磁场强度
    } jy901_mag_t;

    // 温度数据结构
    typedef struct
    {
        float temperature; // 温度 (℃)
    } jy901_temp_t;

    // 气压高度数据结构
    typedef struct
    {
        float pressure; // 气压 (Pa)
        float height;   // 高度 (m)
    } jy901_press_t;

    // 传感器配置结构
    typedef struct
    {
        uint8_t i2c_port;    // I2C端口号
        uint8_t device_addr; // 设备I2C地址
        uint8_t sda_pin;     // SDA引脚
        uint8_t scl_pin;     // SCL引脚
        uint32_t i2c_freq;   // I2C频率
        uint32_t timeout_ms; // 超时时间(ms)
    } jy901_config_t;

    // 驱动句柄结构
    typedef struct
    {
        jy901_config_t config;
        bool initialized;
        int16_t raw_angle[3];   // 原始角度数据 [Roll, Pitch, Yaw]
        int16_t raw_acc[3];     // 原始加速度数据 [AX, AY, AZ]
        int16_t raw_gyro[3];    // 原始陀螺仪数据 [GX, GY, GZ]
        int16_t raw_mag[3];     // 原始磁力计数据 [HX, HY, HZ]
        int16_t raw_temp;       // 原始温度数据
        int32_t raw_pressure;   // 原始气压数据 (32位)
        int32_t raw_height;     // 原始高度数据 (32位)
        int32_t height_offset;  // 高度偏移量，用于高度置零
        bool height_calibrated; // 高度是否已校准标志
    } jy901_handle_t;

    /**
     * @brief 初始化JY901传感器
     * @param handle 驱动句柄指针
     * @param config 配置参数指针
     * @return JY901_OK 成功, 其他值为错误码
     */
    int jy901_init(jy901_handle_t *handle, const jy901_config_t *config);

    /**
     * @brief 反初始化JY901传感器
     * @param handle 驱动句柄指针
     * @return JY901_OK 成功, 其他值为错误码
     */
    int jy901_deinit(jy901_handle_t *handle);

    /**
     * @brief 自动扫描传感器地址
     * @param handle 驱动句柄指针
     * @return JY901_OK 成功找到设备, 其他值为错误码
     */
    int jy901_auto_scan(jy901_handle_t *handle);

    /**
     * @brief 读取角度数据
     * @param handle 驱动句柄指针
     * @param angle 角度数据结构指针
     * @return JY901_OK 成功, 其他值为错误码
     */
    int jy901_read_angle(jy901_handle_t *handle, jy901_angle_t *angle);

    /**
     * @brief 读取原始角度数据
     * @param handle 驱动句柄指针
     * @param raw_data 原始数据数组[3] (Roll, Pitch, Yaw)
     * @return JY901_OK 成功, 其他值为错误码
     */
    int jy901_read_raw_angle(jy901_handle_t *handle, int16_t raw_data[3]);

    /**
     * @brief 设置输出频率
     * @param handle 驱动句柄指针
     * @param rate 输出频率 (RRATE_*定义)
     * @return JY901_OK 成功, 其他值为错误码
     */
    int jy901_set_output_rate(jy901_handle_t *handle, uint8_t rate);

    /**
     * @brief 开始加速度计校准
     * @param handle 驱动句柄指针
     * @return JY901_OK 成功, 其他值为错误码
     */
    int jy901_start_acc_calibration(jy901_handle_t *handle);

    /**
     * @brief 开始磁力计校准
     * @param handle 驱动句柄指针
     * @return JY901_OK 成功, 其他值为错误码
     */
    int jy901_start_mag_calibration(jy901_handle_t *handle);

    /**
     * @brief 停止磁力计校准
     * @param handle 驱动句柄指针
     * @return JY901_OK 成功, 其他值为错误码
     */
    int jy901_stop_mag_calibration(jy901_handle_t *handle);

    /**
     * @brief 检查设备连接状态
     * @param handle 驱动句柄指针
     * @return JY901_OK 设备连接正常, 其他值为错误码
     */
    int jy901_check_connection(jy901_handle_t *handle);

    /**
     * @brief 获取默认配置
     * @param config 配置结构指针
     */
    void jy901_get_default_config(jy901_config_t *config);

    /**
     * @brief 读取原始加速度数据
     * @param handle 驱动句柄指针
     * @param raw_data 原始数据数组[3] (AX, AY, AZ)
     * @return JY901_OK 成功, 其他值为错误码
     */
    int jy901_read_raw_acc(jy901_handle_t *handle, int16_t raw_data[3]);

    /**
     * @brief 读取加速度数据
     * @param handle 驱动句柄指针
     * @param acc 加速度数据结构指针
     * @return JY901_OK 成功, 其他值为错误码
     */
    int jy901_read_acc(jy901_handle_t *handle, jy901_acc_t *acc);

    /**
     * @brief 读取原始陀螺仪数据
     * @param handle 驱动句柄指针
     * @param raw_data 原始数据数组[3] (GX, GY, GZ)
     * @return JY901_OK 成功, 其他值为错误码
     */
    int jy901_read_raw_gyro(jy901_handle_t *handle, int16_t raw_data[3]);

    /**
     * @brief 读取陀螺仪数据
     * @param handle 驱动句柄指针
     * @param gyro 陀螺仪数据结构指针
     * @return JY901_OK 成功, 其他值为错误码
     */
    int jy901_read_gyro(jy901_handle_t *handle, jy901_gyro_t *gyro);

    /**
     * @brief 读取原始磁力计数据
     * @param handle 驱动句柄指针
     * @param raw_data 原始数据数组[3] (HX, HY, HZ)
     * @return JY901_OK 成功, 其他值为错误码
     */
    int jy901_read_raw_mag(jy901_handle_t *handle, int16_t raw_data[3]);

    /**
     * @brief 读取磁力计数据
     * @param handle 驱动句柄指针
     * @param mag 磁力计数据结构指针
     * @return JY901_OK 成功, 其他值为错误码
     */
    int jy901_read_mag(jy901_handle_t *handle, jy901_mag_t *mag);

    /**
     * @brief 读取原始温度数据
     * @param handle 驱动句柄指针
     * @param raw_temp 原始温度数据指针
     * @return JY901_OK 成功, 其他值为错误码
     */
    int jy901_read_raw_temp(jy901_handle_t *handle, int16_t *raw_temp);

    /**
     * @brief 读取温度数据
     * @param handle 驱动句柄指针
     * @param temp 温度数据结构指针
     * @return JY901_OK 成功, 其他值为错误码
     */
    int jy901_read_temp(jy901_handle_t *handle, jy901_temp_t *temp);

    /**
     * @brief 读取原始气压数据
     * @param handle 驱动句柄指针
     * @param raw_pressure 原始气压数据指针 (32位)
     * @return JY901_OK 成功, 其他值为错误码
     */
    int jy901_read_raw_pressure(jy901_handle_t *handle, int32_t *raw_pressure);

    /**
     * @brief 读取原始高度数据
     * @param handle 驱动句柄指针
     * @param raw_height 原始高度数据指针 (32位)
     * @return JY901_OK 成功, 其他值为错误码
     */
    int jy901_read_raw_height(jy901_handle_t *handle, int32_t *raw_height);

    /**
     * @brief 读取原始气压高度数据
     * @param handle 驱动句柄指针
     * @param raw_pressure 原始气压数据指针
     * @param raw_height 原始高度数据指针
     * @return JY901_OK 成功, 其他值为错误码
     */
    int jy901_read_raw_pressure_height(jy901_handle_t *handle, int32_t *raw_pressure, int32_t *raw_height);

    /**
     * @brief 读取气压高度数据(带高度校准)
     * @param handle 驱动句柄指针
     * @param press 气压高度数据结构指针
     * @return JY901_OK 成功, 其他值为错误码
     * @note 气压为原始值，高度为校准后的值
     */
    int jy901_read_pressure_height_calibrated(jy901_handle_t *handle, jy901_press_t *press);

    /**
     * @brief 读取所有传感器原始数据
     * @param handle 驱动句柄指针
     * @return JY901_OK 成功, 其他值为错误码
     * @note 一次性读取AX到HeightH的所有寄存器数据，提升I2C通信效率
     */
    int jy901_read_all_raw_data(jy901_handle_t *handle);

    /**
     * @brief 高度置零操作
     * @param handle 驱动句柄指针
     * @return JY901_OK 成功, 其他值为错误码
     * @note 将当前高度设置为0米基准点，后续高度读取将基于此点计算
     */
    int jy901_reset_height_zero(jy901_handle_t *handle);

    /**
     * @brief 清除高度校准
     * @param handle 驱动句柄指针
     * @return JY901_OK 成功, 其他值为错误码
     */
    int jy901_clear_height_calibration(jy901_handle_t *handle);

    /**
     * @brief 检查高度是否已校准
     * @param handle 驱动句柄指针
     * @return true 已校准, false 未校准
     */
    bool jy901_is_height_calibrated(jy901_handle_t *handle);

    /**
     * @brief 从缓存获取角度数据
     * @param handle 驱动句柄指针
     * @param angle 角度数据结构指针
     * @return JY901_OK 成功, 其他值为错误码
     * @note 需要先调用jy901_read_all_raw_data()更新缓存
     */
    int jy901_get_angle_from_cache(jy901_handle_t *handle, jy901_angle_t *angle);

    /**
     * @brief 从缓存获取气压高度数据
     * @param handle 驱动句柄指针
     * @param press 气压高度数据结构指针
     * @return JY901_OK 成功, 其他值为错误码
     * @note 需要先调用jy901_read_all_raw_data()更新缓存，高度自动应用校准
     */
    int jy901_get_pressure_height_from_cache(jy901_handle_t *handle, jy901_press_t *press);

    /**
     * @brief 从缓存获取加速度数据
     * @param handle 驱动句柄指针
     * @param acc 加速度数据结构指针
     * @return JY901_OK 成功, 其他值为错误码
     * @note 需要先调用jy901_read_all_raw_data()更新缓存
     */
    int jy901_get_acc_from_cache(jy901_handle_t *handle, jy901_acc_t *acc);

    /**
     * @brief 从缓存获取陀螺仪数据
     * @param handle 驱动句柄指针
     * @param gyro 陀螺仪数据结构指针
     * @return JY901_OK 成功, 其他值为错误码
     * @note 需要先调用jy901_read_all_raw_data()更新缓存
     */
    int jy901_get_gyro_from_cache(jy901_handle_t *handle, jy901_gyro_t *gyro);

    /**
     * @brief 从缓存获取温度数据
     * @param handle 驱动句柄指针
     * @param temp 温度数据结构指针
     * @return JY901_OK 成功, 其他值为错误码
     * @note 需要先调用jy901_read_all_raw_data()更新缓存
     */
    int jy901_get_temp_from_cache(jy901_handle_t *handle, jy901_temp_t *temp);

#ifdef __cplusplus
}
#endif

#endif // __JY901_I2C_DRIVER_H