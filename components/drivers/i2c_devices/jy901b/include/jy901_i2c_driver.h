#ifndef JY901_I2C_DRIVER_H
#define JY901_I2C_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "i2cdev.h"
#include "jy901_registers.h"
// 默认设备地址
#define JY901_DEFAULT_ADDR 0x50

// 返回值定义
#define JY901_OK 0
#define JY901_ERROR -1
#define JY901_NOT_FOUND -2

// JY901寄存器地址定义
#define AX 0x34        // X轴加速度
#define AY 0x35        // Y轴加速度
#define AZ 0x36        // Z轴加速度
#define GX 0x37        // X轴角速度
#define GY 0x38        // Y轴角速度
#define GZ 0x39        // Z轴角速度
#define HX 0x3A        // X轴磁场
#define HY 0x3B        // Y轴磁场
#define HZ 0x3C        // Z轴磁场
#define Roll 0x3D      // 横滚角
#define Pitch 0x3E     // 俯仰角
#define Yaw 0x3F       // 偏航角
#define TEMP 0x40      // 温度
#define PressureL 0x45 // 气压低位
#define PressureH 0x46 // 气压高位
#define HeightL 0x47   // 高度低位
#define HeightH 0x48   // 高度高位

// 控制寄存器
#define SAVE 0x00  // 保存设置
#define CALSW 0x01 // 校准
#define RRATE 0x03 // 返回速率
#define KEY 0x69   // 解锁寄存器

// 控制命令
#define SAVE_PARAM 0x00
#define NORMAL 0x00
#define CALGYROACC 0x01 // 校准陀螺仪和加速度计
#define CALMAGMM 0x07   // 校准磁力计
#define KEY_UNLOCK 0x69 // 解锁寄存器

// 数据结构定义
typedef struct
{
    uint8_t device_addr;
    uint32_t timeout_ms;
} jy901_config_t;

typedef struct
{
    float ax; // X轴加速度 (g)
    float ay; // Y轴加速度 (g)
    float az; // Z轴加速度 (g)
} jy901_acc_t;

typedef struct
{
    float gx; // X轴角速度 (°/s)
    float gy; // Y轴角速度 (°/s)
    float gz; // Z轴角速度 (°/s)
} jy901_gyro_t;

typedef struct
{
    float hx; // X轴磁场强度
    float hy; // Y轴磁场强度
    float hz; // Z轴磁场强度
} jy901_mag_t;

typedef struct
{
    float roll;  // 横滚角 (°)
    float pitch; // 俯仰角 (°)
    float yaw;   // 偏航角 (°)
} jy901_angle_t;

typedef struct
{
    float temperature; // 温度 (°C)
} jy901_temp_t;

typedef struct
{
    float pressure; // 气压 (Pa)
    float height;   // 高度 (m)
} jy901_press_t;

typedef struct
{
    jy901_config_t config;
    I2C_Dev *i2c_dev; // 使用 esp-drone 的 I2C 设备
    bool initialized;

    // 原始数据缓存
    int16_t raw_acc[3];
    int16_t raw_gyro[3];
    int16_t raw_mag[3];
    int16_t raw_angle[3];
    int16_t raw_temp;
    int32_t raw_pressure;
    int32_t raw_height;

    // 高度校准相关
    int32_t height_offset;
    bool height_calibrated;
} jy901_handle_t;

// API函数声明
void jy901_get_default_config(jy901_config_t *config);
int jy901_init(jy901_handle_t *handle, const jy901_config_t *config);
int jy901_deinit(jy901_handle_t *handle);
int jy901_check_connection(jy901_handle_t *handle);

// 数据读取函数
int jy901_read_raw_angle(jy901_handle_t *handle, int16_t raw_data[3]);
int jy901_read_angle(jy901_handle_t *handle, jy901_angle_t *angle);
int jy901_read_raw_acc(jy901_handle_t *handle, int16_t raw_data[3]);
int jy901_read_acc(jy901_handle_t *handle, jy901_acc_t *acc);
int jy901_read_raw_gyro(jy901_handle_t *handle, int16_t raw_data[3]);
int jy901_read_gyro(jy901_handle_t *handle, jy901_gyro_t *gyro);
int jy901_read_temp(jy901_handle_t *handle, jy901_temp_t *temp);
int jy901_read_pressure_height_calibrated(jy901_handle_t *handle, jy901_press_t *press);

// 批量读取和缓存访问
int jy901_read_all_raw_data(jy901_handle_t *handle);
int jy901_get_angle_from_cache(jy901_handle_t *handle, jy901_angle_t *angle);
int jy901_get_acc_from_cache(jy901_handle_t *handle, jy901_acc_t *acc);
int jy901_get_gyro_from_cache(jy901_handle_t *handle, jy901_gyro_t *gyro);

// 校准函数
int jy901_start_acc_calibration(jy901_handle_t *handle);
int jy901_start_mag_calibration(jy901_handle_t *handle);
int jy901_stop_mag_calibration(jy901_handle_t *handle);
int jy901_reset_height_zero(jy901_handle_t *handle);

// 配置函数
int jy901_set_output_rate(jy901_handle_t *handle, uint8_t rate);

#endif // JY901_I2C_DRIVER_H