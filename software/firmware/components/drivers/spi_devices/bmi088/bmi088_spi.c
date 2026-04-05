#include "bmi088_spi.h"
#include "bmi088_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"

#define DEBUG_MODULE "BMI088_SPI"
#include "debug_cf.h"

static const char *TAG = "BMI088_SPI";

// SPI读写标志位
#define BMI088_SPI_READ_FLAG 0x80
#define BMI088_SPI_WRITE_FLAG 0x00

// 延时宏定义
#define BMI088_DELAY_MS(ms) vTaskDelay(pdMS_TO_TICKS(ms))

static bool bmi088_acc_configure_data_ready_interrupt(bmi088_dev_t *dev);
static bool bmi088_gyro_configure_data_ready_interrupt(bmi088_dev_t *dev);

// 量程比例因子表
static const float acc_scale_table[] = {
    [BMI088_ACC_RANGE_3G] = 3.0f / 32768.0f,
    [BMI088_ACC_RANGE_6G] = 6.0f / 32768.0f,
    [BMI088_ACC_RANGE_12G] = 12.0f / 32768.0f,
    [BMI088_ACC_RANGE_24G] = 24.0f / 32768.0f};

static const float gyro_scale_table[] = {
    [BMI088_GYRO_RANGE_2000_DPS] = 2000.0f / 32768.0f,
    [BMI088_GYRO_RANGE_1000_DPS] = 1000.0f / 32768.0f,
    [BMI088_GYRO_RANGE_500_DPS] = 500.0f / 32768.0f,
    [BMI088_GYRO_RANGE_250_DPS] = 250.0f / 32768.0f,
    [BMI088_GYRO_RANGE_125_DPS] = 125.0f / 32768.0f};

bool bmi088_spi_init(bmi088_dev_t *dev,
                     spi_drv_bus_config_t *bus_config,
                     const spi_drv_device_config_t *acc_device_config,
                     const spi_drv_device_config_t *gyro_device_config)
{
    if (!dev || !acc_device_config || !gyro_device_config)
    {
        return false;
    }

    // 初始化加速度计SPI设备（bus_config可以为NULL，使用全局配置）
    if (!spiDrvDeviceInit(&dev->acc_spi, bus_config, acc_device_config))
    {
        DEBUG_PRINT("Failed to initialize accelerometer SPI\n");
        return false;
    }

    // 初始化陀螺仪SPI设备
    if (!spiDrvDeviceInit(&dev->gyro_spi, bus_config, gyro_device_config))
    {
        DEBUG_PRINT("Failed to initialize gyroscope SPI\n");
        spiDrvDeinit(&dev->acc_spi);
        return false;
    }

    // 激活SPI模式：加速度计上电后处于I2C模式，需读取两次寄存器来切换到SPI模式
    uint8_t dummy;
    bmi088_acc_read_reg(dev, BMI088_ACC_CHIP_ID_REG, &dummy, 1);
    BMI088_DELAY_MS(2);
    bmi088_acc_read_reg(dev, BMI088_ACC_CHIP_ID_REG, &dummy, 1);
    BMI088_DELAY_MS(2);

    // 步骤1: 软复位
    if (!bmi088_spi_soft_reset(dev))
    {
        DEBUG_PRINT("Failed to reset BMI088\n");
        bmi088_spi_deinit(dev);
        return false;
    }

    BMI088_DELAY_MS(50); // 等待IMU完全就绪

    // 步骤2: 检查芯片ID
    if (!bmi088_spi_check_chip_id(dev))
    {
        DEBUG_PRINT("Chip ID verification failed\n");
        bmi088_spi_deinit(dev);
        return false;
    }

    // 步骤3: 设置电源模式
    bmi088_config_t sensor_config = {
        .acc_range = BMI088_CONFIG_ACC_RANGE,
        .acc_odr = BMI088_CONFIG_ACC_ODR,
        .acc_bwp = BMI088_CONFIG_ACC_BWP,
        .acc_power = BMI088_CONFIG_ACC_POWER,
        .gyro_range = BMI088_CONFIG_GYRO_RANGE,
        .gyro_bw = BMI088_CONFIG_GYRO_BW,
        .gyro_power = BMI088_CONFIG_GYRO_POWER};

    if (!bmi088_spi_set_power_mode(dev, sensor_config.acc_power, sensor_config.gyro_power))
    {
        DEBUG_PRINT("Power mode configuration failed\n");
        bmi088_spi_deinit(dev);
        return false;
    }

    // 步骤4: 设置传感器配置（量程、ODR等）
    if (!bmi088_spi_configure(dev, &sensor_config))
    {

        DEBUG_PRINT("Sensor configuration failed\n");
        bmi088_spi_deinit(dev);
        return false;
    }

    // 步骤5: 配置数据就绪中断
    if (!bmi088_acc_configure_data_ready_interrupt(dev))
    {
        DEBUG_PRINT("ACC interrupt configuration failed\n");
        bmi088_spi_deinit(dev);
        return false;
    }

    if (!bmi088_gyro_configure_data_ready_interrupt(dev))
    {
        DEBUG_PRINT("GYRO interrupt configuration failed\n");
        bmi088_spi_deinit(dev);
        return false;
    }

    // 步骤6: 等待数据采集稳定
    BMI088_DELAY_MS(50);

    dev->is_initialized = true;
    return true;
}

bool bmi088_spi_deinit(bmi088_dev_t *dev)
{
    if (!dev)
    {
        return false;
    }

    spiDrvDeinit(&dev->acc_spi);
    spiDrvDeinit(&dev->gyro_spi);
    dev->is_initialized = false;
    return true;
}

bool bmi088_acc_read_reg(bmi088_dev_t *dev, uint8_t reg_addr, uint8_t *data, size_t length)
{
    if (!dev)
    {
        return false;
    }

    // BMI088加速度计需要发送虚拟字节
    uint8_t tx_buf[length + 2];
    uint8_t rx_buf[length + 2];

    tx_buf[0] = reg_addr | BMI088_SPI_READ_FLAG;
    memset(&tx_buf[1], 0x00, length + 1); // 虚拟字节 + 数据字节

    spi_drv_transfer_t transfer = {
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
        .length = length + 2,
        .is_write = false,
        .flags = 0};

    if (!spiDrvTransfer(&dev->acc_spi, &transfer))
    {
        return false;
    }

    memcpy(data, &rx_buf[2], length); // BMI088 ACC: 虚拟字节后数据从第三个字节开始
    return true;
}

bool bmi088_acc_write_reg(bmi088_dev_t *dev, uint8_t reg_addr, const uint8_t *data, size_t length)
{
    if (!dev)
    {
        return false;
    }

    return spiDrvWriteReg(&dev->acc_spi, reg_addr | BMI088_SPI_WRITE_FLAG, data, length);
}

bool bmi088_gyro_read_reg(bmi088_dev_t *dev, uint8_t reg_addr, uint8_t *data, size_t length)
{
    if (!dev)
    {
        return false;
    }

    // 陀螺仪不需要虚拟字节
    uint8_t tx_buf[length + 1];
    uint8_t rx_buf[length + 1];

    tx_buf[0] = reg_addr | BMI088_SPI_READ_FLAG;
    memset(&tx_buf[1], 0x00, length);

    spi_drv_transfer_t transfer = {
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
        .length = length + 1,
        .is_write = false,
        .flags = 0};

    if (spiDrvTransfer(&dev->gyro_spi, &transfer))
    {
        memcpy(data, &rx_buf[1], length);
        return true;
    }
    return false;
}

bool bmi088_gyro_write_reg(bmi088_dev_t *dev, uint8_t reg_addr, const uint8_t *data, size_t length)
{
    if (!dev)
    {
        return false;
    }

    return spiDrvWriteReg(&dev->gyro_spi, reg_addr | BMI088_SPI_WRITE_FLAG, data, length);
}

bool bmi088_spi_configure(bmi088_dev_t *dev, const bmi088_config_t *config)
{
    if (!dev || !config)
    {
        return false;
    }

    // 保存配置
    memcpy(&dev->config, config, sizeof(bmi088_config_t));

    // 配置加速度计
    if (!bmi088_spi_set_acc_config(dev, config->acc_range, config->acc_odr, config->acc_bwp))
    {
        return false;
    }

    // 配置陀螺仪
    if (!bmi088_spi_set_gyro_config(dev, config->gyro_range, config->gyro_bw))
    {
        return false;
    }

    DEBUG_PRINT("BMI088 configuration completed\n");
    return true;
}

bool bmi088_spi_set_acc_config(bmi088_dev_t *dev, bmi088_acc_range_t range,
                               bmi088_acc_odr_t odr, bmi088_acc_bwp_t bwp)
{
    if (!dev)
    {
        return false;
    }

    // 【关键】严格按照BMI088数据手册的上电顺序：
    // 1. PWR_CONF (0x7C) = 0x00 退出Suspend模式（必须先退出suspend才能配置！）
    // 2. 等待450us以上
    // 3. PWR_CTRL (0x7D) = 0x04 使能加速度计
    // 4. 等待50ms（让模拟部分完全稳定）
    // 5. ACC_CONF (0x40) = ... 配置采样率
    // 6. ACC_RANGE (0x41) = ... 配置量程

    // Step 1: 先退出Suspend模式（PWR_CONF=0x00）
    uint8_t pwr_conf = 0x00;
    bmi088_acc_write_reg(dev, BMI088_ACC_PWR_CONF_REG, &pwr_conf, 1);
    BMI088_DELAY_MS(1); // 等待450us以上

    // Step 2: 使能加速度计（PWR_CTRL=0x04）
    uint8_t pwr_ctrl = 0x04;
    bmi088_acc_write_reg(dev, BMI088_ACC_PWR_CTRL_REG, &pwr_ctrl, 1);
    BMI088_DELAY_MS(50); // 【关键】等待50ms让模拟电路稳定

    // 【新增】在配置前读取并清除错误寄存器
    uint8_t err_reg = 0;
    bmi088_acc_read_reg(dev, 0x02, &err_reg, 1); // ACC_ERR_REG
    if (err_reg != 0)
    {
        ESP_LOGW(TAG, "ACC_ERR_REG before config: 0x%02X, clearing...", err_reg);
    }

    // 配置ACC_CONF（ODR + 带宽）
    // 使用OSR4带宽(0x80) + ODR
    uint8_t conf_val = 0x80 | ((uint8_t)bwp << 4) | (uint8_t)odr;
    if (!bmi088_acc_write_reg(dev, BMI088_ACC_CONF_REG, &conf_val, 1))
    {
        return false;
    }
    BMI088_DELAY_MS(10);

    // 配置ACC_RANGE（量程）
    uint8_t range_val = (uint8_t)range;
    if (!bmi088_acc_write_reg(dev, BMI088_ACC_RANGE_REG, &range_val, 1))
    {
        return false;
    }
    BMI088_DELAY_MS(10);

    // 检查配置后是否有错误
    bmi088_acc_read_reg(dev, 0x02, &err_reg, 1);
    if (err_reg != 0)
    {
        ESP_LOGE(TAG, "ACC_ERR_REG after config: 0x%02X (error_code=%d)", err_reg, (err_reg >> 2) & 0x07);
    }

    dev->config.acc_range = range;
    dev->config.acc_odr = odr;
    dev->config.acc_bwp = bwp;

    return true;
}

bool bmi088_spi_set_gyro_config(bmi088_dev_t *dev, bmi088_gyro_range_t range, bmi088_gyro_bw_t bw)
{
    if (!dev)
    {
        return false;
    }

    // 设置量程
    uint8_t range_val = (uint8_t)range;
    if (!bmi088_gyro_write_reg(dev, BMI088_GYRO_RANGE_REG, &range_val, 1))
    {
        return false;
    }
    BMI088_DELAY_MS(2); // 确保写入间隔

    // 设置带宽
    uint8_t bw_val = (uint8_t)bw;
    if (!bmi088_gyro_write_reg(dev, BMI088_GYRO_BANDWIDTH_REG, &bw_val, 1))
    {
        return false;
    }
    BMI088_DELAY_MS(2); // 确保写入间隔

    dev->config.gyro_range = range;
    dev->config.gyro_bw = bw;

    return true;
}

bool bmi088_spi_set_power_mode(bmi088_dev_t *dev, bmi088_acc_power_t acc_power, bmi088_gyro_power_t gyro_power)
{
    if (!dev)
    {
        return false;
    }

    // 注意：软复位后已经执行过PWR_CONF和PWR_CTRL的写入
    // 这里只需确认或调整电源模式（如果需要不同于Active的模式）

    // 如果需要的模式不是Active(0x00)，则重新配置
    if (acc_power != BMI088_ACC_PWR_ACTIVE)
    {
        uint8_t acc_pwr_conf = (uint8_t)acc_power;
        if (!bmi088_acc_write_reg(dev, BMI088_ACC_PWR_CONF_REG, &acc_pwr_conf, 1))
        {
            return false;
        }
        BMI088_DELAY_MS(5);
    }

    // 配置陀螺仪power模式
    uint8_t gyro_pwr = (uint8_t)gyro_power;
    if (!bmi088_gyro_write_reg(dev, BMI088_GYRO_LPM1_REG, &gyro_pwr, 1))
    {
        return false;
    }
    BMI088_DELAY_MS(30); // 等待陀螺仪电源模式切换

    dev->config.acc_power = acc_power;
    dev->config.gyro_power = gyro_power;

    return true;
}

bool bmi088_spi_read_acc_data(bmi088_dev_t *dev, bmi088_sensor_data_t *acc_data)
{
    if (!dev || !dev->is_initialized || !acc_data)
    {
        return false;
    }

    uint8_t data[6];
    if (!bmi088_acc_read_reg(dev, BMI088_ACC_X_LSB_REG, data, 6))
    {
        return false;
    }

    // 组装16位数据 (LSB + MSB)
    acc_data->x = (int16_t)((data[1] << 8) | data[0]);
    acc_data->y = (int16_t)((data[3] << 8) | data[2]);
    acc_data->z = (int16_t)((data[5] << 8) | data[4]);

    return true;
}

bool bmi088_spi_read_gyro_data(bmi088_dev_t *dev, bmi088_sensor_data_t *gyro_data)
{
    if (!dev || !dev->is_initialized || !gyro_data)
    {
        return false;
    }

    uint8_t data[6];
    if (!bmi088_gyro_read_reg(dev, BMI088_GYRO_X_LSB_REG, data, 6))
    {
        return false;
    }

    // 组装16位数据 (LSB + MSB)
    gyro_data->x = (int16_t)((data[1] << 8) | data[0]);
    gyro_data->y = (int16_t)((data[3] << 8) | data[2]);
    gyro_data->z = (int16_t)((data[5] << 8) | data[4]);

    return true;
}

bool bmi088_spi_read_both_data(bmi088_dev_t *dev, bmi088_sensor_data_t *acc_data, bmi088_sensor_data_t *gyro_data)
{
    if (!bmi088_spi_read_acc_data(dev, acc_data))
    {
        return false;
    }

    return bmi088_spi_read_gyro_data(dev, gyro_data);
}

bool bmi088_spi_check_chip_id(bmi088_dev_t *dev)
{
    if (!dev)
    {
        return false;
    }

    uint8_t acc_chip_id, gyro_chip_id;

    // 检查加速度计芯片ID
    // BMI088加速度计需要虚拟读取：第一次读取任何寄存器会返回无效数据
    uint8_t dummy;
    if (!bmi088_acc_read_reg(dev, BMI088_ACC_CHIP_ID_REG, &dummy, 1))
    {
        DEBUG_PRINT("Failed to read accelerometer chip ID\n");
        return false;
    }

    // 延时后进行真实读取
    BMI088_DELAY_MS(10);

    if (!bmi088_acc_read_reg(dev, BMI088_ACC_CHIP_ID_REG, &acc_chip_id, 1))
    {
        DEBUG_PRINT("Failed to read accelerometer chip ID\n");
        return false;
    }

    if (acc_chip_id != BMI088_ACC_CHIP_ID_VALUE)
    {
        DEBUG_PRINT("Invalid accelerometer chip ID: 0x%02X (expected: 0x%02X)\n",
                    acc_chip_id, BMI088_ACC_CHIP_ID_VALUE);
        return false;
    }

    // 检查陀螺仪芯片ID
    // BMI088陀螺仪也需要虚拟读取（与加速度计相同）
    if (!bmi088_gyro_read_reg(dev, BMI088_GYRO_CHIP_ID_REG, &gyro_chip_id, 1))
    {
        DEBUG_PRINT("Failed to read gyroscope chip ID (dummy read)\n");
        return false;
    }
    BMI088_DELAY_MS(10);

    if (!bmi088_gyro_read_reg(dev, BMI088_GYRO_CHIP_ID_REG, &gyro_chip_id, 1))
    {
        DEBUG_PRINT("Failed to read gyroscope chip ID\n");
        return false;
    }

    if (gyro_chip_id != BMI088_GYRO_CHIP_ID_VALUE)
    {
        DEBUG_PRINT("Invalid gyroscope chip ID: 0x%02X (expected: 0x%02X)\n",
                    gyro_chip_id, BMI088_GYRO_CHIP_ID_VALUE);
        return false;
    }

    DEBUG_PRINT("BMI088 chip ID verification passed (ACC: 0x%02X, GYRO: 0x%02X)\n",
                acc_chip_id, gyro_chip_id);
    return true;
}

bool bmi088_spi_soft_reset(bmi088_dev_t *dev)
{
    if (!dev)
    {
        return false;
    }

    // 软复位加速度计
    uint8_t acc_reset_cmd = 0xB6;
    if (!bmi088_acc_write_reg(dev, BMI088_ACC_SOFTRESET_REG, &acc_reset_cmd, 1))
    {
        return false;
    }
    BMI088_DELAY_MS(1); // 数据手册：软复位后等待1ms

    // 【关键】软复位后必须再次读取寄存器来重新激活SPI模式！
    // 加速度计复位后会回到默认状态（I2C模式），需要通过SPI读取来切换
    // 根据BMI088数据手册，第一次读取会返回无效数据，必须读两次
    uint8_t dummy;

    // 第一次读取：切换到SPI模式（返回无效数据）
    bmi088_acc_read_reg(dev, BMI088_ACC_CHIP_ID_REG, &dummy, 1);
    BMI088_DELAY_MS(2);

    // 第二次读取：获取有效数据
    bmi088_acc_read_reg(dev, BMI088_ACC_CHIP_ID_REG, &dummy, 1);
    BMI088_DELAY_MS(2);

    // 第三次读取：确保SPI模式完全激活
    bmi088_acc_read_reg(dev, BMI088_ACC_CHIP_ID_REG, &dummy, 1);

    if (dummy != 0x1E)
    {
        // 如果读取的芯片ID不正确，说明SPI模式未成功激活
        DEBUG_PRINT("Failed to activate SPI mode after ACC reset, read CHIP_ID: 0x%02X\n", dummy);
        return false;
    }

    // 【关键启动序列】参考数据手册和社区经验：
    // 1. 写PWR_CONF=0x00 (退出suspend模式)
    // 2. 等待450us
    // 3. 写PWR_CTRL=0x04 (使能加速度计)
    // 4. 等待50ms（数据手册要求，让模拟部分完全稳定）
    // 注意：不要使用0x59配置文件加载功能，会导致初始化失败！

    // Step 1: 退出Suspend模式
    uint8_t pwr_conf = 0x00; // Active模式
    for (int retry = 0; retry < 3; retry++)
    {
        bmi088_acc_write_reg(dev, BMI088_ACC_PWR_CONF_REG, &pwr_conf, 1);
        BMI088_DELAY_MS(1); // 等待寄存器更新

        // 读回验证
        uint8_t pwr_conf_read = 0xFF;
        bmi088_acc_read_reg(dev, BMI088_ACC_PWR_CONF_REG, &pwr_conf_read, 1);

        if (pwr_conf_read == 0x00)
        {
            break;
        }

        if (retry == 2)
        {
            return false;
        }
    }

    // Step 2: 使能加速度计
    uint8_t pwr_ctrl = 0x04; // 使能加速度计
    for (int retry = 0; retry < 3; retry++)
    {
        bmi088_acc_write_reg(dev, BMI088_ACC_PWR_CTRL_REG, &pwr_ctrl, 1);
        BMI088_DELAY_MS(1);

        // 读回验证
        uint8_t pwr_ctrl_read = 0xFF;
        bmi088_acc_read_reg(dev, BMI088_ACC_PWR_CTRL_REG, &pwr_ctrl_read, 1);
        if (pwr_ctrl_read == 0x04)
        {
            break;
        }

        if (retry == 2)
        {
            return false;
        }
    }

    // Step 3: 等待加速度计完全启动（关键！）
    // 数据手册要求切换到Normal Mode后等待50ms让模拟部分完全稳定
    BMI088_DELAY_MS(50);

    // 软复位陀螺仪
    uint8_t gyro_reset_cmd = 0xB6;
    if (!bmi088_gyro_write_reg(dev, BMI088_GYRO_SOFTRESET_REG, &gyro_reset_cmd, 1))
    {
        return false;
    }
    BMI088_DELAY_MS(80); // 陀螺仪需要 30ms 启动时间，增加到80ms确保稳定

    DEBUG_PRINT("BMI088 soft reset completed\n");
    return true;
}

bool bmi088_spi_self_test(bmi088_dev_t *dev)
{
    if (!dev || !dev->is_initialized)
    {
        return false;
    }

    bmi088_sensor_data_t acc_data, gyro_data;

    // 读取几次数据验证传感器工作正常
    for (int i = 0; i < 5; i++)
    {
        if (!bmi088_spi_read_both_data(dev, &acc_data, &gyro_data))
        {
            return false;
        }
        BMI088_DELAY_MS(10);
    }

    DEBUG_PRINT("BMI088 self test passed\n");
    return true;
}

bool bmi088_spi_read_temperature(bmi088_dev_t *dev, float *temperature)
{
    if (!dev || !dev->is_initialized || !temperature)
    {
        return false;
    }

    uint8_t temp_data[2];
    if (!bmi088_acc_read_reg(dev, BMI088_ACC_TEMP_LSB_REG, temp_data, 2))
    {
        return false;
    }

    // 温度计算 (根据BMI088数据手册)
    int16_t temp_raw = (int16_t)((temp_data[1] << 8) | temp_data[0]);
    temp_raw = temp_raw >> 5; // 11位有效数据

    if (temp_raw > 1023)
    {
        temp_raw -= 2048; // 补码转换
    }

    *temperature = temp_raw * 0.125f + 23.0f; // 转换为摄氏度

    return true;
}

float bmi088_spi_get_acc_scale(bmi088_acc_range_t range)
{
    if (range >= sizeof(acc_scale_table) / sizeof(acc_scale_table[0]))
    {
        return 0.0f;
    }
    return acc_scale_table[range];
}

float bmi088_spi_get_gyro_scale(bmi088_gyro_range_t range)
{
    if (range >= sizeof(gyro_scale_table) / sizeof(gyro_scale_table[0]))
    {
        return 0.0f;
    }
    return gyro_scale_table[range];
}

/**
 * @brief 配置BMI088加速度计数据就绪中断(INT1)
 * @param dev BMI088设备结构体
 * @return true: 成功, false: 失败
 */
static bool bmi088_acc_configure_data_ready_interrupt(bmi088_dev_t *dev)
{
    if (!dev)
    {
        return false;
    }

    // 检查错误寄存器
    uint8_t acc_err = 0;
    bmi088_acc_read_reg(dev, 0x02, &acc_err, 1);
    if (acc_err != 0x00)
    {
        ESP_LOGE(TAG, "ACC has error! ACC_ERR=0x%02X error_code=%d", acc_err, (acc_err >> 2) & 0x07);
    }

    // 配置INT1引脚为推挽输出,低电平有效
    uint8_t int1_io_conf = BMI088_ACC_INT1_OUTPUT_EN;
    if (!bmi088_acc_write_reg(dev, BMI088_ACC_INT1_IO_CONF_REG, &int1_io_conf, 1))
    {
        return false;
    }
    BMI088_DELAY_MS(1);

    // 将数据就绪中断映射到INT1引脚
    uint8_t int_map = BMI088_ACC_DRDY_INT1;
    if (!bmi088_acc_write_reg(dev, BMI088_ACC_INT1_INT2_MAP_DATA_REG, &int_map, 1))
    {
        return false;
    }
    BMI088_DELAY_MS(1);

    // 读取一次数据激活数据通路
    uint8_t dummy_data[6];
    bmi088_acc_read_reg(dev, BMI088_ACC_X_LSB_REG, dummy_data, 6);
    BMI088_DELAY_MS(2);

    // 清除中断状态
    uint8_t int_status = 0;
    bmi088_acc_read_reg(dev, BMI088_ACC_INT_STAT_1_REG, &int_status, 1);

    return true;
}

/**
 * @brief 配置BMI088陀螺仪数据就绪中断(INT3)
 * @param dev BMI088设备结构体
 * @return true: 成功, false: 失败
 */
static bool bmi088_gyro_configure_data_ready_interrupt(bmi088_dev_t *dev)
{
    if (!dev)
    {
        return false;
    }

    // 配置INT3引脚为推挽输出,高电平有效
    uint8_t int3_io_conf = BMI088_GYRO_INT3_ACTIVE_HIGH | BMI088_GYRO_INT3_PUSH_PULL;
    if (!bmi088_gyro_write_reg(dev, BMI088_GYRO_INT3_INT4_IO_CONF_REG, &int3_io_conf, 1))
    {
        return false;
    }
    BMI088_DELAY_MS(1);

    // 使能数据就绪中断
    uint8_t int_ctrl = 0x80;
    if (!bmi088_gyro_write_reg(dev, BMI088_GYRO_INT_CTRL_REG, &int_ctrl, 1))
    {
        return false;
    }
    BMI088_DELAY_MS(1);

    // 将数据就绪中断映射到INT3引脚
    uint8_t int_map = BMI088_GYRO_DRDY_INT3;
    if (!bmi088_gyro_write_reg(dev, BMI088_GYRO_INT3_INT4_IO_MAP_REG, &int_map, 1))
    {
        return false;
    }
    BMI088_DELAY_MS(1);

    return true;
}
