#include "bmi088_spi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "BMI088_SPI";

// SPI读写标志位
#define BMI088_SPI_READ_FLAG 0x80
#define BMI088_SPI_WRITE_FLAG 0x00

// 延时宏定义
#define BMI088_DELAY_MS(ms) vTaskDelay(pdMS_TO_TICKS(ms))

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
    if (!dev || !bus_config || !acc_device_config || !gyro_device_config)
    {
        ESP_LOGE(TAG, "Invalid parameters");
        return false;
    }

    // 初始化加速度计SPI设备
    if (!spiDrvDeviceInit(&dev->acc_spi, bus_config, acc_device_config))
    {
        ESP_LOGE(TAG, "Failed to initialize accelerometer SPI");
        return false;
    }

    // 初始化陀螺仪SPI设备
    if (!spiDrvDeviceInit(&dev->gyro_spi, bus_config, gyro_device_config))
    {
        ESP_LOGE(TAG, "Failed to initialize gyroscope SPI");
        spiDrvDeinit(&dev->acc_spi);
        return false;
    }

    // 软复位
    if (!bmi088_spi_soft_reset(dev))
    {
        ESP_LOGE(TAG, "Soft reset failed");
        bmi088_spi_deinit(dev);
        return false;
    }

    // 检查芯片ID
    if (!bmi088_spi_check_chip_id(dev))
    {
        ESP_LOGE(TAG, "Chip ID verification failed");
        bmi088_spi_deinit(dev);
        return false;
    }

    // 设置默认配置
    bmi088_config_t default_config = {
        .acc_range = BMI088_ACC_RANGE_6G,
        .acc_odr = BMI088_ACC_ODR_800_HZ,
        .acc_bwp = BMI088_ACC_BWP_NORMAL,
        .acc_power = BMI088_ACC_PWR_ACTIVE,
        .gyro_range = BMI088_GYRO_RANGE_1000_DPS,
        .gyro_bw = BMI088_GYRO_BW_116_ODR_1000_HZ,
        .gyro_power = BMI088_GYRO_PWR_NORMAL};

    if (!bmi088_spi_configure(dev, &default_config))
    {
        ESP_LOGE(TAG, "Default configuration failed");
        bmi088_spi_deinit(dev);
        return false;
    }

    dev->is_initialized = true;
    ESP_LOGI(TAG, "BMI088 SPI initialization successful");
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
    if (!dev || !dev->is_initialized)
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

    memcpy(data, &rx_buf[2], length); // 跳过寄存器地址和虚拟字节
    return true;
}

bool bmi088_acc_write_reg(bmi088_dev_t *dev, uint8_t reg_addr, const uint8_t *data, size_t length)
{
    if (!dev || !dev->is_initialized)
    {
        return false;
    }

    return spiDrvWriteReg(&dev->acc_spi, reg_addr | BMI088_SPI_WRITE_FLAG, data, length);
}

bool bmi088_gyro_read_reg(bmi088_dev_t *dev, uint8_t reg_addr, uint8_t *data, size_t length)
{
    if (!dev || !dev->is_initialized)
    {
        return false;
    }

    // 陀螺仪不需要虚拟字节
    return spiDrvReadReg(&dev->gyro_spi, reg_addr | BMI088_SPI_READ_FLAG, data, length);
}

bool bmi088_gyro_write_reg(bmi088_dev_t *dev, uint8_t reg_addr, const uint8_t *data, size_t length)
{
    if (!dev || !dev->is_initialized)
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

    // 设置电源模式
    if (!bmi088_spi_set_power_mode(dev, config->acc_power, config->gyro_power))
    {
        return false;
    }

    BMI088_DELAY_MS(10); // 等待配置生效

    ESP_LOGI(TAG, "BMI088 configuration completed");
    return true;
}

bool bmi088_spi_set_acc_config(bmi088_dev_t *dev, bmi088_acc_range_t range,
                               bmi088_acc_odr_t odr, bmi088_acc_bwp_t bwp)
{
    if (!dev || !dev->is_initialized)
    {
        return false;
    }

    // 设置量程
    uint8_t range_val = (uint8_t)range;
    if (!bmi088_acc_write_reg(dev, BMI088_ACC_RANGE_REG, &range_val, 1))
    {
        return false;
    }

    BMI088_DELAY_MS(1);

    // 设置ODR和带宽
    uint8_t conf_val = ((uint8_t)odr << 0) | ((uint8_t)bwp << 4);
    if (!bmi088_acc_write_reg(dev, BMI088_ACC_CONF_REG, &conf_val, 1))
    {
        return false;
    }

    BMI088_DELAY_MS(1);

    dev->config.acc_range = range;
    dev->config.acc_odr = odr;
    dev->config.acc_bwp = bwp;

    return true;
}

bool bmi088_spi_set_gyro_config(bmi088_dev_t *dev, bmi088_gyro_range_t range, bmi088_gyro_bw_t bw)
{
    if (!dev || !dev->is_initialized)
    {
        return false;
    }

    // 设置量程
    uint8_t range_val = (uint8_t)range;
    if (!bmi088_gyro_write_reg(dev, BMI088_GYRO_RANGE_REG, &range_val, 1))
    {
        return false;
    }

    BMI088_DELAY_MS(1);

    // 设置带宽
    uint8_t bw_val = (uint8_t)bw;
    if (!bmi088_gyro_write_reg(dev, BMI088_GYRO_BANDWIDTH_REG, &bw_val, 1))
    {
        return false;
    }

    BMI088_DELAY_MS(1);

    dev->config.gyro_range = range;
    dev->config.gyro_bw = bw;

    return true;
}

bool bmi088_spi_set_power_mode(bmi088_dev_t *dev, bmi088_acc_power_t acc_power, bmi088_gyro_power_t gyro_power)
{
    if (!dev || !dev->is_initialized)
    {
        return false;
    }

    // 配置加速度计电源
    uint8_t acc_pwr_ctrl = 0x04; // 启用加速度计
    if (!bmi088_acc_write_reg(dev, BMI088_ACC_PWR_CTRL_REG, &acc_pwr_ctrl, 1))
    {
        return false;
    }

    BMI088_DELAY_MS(5);

    uint8_t acc_pwr_conf = (uint8_t)acc_power;
    if (!bmi088_acc_write_reg(dev, BMI088_ACC_PWR_CONF_REG, &acc_pwr_conf, 1))
    {
        return false;
    }

    BMI088_DELAY_MS(5);

    // 配置陀螺仪电源
    uint8_t gyro_pwr = (uint8_t)gyro_power;
    if (!bmi088_gyro_write_reg(dev, BMI088_GYRO_LPM1_REG, &gyro_pwr, 1))
    {
        return false;
    }

    BMI088_DELAY_MS(30);

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
    if (!bmi088_acc_read_reg(dev, BMI088_ACC_CHIP_ID_REG, &acc_chip_id, 1))
    {
        ESP_LOGE(TAG, "Failed to read accelerometer chip ID");
        return false;
    }

    if (acc_chip_id != BMI088_ACC_CHIP_ID_VALUE)
    {
        ESP_LOGE(TAG, "Invalid accelerometer chip ID: 0x%02X (expected: 0x%02X)",
                 acc_chip_id, BMI088_ACC_CHIP_ID_VALUE);
        return false;
    }

    // 检查陀螺仪芯片ID
    if (!bmi088_gyro_read_reg(dev, BMI088_GYRO_CHIP_ID_REG, &gyro_chip_id, 1))
    {
        ESP_LOGE(TAG, "Failed to read gyroscope chip ID");
        return false;
    }

    if (gyro_chip_id != BMI088_GYRO_CHIP_ID_VALUE)
    {
        ESP_LOGE(TAG, "Invalid gyroscope chip ID: 0x%02X (expected: 0x%02X)",
                 gyro_chip_id, BMI088_GYRO_CHIP_ID_VALUE);
        return false;
    }

    ESP_LOGI(TAG, "BMI088 chip ID verification passed (ACC: 0x%02X, GYRO: 0x%02X)",
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

    BMI088_DELAY_MS(50);

    // 软复位陀螺仪
    uint8_t gyro_reset_cmd = 0xB6;
    if (!bmi088_gyro_write_reg(dev, BMI088_GYRO_SOFTRESET_REG, &gyro_reset_cmd, 1))
    {
        return false;
    }

    BMI088_DELAY_MS(50);

    ESP_LOGI(TAG, "BMI088 soft reset completed");
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

    ESP_LOGI(TAG, "BMI088 self test passed");
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
