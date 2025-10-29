#include "jy901_i2c_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "string.h" // 添加 memcpy 函数所需的头文件

static const char *TAG = "JY901_DRIVER";

// 内部函数声明
static int jy901_i2c_read_reg(jy901_handle_t *handle, uint8_t reg, uint8_t *data, uint8_t len);
static int jy901_i2c_write_reg(jy901_handle_t *handle, uint8_t reg, uint16_t data);
static void jy901_delay_ms(uint32_t ms);

void jy901_get_default_config(jy901_config_t *config)
{
    if (config == NULL)
        return;

    config->i2c_port = I2C_NUM_0;
    config->device_addr = JY901_DEFAULT_ADDR;
    config->sda_pin = GPIO_NUM_5;
    config->scl_pin = GPIO_NUM_4;
    config->i2c_freq = JY901_I2C_FREQ; // 400kHz
    config->timeout_ms = JY901_TIMEOUT_MS;
}

int jy901_init(jy901_handle_t *handle, const jy901_config_t *config)
{
    if (handle == NULL || config == NULL)
    {
        ESP_LOGE(TAG, "Invalid parameters");
        return JY901_ERROR;
    }

    // 复制配置
    memcpy(&handle->config, config, sizeof(jy901_config_t));
    handle->initialized = false;
    handle->height_offset = 0;         // 初始化高度偏移量
    handle->height_calibrated = false; // 初始化校准标志

    // 配置I2C
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = config->sda_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = config->scl_pin,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = config->i2c_freq,
    };

    esp_err_t ret = i2c_param_config(config->i2c_port, &i2c_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C param config failed");
        return JY901_ERROR;
    }

    ret = i2c_driver_install(config->i2c_port, i2c_config.mode, 0, 0, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C driver install failed");
        return JY901_ERROR;
    }

    // 检查设备连接
    if (jy901_check_connection(handle) != JY901_OK)
    {
        ESP_LOGE(TAG, "Device not found at address 0x%02X", config->device_addr);
        i2c_driver_delete(config->i2c_port);
        return JY901_NOT_FOUND;
    }

    handle->initialized = true;
    ESP_LOGI(TAG, "JY901 initialized successfully at address 0x%02X", config->device_addr);
    return JY901_OK;
}

int jy901_deinit(jy901_handle_t *handle)
{
    if (handle == NULL || !handle->initialized)
    {
        return JY901_ERROR;
    }

    esp_err_t ret = i2c_driver_delete(handle->config.i2c_port);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C driver delete failed");
        return JY901_ERROR;
    }

    handle->initialized = false;
    ESP_LOGI(TAG, "JY901 deinitialized");
    return JY901_OK;
}

int jy901_auto_scan(jy901_handle_t *handle)
{
    if (handle == NULL)
    {
        return JY901_ERROR;
    }

    ESP_LOGI(TAG, "Scanning for JY901 device...");

    for (uint8_t addr = 0x01; addr < 0x7F; addr++)
    {
        handle->config.device_addr = addr;

        if (jy901_check_connection(handle) == JY901_OK)
        {
            ESP_LOGI(TAG, "Found JY901 device at address 0x%02X", addr);
            return JY901_OK;
        }
        jy901_delay_ms(1);
    }

    ESP_LOGE(TAG, "No JY901 device found");
    return JY901_NOT_FOUND;
}

int jy901_check_connection(jy901_handle_t *handle)
{
    if (handle == NULL)
    {
        return JY901_ERROR;
    }

    uint8_t test_data;
    int ret = jy901_i2c_read_reg(handle, Roll, &test_data, 1);
    return (ret == JY901_OK) ? JY901_OK : JY901_NOT_FOUND;
}

int jy901_read_raw_angle(jy901_handle_t *handle, int16_t raw_data[3])
{
    if (handle == NULL || raw_data == NULL || !handle->initialized)
    {
        return JY901_ERROR;
    }

    uint8_t data[6];
    int ret = jy901_i2c_read_reg(handle, Roll, data, 6);
    if (ret != JY901_OK)
    {
        ESP_LOGE(TAG, "Failed to read angle data");
        return ret;
    }

    // 转换为16位有符号整数 (小端格式)
    raw_data[0] = (int16_t)((data[1] << 8) | data[0]); // Roll
    raw_data[1] = (int16_t)((data[3] << 8) | data[2]); // Pitch
    raw_data[2] = (int16_t)((data[5] << 8) | data[4]); // Yaw

    // 保存到句柄中
    memcpy(handle->raw_angle, raw_data, sizeof(handle->raw_angle));

    return JY901_OK;
}

int jy901_read_angle(jy901_handle_t *handle, jy901_angle_t *angle)
{
    if (handle == NULL || angle == NULL)
    {
        return JY901_ERROR;
    }

    int16_t raw_data[3];
    int ret = jy901_read_raw_angle(handle, raw_data);
    if (ret != JY901_OK)
    {
        return ret;
    }

    // 转换为角度值 (范围: -180° 到 +180°)
    angle->roll = (float)raw_data[0] / 32768.0f * 180.0f;
    angle->pitch = (float)raw_data[1] / 32768.0f * 180.0f;
    angle->yaw = (float)raw_data[2] / 32768.0f * 180.0f;

    return JY901_OK;
}

int jy901_set_output_rate(jy901_handle_t *handle, uint8_t rate)
{
    if (handle == NULL || !handle->initialized)
    {
        return JY901_ERROR;
    }

    // 解锁寄存器
    int ret = jy901_i2c_write_reg(handle, KEY, KEY_UNLOCK);
    if (ret != JY901_OK)
    {
        return ret;
    }
    jy901_delay_ms(10);

    // 设置输出频率
    ret = jy901_i2c_write_reg(handle, RRATE, rate);
    if (ret != JY901_OK)
    {
        return ret;
    }

    ESP_LOGI(TAG, "Output rate set to %d", rate);
    return JY901_OK;
}

int jy901_start_acc_calibration(jy901_handle_t *handle)
{
    if (handle == NULL || !handle->initialized)
    {
        return JY901_ERROR;
    }

    ESP_LOGI(TAG, "Starting accelerometer calibration - keep device horizontal");

    // 解锁寄存器
    int ret = jy901_i2c_write_reg(handle, KEY, KEY_UNLOCK);
    if (ret != JY901_OK)
        return ret;
    jy901_delay_ms(10);

    // 开始加速度计校准
    ret = jy901_i2c_write_reg(handle, CALSW, CALGYROACC);
    if (ret != JY901_OK)
        return ret;
    jy901_delay_ms(10);

    // 停止校准并保存
    ret = jy901_i2c_write_reg(handle, CALSW, NORMAL);
    if (ret != JY901_OK)
        return ret;
    jy901_delay_ms(10);

    ret = jy901_i2c_write_reg(handle, SAVE, SAVE_PARAM);
    if (ret != JY901_OK)
        return ret;

    ESP_LOGI(TAG, "Accelerometer calibration completed");
    return JY901_OK;
}

int jy901_start_mag_calibration(jy901_handle_t *handle)
{
    if (handle == NULL || !handle->initialized)
    {
        return JY901_ERROR;
    }

    ESP_LOGI(TAG, "Starting magnetometer calibration - rotate device in all directions");

    // 解锁寄存器
    int ret = jy901_i2c_write_reg(handle, KEY, KEY_UNLOCK);
    if (ret != JY901_OK)
        return ret;
    jy901_delay_ms(10);

    // 开始磁力计校准
    ret = jy901_i2c_write_reg(handle, CALSW, CALMAGMM);
    return ret;
}

int jy901_stop_mag_calibration(jy901_handle_t *handle)
{
    if (handle == NULL || !handle->initialized)
    {
        return JY901_ERROR;
    }

    // 解锁寄存器
    int ret = jy901_i2c_write_reg(handle, KEY, KEY_UNLOCK);
    if (ret != JY901_OK)
        return ret;
    jy901_delay_ms(10);

    // 停止校准
    ret = jy901_i2c_write_reg(handle, CALSW, NORMAL);
    if (ret != JY901_OK)
        return ret;
    jy901_delay_ms(10);

    // 保存参数
    ret = jy901_i2c_write_reg(handle, SAVE, SAVE_PARAM);
    if (ret != JY901_OK)
        return ret;

    ESP_LOGI(TAG, "Magnetometer calibration completed and saved");
    return JY901_OK;
}

int jy901_read_raw_acc(jy901_handle_t *handle, int16_t raw_data[3])
{
    if (handle == NULL || raw_data == NULL || !handle->initialized)
    {
        return JY901_ERROR;
    }

    uint8_t data[6];
    int ret = jy901_i2c_read_reg(handle, AX, data, 6);
    if (ret != JY901_OK)
    {
        ESP_LOGE(TAG, "Failed to read accelerometer data");
        return ret;
    }

    // 转换为16位有符号整数 (小端格式)
    raw_data[0] = (int16_t)((data[1] << 8) | data[0]); // AX
    raw_data[1] = (int16_t)((data[3] << 8) | data[2]); // AY
    raw_data[2] = (int16_t)((data[5] << 8) | data[4]); // AZ

    // 保存到句柄中
    memcpy(handle->raw_acc, raw_data, sizeof(handle->raw_acc));

    return JY901_OK;
}

int jy901_read_acc(jy901_handle_t *handle, jy901_acc_t *acc)
{
    if (handle == NULL || acc == NULL)
    {
        return JY901_ERROR;
    }

    int16_t raw_data[3];
    int ret = jy901_read_raw_acc(handle, raw_data);
    if (ret != JY901_OK)
    {
        return ret;
    }

    // 转换为加速度值 (范围: -16g 到 +16g)
    acc->ax = (float)raw_data[0] / 32768.0f * 16.0f;
    acc->ay = (float)raw_data[1] / 32768.0f * 16.0f;
    acc->az = (float)raw_data[2] / 32768.0f * 16.0f;

    return JY901_OK;
}

int jy901_read_raw_gyro(jy901_handle_t *handle, int16_t raw_data[3])
{
    if (handle == NULL || raw_data == NULL || !handle->initialized)
    {
        return JY901_ERROR;
    }

    uint8_t data[6];
    int ret = jy901_i2c_read_reg(handle, GX, data, 6);
    if (ret != JY901_OK)
    {
        ESP_LOGE(TAG, "Failed to read gyroscope data");
        return ret;
    }

    // 转换为16位有符号整数 (小端格式)
    raw_data[0] = (int16_t)((data[1] << 8) | data[0]); // GX
    raw_data[1] = (int16_t)((data[3] << 8) | data[2]); // GY
    raw_data[2] = (int16_t)((data[5] << 8) | data[4]); // GZ

    // 保存到句柄中
    memcpy(handle->raw_gyro, raw_data, sizeof(handle->raw_gyro));

    return JY901_OK;
}

int jy901_read_gyro(jy901_handle_t *handle, jy901_gyro_t *gyro)
{
    if (handle == NULL || gyro == NULL)
    {
        return JY901_ERROR;
    }

    int16_t raw_data[3];
    int ret = jy901_read_raw_gyro(handle, raw_data);
    if (ret != JY901_OK)
    {
        return ret;
    }

    // 转换为角速度值 (范围: -2000°/s 到 +2000°/s)
    gyro->gx = (float)raw_data[0] / 32768.0f * 2000.0f;
    gyro->gy = (float)raw_data[1] / 32768.0f * 2000.0f;
    gyro->gz = (float)raw_data[2] / 32768.0f * 2000.0f;

    return JY901_OK;
}

int jy901_read_raw_mag(jy901_handle_t *handle, int16_t raw_data[3])
{
    if (handle == NULL || raw_data == NULL || !handle->initialized)
    {
        return JY901_ERROR;
    }

    uint8_t data[6];
    int ret = jy901_i2c_read_reg(handle, HX, data, 6);
    if (ret != JY901_OK)
    {
        ESP_LOGE(TAG, "Failed to read magnetometer data");
        return ret;
    }

    // 转换为16位有符号整数 (小端格式)
    raw_data[0] = (int16_t)((data[1] << 8) | data[0]); // HX
    raw_data[1] = (int16_t)((data[3] << 8) | data[2]); // HY
    raw_data[2] = (int16_t)((data[5] << 8) | data[4]); // HZ

    // 保存到句柄中
    memcpy(handle->raw_mag, raw_data, sizeof(handle->raw_mag));

    return JY901_OK;
}

int jy901_read_mag(jy901_handle_t *handle, jy901_mag_t *mag)
{
    if (handle == NULL || mag == NULL)
    {
        return JY901_ERROR;
    }

    int16_t raw_data[3];
    int ret = jy901_read_raw_mag(handle, raw_data);
    if (ret != JY901_OK)
    {
        return ret;
    }

    // 转换为磁场强度值 (根据JY901规格书，通常无单位或相对值)
    mag->hx = (float)raw_data[0];
    mag->hy = (float)raw_data[1];
    mag->hz = (float)raw_data[2];

    return JY901_OK;
}

int jy901_read_raw_temp(jy901_handle_t *handle, int16_t *raw_temp)
{
    if (handle == NULL || raw_temp == NULL || !handle->initialized)
    {
        return JY901_ERROR;
    }

    uint8_t data[2];
    int ret = jy901_i2c_read_reg(handle, TEMP, data, 2);
    if (ret != JY901_OK)
    {
        ESP_LOGE(TAG, "Failed to read temperature data");
        return ret;
    }

    // 转换为16位有符号整数 (小端格式)
    *raw_temp = (int16_t)((data[1] << 8) | data[0]);

    // 保存到句柄中
    handle->raw_temp = *raw_temp;

    return JY901_OK;
}

int jy901_read_temp(jy901_handle_t *handle, jy901_temp_t *temp)
{
    if (handle == NULL || temp == NULL)
    {
        return JY901_ERROR;
    }

    int16_t raw_temp;
    int ret = jy901_read_raw_temp(handle, &raw_temp);
    if (ret != JY901_OK)
    {
        return ret;
    }

    // 转换为温度值 (根据JY901规格书: 温度 = 原始值/340.0 + 36.25)
    temp->temperature = (float)raw_temp / 340.0f + 36.25f;

    return JY901_OK;
}

int jy901_read_raw_pressure_height(jy901_handle_t *handle, int32_t *raw_pressure, int32_t *raw_height)
{
    if (handle == NULL || raw_pressure == NULL || raw_height == NULL || !handle->initialized)
    {
        return JY901_ERROR;
    }

    uint8_t data[8];
    int ret = jy901_i2c_read_reg(handle, PressureL, data, 8);
    if (ret != JY901_OK)
    {
        ESP_LOGE(TAG, "Failed to read pressure and height data");
        return ret;
    }

    // 按照官方文档组装气压数据
    uint16_t pressure_l = (uint16_t)((data[1] << 8) | data[0]); // PressureL
    uint16_t pressure_h = (uint16_t)((data[3] << 8) | data[2]); // PressureH
    *raw_pressure = ((int32_t)pressure_h << 16) | pressure_l;

    // 按照官方文档组装高度数据
    uint16_t height_l = (uint16_t)((data[5] << 8) | data[4]); // HeightL
    uint16_t height_h = (uint16_t)((data[7] << 8) | data[6]); // HeightH
    *raw_height = ((int32_t)height_h << 16) | height_l;

    // 保存到句柄中
    handle->raw_pressure = *raw_pressure;
    handle->raw_height = *raw_height;

    return JY901_OK;
}

int jy901_read_pressure_height_calibrated(jy901_handle_t *handle, jy901_press_t *press)
{
    if (handle == NULL || press == NULL)
    {
        return JY901_ERROR;
    }

    int32_t raw_pressure, raw_height;
    int ret = jy901_read_raw_pressure_height(handle, &raw_pressure, &raw_height);
    if (ret != JY901_OK)
    {
        return ret;
    }

    // 气压直接转换
    press->pressure = (float)raw_pressure;

    // 高度根据校准状态处理
    if (handle->height_calibrated)
    {
        // 计算相对高度
        int32_t calibrated_height_cm = raw_height - handle->height_offset;
        press->height = (float)calibrated_height_cm / 100.0f;
    }
    else
    {
        // 未校准，返回原始高度
        press->height = (float)raw_height / 100.0f;
    }

    return JY901_OK;
}

int jy901_read_all_raw_data(jy901_handle_t *handle)
{
    if (handle == NULL || !handle->initialized)
    {
        return JY901_ERROR;
    }

    ESP_LOGD(TAG, "Reading all sensor raw data...");

    // 读取所有传感器数据 (AX到HeightH，连续读取17个寄存器)
    uint8_t data[34]; // 17个16位寄存器 = 34字节
    int ret = jy901_i2c_read_reg(handle, AX, data, 34);
    if (ret != JY901_OK)
    {
        ESP_LOGE(TAG, "Failed to read all sensor data");
        return ret;
    }

    // 解析加速度计数据 (AX, AY, AZ)
    handle->raw_acc[0] = (int16_t)((data[1] << 8) | data[0]); // AX
    handle->raw_acc[1] = (int16_t)((data[3] << 8) | data[2]); // AY
    handle->raw_acc[2] = (int16_t)((data[5] << 8) | data[4]); // AZ

    // 解析陀螺仪数据 (GX, GY, GZ)
    handle->raw_gyro[0] = (int16_t)((data[7] << 8) | data[6]);   // GX
    handle->raw_gyro[1] = (int16_t)((data[9] << 8) | data[8]);   // GY
    handle->raw_gyro[2] = (int16_t)((data[11] << 8) | data[10]); // GZ

    // 解析磁力计数据 (HX, HY, HZ)
    handle->raw_mag[0] = (int16_t)((data[13] << 8) | data[12]); // HX
    handle->raw_mag[1] = (int16_t)((data[15] << 8) | data[14]); // HY
    handle->raw_mag[2] = (int16_t)((data[17] << 8) | data[16]); // HZ

    // 解析角度数据 (Roll, Pitch, Yaw)
    handle->raw_angle[0] = (int16_t)((data[19] << 8) | data[18]); // Roll
    handle->raw_angle[1] = (int16_t)((data[21] << 8) | data[20]); // Pitch
    handle->raw_angle[2] = (int16_t)((data[23] << 8) | data[22]); // Yaw

    // 解析温度数据 (TEMP)
    handle->raw_temp = (int16_t)((data[25] << 8) | data[24]); // TEMP

    // 按照官方文档组装气压数据 (PressureL, PressureH)
    uint16_t pressure_l = (uint16_t)((data[27] << 8) | data[26]); // PressureL
    uint16_t pressure_h = (uint16_t)((data[29] << 8) | data[28]); // PressureH
    handle->raw_pressure = ((int32_t)pressure_h << 16) | pressure_l;

    // 按照官方文档组装高度数据 (HeightL, HeightH)
    uint16_t height_l = (uint16_t)((data[31] << 8) | data[30]); // HeightL
    uint16_t height_h = (uint16_t)((data[33] << 8) | data[32]); // HeightH
    handle->raw_height = ((int32_t)height_h << 16) | height_l;

    ESP_LOGD(TAG, "All sensor raw data read successfully");
    return JY901_OK;
}

// 内部函数实现
static int jy901_i2c_read_reg(jy901_handle_t *handle, uint8_t reg, uint8_t *data, uint8_t len)
{
    if (handle == NULL || data == NULL || len == 0)
    {
        return JY901_ERROR;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // 写寄存器地址
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->config.device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    // 读取数据
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->config.device_addr << 1) | I2C_MASTER_READ, true);

    for (int i = 0; i < len; i++)
    {
        if (i == len - 1)
        {
            i2c_master_read_byte(cmd, &data[i], I2C_MASTER_NACK);
        }
        else
        {
            i2c_master_read_byte(cmd, &data[i], I2C_MASTER_ACK);
        }
    }

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(handle->config.i2c_port, cmd,
                                         handle->config.timeout_ms / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK) ? JY901_OK : JY901_ERROR;
}

static int jy901_i2c_write_reg(jy901_handle_t *handle, uint8_t reg, uint16_t data)
{
    if (handle == NULL)
    {
        return JY901_ERROR;
    }

    uint8_t write_data[2] = {data & 0xFF, (data >> 8) & 0xFF};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->config.device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, write_data[0], true);
    i2c_master_write_byte(cmd, write_data[1], true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(handle->config.i2c_port, cmd,
                                         handle->config.timeout_ms / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK) ? JY901_OK : JY901_ERROR;
}

static void jy901_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

int jy901_reset_height_zero(jy901_handle_t *handle)
{
    if (handle == NULL || !handle->initialized)
    {
        return JY901_ERROR;
    }

    int32_t current_pressure, current_height;
    int ret = jy901_read_raw_pressure_height(handle, &current_pressure, &current_height);
    if (ret != JY901_OK)
    {
        ESP_LOGE(TAG, "Failed to read current height for calibration");
        return ret;
    }

    // 将当前高度作为偏移量，实现高度置零
    handle->height_offset = current_height;
    handle->height_calibrated = true;

    ESP_LOGI(TAG, "Height zero reset completed, offset: %ld cm", handle->height_offset);
    return JY901_OK;
}

int jy901_clear_height_calibration(jy901_handle_t *handle)
{
    if (handle == NULL)
    {
        return JY901_ERROR;
    }

    handle->height_offset = 0;
    handle->height_calibrated = false;

    ESP_LOGI(TAG, "Height calibration cleared");
    return JY901_OK;
}

bool jy901_is_height_calibrated(jy901_handle_t *handle)
{
    if (handle == NULL)
    {
        return false;
    }

    return handle->height_calibrated;
}

int jy901_get_angle_from_cache(jy901_handle_t *handle, jy901_angle_t *angle)
{
    if (handle == NULL || angle == NULL || !handle->initialized)
    {
        return JY901_ERROR;
    }

    // 从缓存中的原始数据转换为角度值
    angle->roll = (float)handle->raw_angle[0] / 32768.0f * 180.0f;
    angle->pitch = (float)handle->raw_angle[1] / 32768.0f * 180.0f;
    angle->yaw = (float)handle->raw_angle[2] / 32768.0f * 180.0f;

    return JY901_OK;
}

int jy901_get_pressure_height_from_cache(jy901_handle_t *handle, jy901_press_t *press)
{
    if (handle == NULL || press == NULL || !handle->initialized)
    {
        return JY901_ERROR;
    }

    // 气压直接转换
    press->pressure = (float)handle->raw_pressure;

    // 高度根据校准状态处理
    if (handle->height_calibrated)
    {
        // 计算相对高度
        int32_t calibrated_height_cm = handle->raw_height - handle->height_offset;
        press->height = (float)calibrated_height_cm / 100.0f;
    }
    else
    {
        // 未校准，返回原始高度
        press->height = (float)handle->raw_height / 100.0f;
    }

    return JY901_OK;
}

int jy901_get_acc_from_cache(jy901_handle_t *handle, jy901_acc_t *acc)
{
    if (handle == NULL || acc == NULL || !handle->initialized)
    {
        return JY901_ERROR;
    }

    // 从缓存中的原始数据转换为加速度值
    acc->ax = (float)handle->raw_acc[0] / 32768.0f * 16.0f;
    acc->ay = (float)handle->raw_acc[1] / 32768.0f * 16.0f;
    acc->az = (float)handle->raw_acc[2] / 32768.0f * 16.0f;

    return JY901_OK;
}

int jy901_get_gyro_from_cache(jy901_handle_t *handle, jy901_gyro_t *gyro)
{
    if (handle == NULL || gyro == NULL || !handle->initialized)
    {
        return JY901_ERROR;
    }

    // 从缓存中的原始数据转换为角速度值
    gyro->gx = (float)handle->raw_gyro[0] / 32768.0f * 2000.0f;
    gyro->gy = (float)handle->raw_gyro[1] / 32768.0f * 2000.0f;
    gyro->gz = (float)handle->raw_gyro[2] / 32768.0f * 2000.0f;

    return JY901_OK;
}

int jy901_get_temp_from_cache(jy901_handle_t *handle, jy901_temp_t *temp)
{
    if (handle == NULL || temp == NULL || !handle->initialized)
    {
        return JY901_ERROR;
    }

    // 从缓存中的原始数据转换为温度值
    temp->temperature = (float)handle->raw_temp / 340.0f + 36.25f;

    return JY901_OK;
}