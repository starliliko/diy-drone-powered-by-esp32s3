#include "spi_drv.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"

// 兼容不同的调试输出方式
#ifdef DEBUG_MODULE
#include "debug_cf.h"
#else
static const char *TAG = "SPI_DRV";
#define DEBUG_PRINT(fmt, ...) ESP_LOGD(TAG, fmt, ##__VA_ARGS__)
#endif

// 全局总线互斥锁
SemaphoreHandle_t g_spi_bus_mutex = NULL;

// 内部函数声明
static esp_err_t spi_drv_bus_initialize(spi_drv_bus_config_t *bus_config);
static esp_err_t spi_drv_device_initialize(spi_drv_t *spi);
static bool spi_drv_validate_pins(spi_drv_bus_config_t *bus_config);

bool spiDrvBusInit(spi_drv_bus_config_t *bus_config)
{
    if (!bus_config)
    {
        DEBUG_PRINT("Invalid bus config\n");
        return false;
    }

    if (bus_config->is_bus_initialized)
    {
        DEBUG_PRINT("SPI bus already initialized\n");
        return true;
    }

    // ESP32S3引脚验证
    if (!spi_drv_validate_pins(bus_config))
    {
        DEBUG_PRINT("Invalid pin configuration for ESP32S3\n");
        return false;
    }

    // 创建全局总线互斥锁（如果还没创建）
    if (g_spi_bus_mutex == NULL)
    {
        g_spi_bus_mutex = xSemaphoreCreateMutex();
        if (!g_spi_bus_mutex)
        {
            DEBUG_PRINT("Failed to create bus mutex\n");
            return false;
        }
    }

    // 初始化SPI总线
    esp_err_t ret = spi_drv_bus_initialize(bus_config);
    if (ret != ESP_OK)
    {
        DEBUG_PRINT("Failed to initialize SPI bus: %s\n", esp_err_to_name(ret));
        return false;
    }

    bus_config->is_bus_initialized = true;
    DEBUG_PRINT("SPI bus initialized successfully on ESP32S3\n");
    return true;
}

bool spiDrvDeviceInit(spi_drv_t *spi, spi_drv_bus_config_t *bus_config, const spi_drv_device_config_t *device_config)
{
    if (!spi || !bus_config || !device_config)
    {
        DEBUG_PRINT("Invalid parameters\n");
        return false;
    }

    if (spi->is_initialized)
    {
        DEBUG_PRINT("SPI device already initialized\n");
        return true;
    }

    // 确保总线已初始化
    if (!spiDrvBusInit(bus_config))
    {
        return false;
    }

    // 复制配置
    spi->bus_config = bus_config;
    memcpy(&spi->device_config, device_config, sizeof(spi_drv_device_config_t));

    // 创建设备级互斥锁（可选）
    spi->mutex = xSemaphoreCreateMutex();
    if (!spi->mutex)
    {
        DEBUG_PRINT("Failed to create device mutex\n");
        return false;
    }

    // 初始化SPI设备
    esp_err_t ret = spi_drv_device_initialize(spi);
    if (ret != ESP_OK)
    {
        DEBUG_PRINT("Failed to initialize SPI device: %s\n", esp_err_to_name(ret));
        vSemaphoreDelete(spi->mutex);
        return false;
    }

    // 配置CS引脚
    if (device_config->cs_pin >= 0)
    {
        gpio_config_t cs_conf = {
            .pin_bit_mask = (1ULL << device_config->cs_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };

        ret = gpio_config(&cs_conf);
        if (ret != ESP_OK)
        {
            DEBUG_PRINT("Failed to configure CS pin: %s\n", esp_err_to_name(ret));
            spi_bus_remove_device(spi->device);
            vSemaphoreDelete(spi->mutex);
            return false;
        }
        gpio_set_level(device_config->cs_pin, 1); // CS高电平（未选中）
    }

    spi->is_initialized = true;
    DEBUG_PRINT("SPI device initialized successfully on ESP32S3\n");
    return true;
}

bool spiDrvDeinit(spi_drv_t *spi)
{
    if (!spi || !spi->is_initialized)
    {
        return false;
    }

    spi_bus_remove_device(spi->device);

    if (spi->mutex)
    {
        vSemaphoreDelete(spi->mutex);
        spi->mutex = NULL;
    }

    spi->is_initialized = false;
    return true;
}

bool spiDrvTransfer(spi_drv_t *spi, spi_drv_transfer_t *transfer)
{
    if (!spi || !spi->is_initialized || !transfer)
    {
        return false;
    }

    if (transfer->length == 0)
    {
        return true;
    }

    // ESP32S3传输大小限制检查
    if (transfer->length > SPI_DRV_MAX_TRANSFER_SIZE)
    {
        DEBUG_PRINT("Transfer size too large for ESP32S3: %d\n", transfer->length);
        return false;
    }

    // 获取总线级互斥锁
    if (xSemaphoreTake(g_spi_bus_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        DEBUG_PRINT("Failed to take bus mutex\n");
        return false;
    }

    esp_err_t ret = ESP_OK;
    spi_transaction_t trans = {0};

    trans.length = transfer->length * 8; // 以位为单位
    trans.tx_buffer = transfer->tx_buffer;
    trans.rx_buffer = transfer->rx_buffer;
    trans.flags = transfer->flags;

    // ESP32S3优化：对于小数据使用内置缓冲区
    if (transfer->length <= 4)
    {
        trans.flags |= SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        if (transfer->tx_buffer)
        {
            memcpy(trans.tx_data, transfer->tx_buffer, transfer->length);
        }
    }

    // CS控制
    if (spi->device_config.cs_pin >= 0)
    {
        gpio_set_level(spi->device_config.cs_pin, 0); // CS低电平（选中）
        esp_rom_delay_us(1);                          // ESP32S3更精确的延时控制
    }

    // 执行传输 - ESP32S3优化
    if (spi->bus_config->use_dma && transfer->length > 64)
    {
        ret = spi_device_transmit(spi->device, &trans);
    }
    else
    {
        ret = spi_device_polling_transmit(spi->device, &trans);
    }

    // 如果使用了内置缓冲区并且是读取操作，复制数据
    if ((trans.flags & SPI_TRANS_USE_RXDATA) && transfer->rx_buffer)
    {
        memcpy(transfer->rx_buffer, trans.rx_data, transfer->length);
    }

    // CS控制
    if (spi->device_config.cs_pin >= 0)
    {
        esp_rom_delay_us(1);
        gpio_set_level(spi->device_config.cs_pin, 1); // CS高电平（未选中）
    }

    xSemaphoreGive(g_spi_bus_mutex);

    if (ret != ESP_OK)
    {
        DEBUG_PRINT("SPI transfer failed: %s\n", esp_err_to_name(ret));
        return false;
    }

    return true;
}

bool spiDrvWriteReg(spi_drv_t *spi, uint8_t reg_addr, const uint8_t *data, size_t length)
{
    uint8_t tx_buf[length + 1];
    tx_buf[0] = reg_addr; // 寄存器地址
    memcpy(&tx_buf[1], data, length);

    spi_drv_transfer_t transfer = {
        .tx_buffer = tx_buf,
        .rx_buffer = NULL,
        .length = length + 1,
        .is_write = true,
        .flags = 0};

    return spiDrvTransfer(spi, &transfer);
}

bool spiDrvReadReg(spi_drv_t *spi, uint8_t reg_addr, uint8_t *data, size_t length)
{
    uint8_t tx_buf[length + 1];
    uint8_t rx_buf[length + 1];

    tx_buf[0] = reg_addr;             // 寄存器地址
    memset(&tx_buf[1], 0x00, length); // 发送填充字节

    spi_drv_transfer_t transfer = {
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
        .length = length + 1,
        .is_write = false,
        .flags = 0};

    if (spiDrvTransfer(spi, &transfer))
    {
        memcpy(data, &rx_buf[1], length); // 跳过第一个字节
        return true;
    }

    return false;
}

bool spiDrvWriteByte(spi_drv_t *spi, uint8_t reg_addr, uint8_t data)
{
    return spiDrvWriteReg(spi, reg_addr, &data, 1);
}

bool spiDrvReadByte(spi_drv_t *spi, uint8_t reg_addr, uint8_t *data)
{
    return spiDrvReadReg(spi, reg_addr, data, 1);
}

void spiDrvSetCS(spi_drv_t *spi, bool level)
{
    if (spi && spi->device_config.cs_pin >= 0)
    {
        gpio_set_level(spi->device_config.cs_pin, level ? 1 : 0);
    }
}

// ESP32S3特定函数
bool spiDrvGetMaxFreq(spi_host_device_t host_id, uint32_t *max_freq)
{
    if (!max_freq)
    {
        return false;
    }

    // ESP32S3的SPI最大频率
    switch (host_id)
    {
    case SPI2_HOST:
        *max_freq = 80000000; // 80MHz - 独立DMA，性能最优
        break;
    case SPI3_HOST:
        *max_freq = 60000000; // 60MHz - 共享DMA，适当降低
        break;
    default:
        *max_freq = 0;
        return false;
    }

    return true;
}

// 验证引脚是否适合SPI使用
bool spiDrvValidatePin(int pin, const char *pin_name)
{
    // 检查是否为有效GPIO
    if (!GPIO_IS_VALID_GPIO(pin))
    {
        DEBUG_PRINT("Invalid %s pin %d\n", pin_name, pin);
        return false;
    }

    // ESP32S3严格限制：GPIO26-37专用于SPI0/SPI1(Flash/PSRAM)
    if (pin >= 26 && pin <= 37)
    {
        DEBUG_PRINT("Error: %s pin %d is reserved for SPI0/SPI1 (Flash/PSRAM)\n", pin_name, pin);
        return false;
    }

    // 其他特殊引脚检查
    const struct
    {
        int pin;
        const char *warning;
    } special_pins[] = {
        {0, "Boot mode control"},
        {3, "JTAG functionality"},
        {19, "USB D- (conflicts with USB)"},
        {20, "USB D+ (conflicts with USB)"},
        {43, "UART download mode"},
        {44, "UART download mode"},
        {45, "Crystal oscillator"},
        {46, "Crystal oscillator"}};

    for (int i = 0; i < sizeof(special_pins) / sizeof(special_pins[0]); i++)
    {
        if (pin == special_pins[i].pin)
        {
            DEBUG_PRINT("Warning: %s pin %d - %s\n", pin_name, pin, special_pins[i].warning);
            break;
        }
    }

    return true;
}

// 内部函数实现
static esp_err_t spi_drv_bus_initialize(spi_drv_bus_config_t *bus_config)
{
    spi_bus_config_t esp_bus_config = {
        .miso_io_num = bus_config->miso_pin,
        .mosi_io_num = bus_config->mosi_pin,
        .sclk_io_num = bus_config->sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = bus_config->max_transfer_sz ? bus_config->max_transfer_sz : SPI_DRV_MAX_TRANSFER_SIZE,
        .flags = SPICOMMON_BUSFLAG_MASTER,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
    };

    // ESP32S3始终使用自动DMA通道选择
    return spi_bus_initialize(bus_config->host_id, &esp_bus_config,
                              bus_config->use_dma ? SPI_DMA_CH_AUTO : SPI_DMA_DISABLED);
}

static esp_err_t spi_drv_device_initialize(spi_drv_t *spi)
{
    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = spi->device_config.clock_speed_hz,
        .mode = spi->device_config.mode,
        .spics_io_num = -1, // 手动控制CS
        .queue_size = spi->device_config.queue_size,
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .duty_cycle_pos = 128, // ESP32S3默认占空比
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .input_delay_ns = 0,
        .flags = SPI_DEVICE_HALFDUPLEX, // ESP32S3支持半双工优化
    };

    return spi_bus_add_device(spi->bus_config->host_id, &dev_config, &spi->device);
}

static bool spi_drv_validate_pins(spi_drv_bus_config_t *bus_config)
{
    bool valid = true;

    // 验证所有SPI总线引脚
    if (bus_config->miso_pin >= 0)
    {
        valid &= spiDrvValidatePin(bus_config->miso_pin, "MISO");
    }

    if (bus_config->mosi_pin >= 0)
    {
        valid &= spiDrvValidatePin(bus_config->mosi_pin, "MOSI");
    }

    if (bus_config->sclk_pin >= 0)
    {
        valid &= spiDrvValidatePin(bus_config->sclk_pin, "SCLK");
    }

    // 检查引脚冲突
    int pins[] = {bus_config->miso_pin, bus_config->mosi_pin, bus_config->sclk_pin};
    for (int i = 0; i < 3; i++)
    {
        for (int j = i + 1; j < 3; j++)
        {
            if (pins[i] >= 0 && pins[i] == pins[j])
            {
                DEBUG_PRINT("Pin conflict: GPIO%d used multiple times\n", pins[i]);
                valid = false;
            }
        }
    }

    // 强制检查是否使用了SPI0/SPI1专用引脚范围
    for (int i = 0; i < 3; i++)
    {
        if (pins[i] >= 26 && pins[i] <= 37)
        {
            DEBUG_PRINT("Error: GPIO%d is reserved for Flash/PSRAM, cannot be used for external SPI\n", pins[i]);
            valid = false;
        }
    }

    // 打印引脚映射
    if (valid)
    {
        spiDrvPrintPinMapping(bus_config);
    }

    return valid;
}
