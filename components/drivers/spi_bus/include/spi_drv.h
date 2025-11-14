#ifndef __SPI_DRV_H__
#define __SPI_DRV_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "soc/soc_caps.h"

#ifdef __cplusplus
extern "C"
{
#endif

// ESP32S3 SPI 波特率定义
#define SPI_BAUDRATE_125KHZ 125000
#define SPI_BAUDRATE_250KHZ 250000
#define SPI_BAUDRATE_500KHZ 500000
#define SPI_BAUDRATE_1MHZ 1000000
#define SPI_BAUDRATE_2MHZ 2000000
#define SPI_BAUDRATE_4MHZ 4000000
#define SPI_BAUDRATE_8MHZ 8000000
#define SPI_BAUDRATE_10MHZ 10000000
#define SPI_BAUDRATE_20MHZ 20000000
#define SPI_BAUDRATE_40MHZ 40000000
#define SPI_BAUDRATE_80MHZ 80000000

// SPI 模式定义
#define SPI_MODE_0 0 // CPOL=0, CPHA=0
#define SPI_MODE_1 1 // CPOL=0, CPHA=1
#define SPI_MODE_2 2 // CPOL=1, CPHA=0
#define SPI_MODE_3 3 // CPOL=1, CPHA=1

// ESP32S3 SPI主机定义
#define SPI_DRV_HOST_PRIMARY SPI2_HOST   // 首选：独立DMA通道
#define SPI_DRV_HOST_SECONDARY SPI3_HOST // 备选：共享DMA通道
#define SPI_DRV_HOST_DEFAULT SPI_DRV_HOST_PRIMARY

// ESP32S3最大传输大小
#define SPI_DRV_MAX_TRANSFER_SIZE (SOC_SPI_MAXIMUM_BUFFER_SIZE)

    // SPI 总线配置（共享的硬件资源）
    typedef struct
    {
        spi_host_device_t host_id; // SPI主机ID (SPI2_HOST 或 SPI3_HOST)
        int miso_pin;              // MISO引脚
        int mosi_pin;              // MOSI引脚
        int sclk_pin;              // 时钟引脚
        bool use_dma;              // 是否使用DMA
        bool is_bus_initialized;   // 总线是否已初始化
        uint32_t max_transfer_sz;  // 最大传输大小
    } spi_drv_bus_config_t;

    // SPI 设备配置（每个设备独有）
    typedef struct
    {
        int cs_pin;              // 片选引脚
        uint32_t clock_speed_hz; // 时钟频率
        uint8_t mode;            // SPI模式
        uint8_t queue_size;      // 队列大小
    } spi_drv_device_config_t;

    // SPI 驱动实例
    typedef struct
    {
        spi_drv_bus_config_t *bus_config;      // 总线配置（共享）
        spi_drv_device_config_t device_config; // 设备配置（独有）
        spi_device_handle_t device;            // SPI设备句柄
        SemaphoreHandle_t mutex;               // 互斥锁（可选，用于设备级保护）
        bool is_initialized;                   // 初始化标志
    } spi_drv_t;

    // SPI传输结构
    typedef struct
    {
        const uint8_t *tx_buffer; // 发送缓冲区
        uint8_t *rx_buffer;       // 接收缓冲区
        size_t length;            // 传输长度
        bool is_write;            // 是否为写操作
        uint32_t flags;           // 传输标志
    } spi_drv_transfer_t;

    // 全局总线管理
    extern SemaphoreHandle_t g_spi_bus_mutex; // 全局总线互斥锁

    // 基础SPI驱动函数
    bool spiDrvBusInit(spi_drv_bus_config_t *bus_config);
    bool spiDrvDeviceInit(spi_drv_t *spi, spi_drv_bus_config_t *bus_config, const spi_drv_device_config_t *device_config);
    bool spiDrvDeinit(spi_drv_t *spi);
    bool spiDrvTransfer(spi_drv_t *spi, spi_drv_transfer_t *transfer);
    void spiDrvSetCS(spi_drv_t *spi, bool level);

    // 通用寄存器读写函数
    bool spiDrvWriteReg(spi_drv_t *spi, uint8_t reg_addr, const uint8_t *data, size_t length);
    bool spiDrvReadReg(spi_drv_t *spi, uint8_t reg_addr, uint8_t *data, size_t length);
    bool spiDrvWriteByte(spi_drv_t *spi, uint8_t reg_addr, uint8_t data);
    bool spiDrvReadByte(spi_drv_t *spi, uint8_t reg_addr, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_DRV_H__ */
