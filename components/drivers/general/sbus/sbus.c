/**
 * @file sbus.c
 * @brief SBUS protocol driver implementation for ESP32
 *
 * SBUS uses inverted UART at 100000 baud, 8E2 format.
 * ESP32's UART peripheral supports signal inversion in hardware.
 *
 * Copyright (C) 2024 ESP-Drone Project
 * Licensed under GPL-3.0
 */

#include <string.h>
#include "sbus.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "sdkconfig.h"

#define DEBUG_MODULE "SBUS"
#include "debug_cf.h"

/* Configuration - can be overridden in Kconfig */
#ifndef CONFIG_SBUS_UART_NUM
#define CONFIG_SBUS_UART_NUM UART_NUM_1
#endif

#ifndef CONFIG_SBUS_RX_PIN
#define CONFIG_SBUS_RX_PIN 18 // Default RX pin, change as needed
#endif

#ifndef CONFIG_SBUS_TX_PIN
#define CONFIG_SBUS_TX_PIN UART_PIN_NO_CHANGE // TX not used for SBUS input
#endif

/* SBUS timing */
#define SBUS_TIMEOUT_MS 100 // Signal lost timeout
#define SBUS_RX_BUFFER_SIZE 256

/* Module state */
static bool isInit = false;
static bool isAvailable = false;
static sbusFrame_t latestFrame;
static uint32_t lastFrameTime = 0;

/* Frame receive state */
static uint8_t rxBuffer[SBUS_FRAME_SIZE];
static uint8_t rxBufferIndex = 0;
static TickType_t lastByteTime = 0;

/**
 * @brief Initialize SBUS driver
 */
void sbusInit(void)
{
    if (isInit)
    {
        return;
    }

    /* UART configuration for SBUS: 100000 baud, 8E2 */
    uart_config_t uart_config = {
        .baud_rate = SBUS_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* Install UART driver */
    esp_err_t err = uart_driver_install(CONFIG_SBUS_UART_NUM, SBUS_RX_BUFFER_SIZE, 0, 0, NULL, 0);
    if (err != ESP_OK)
    {
        DEBUG_PRINT("UART driver install failed: %d\n", err);
        return;
    }

    err = uart_param_config(CONFIG_SBUS_UART_NUM, &uart_config);
    if (err != ESP_OK)
    {
        DEBUG_PRINT("UART param config failed: %d\n", err);
        return;
    }

    /* Set pins */
    err = uart_set_pin(CONFIG_SBUS_UART_NUM, CONFIG_SBUS_TX_PIN, CONFIG_SBUS_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
    {
        DEBUG_PRINT("UART set pin failed: %d\n", err);
        return;
    }

    /* Note: Hardware inverter is used, no need for software inversion */
    /* If no hardware inverter, uncomment the following:
    err = uart_set_line_inverse(CONFIG_SBUS_UART_NUM, UART_SIGNAL_RXD_INV);
    if (err != ESP_OK)
    {
        DEBUG_PRINT("UART set inverse failed: %d\n", err);
        return;
    }
    */

    /* Initialize frame data */
    memset(&latestFrame, 0, sizeof(latestFrame));
    for (int i = 0; i < SBUS_NUM_CHANNELS; i++)
    {
        latestFrame.channels[i] = SBUS_CHANNEL_CENTER;
    }

    /* Initialize receive buffer */
    rxBufferIndex = 0;
    lastByteTime = 0;

    isInit = true;
    DEBUG_PRINT("SBUS initialized on UART%d, RX pin %d\n", CONFIG_SBUS_UART_NUM, CONFIG_SBUS_RX_PIN);
}

/**
 * @brief Test if SBUS driver is initialized
 */
bool sbusTest(void)
{
    return isInit;
}

/**
 * @brief Check if SBUS signal is available
 */
bool sbusIsAvailable(void)
{
    if (!isInit)
    {
        return false;
    }

    /* Check if we've received a frame recently */
    uint32_t currentTime = xTaskGetTickCount();
    if ((currentTime - lastFrameTime) > pdMS_TO_TICKS(SBUS_TIMEOUT_MS))
    {
        isAvailable = false;
    }

    return isAvailable && !latestFrame.failsafe;
}

/**
 * @brief Check if receiver is in failsafe mode
 */
bool sbusIsFailsafe(void)
{
    return latestFrame.failsafe;
}

/**
 * @brief Get the latest SBUS frame (non-blocking)
 */
BaseType_t sbusGetFrame(sbusFrame_t *frame)
{
    if (!isInit || frame == NULL)
    {
        return pdFALSE;
    }

    /* Copy the latest frame */
    memcpy(frame, &latestFrame, sizeof(sbusFrame_t));
    return isAvailable ? pdTRUE : pdFALSE;
}

/**
 * @brief Get a single channel value
 */
uint16_t sbusGetChannel(uint8_t channel)
{
    if (!isInit || channel >= SBUS_NUM_CHANNELS)
    {
        return SBUS_CHANNEL_CENTER;
    }

    return latestFrame.channels[channel];
}

/**
 * @brief Convert SBUS value to normalized float (-1.0 to 1.0)
 */
float sbusConvertToFloat(uint16_t value)
{
    /* Map 172-1811 to -1.0 to 1.0 */
    float normalized = ((float)value - SBUS_CHANNEL_CENTER) /
                       ((float)(SBUS_CHANNEL_MAX - SBUS_CHANNEL_MIN) / 2.0f);

    /* Clamp to range */
    if (normalized < -1.0f)
        normalized = -1.0f;
    if (normalized > 1.0f)
        normalized = 1.0f;

    return normalized;
}

/**
 * @brief Convert SBUS value to uint16 for thrust (0-65535)
 */
uint16_t sbusConvertToUint16(uint16_t value)
{
    /* Map 172-1811 to 0-65535 */
    if (value < SBUS_CHANNEL_MIN)
        value = SBUS_CHANNEL_MIN;
    if (value > SBUS_CHANNEL_MAX)
        value = SBUS_CHANNEL_MAX;

    uint32_t result = ((uint32_t)(value - SBUS_CHANNEL_MIN) * 65535) /
                      (SBUS_CHANNEL_MAX - SBUS_CHANNEL_MIN);

    return (uint16_t)result;
}

/**
 * @brief Convert SBUS value to custom range
 */
float sbusConvertToRange(uint16_t value, float minOutput, float maxOutput)
{
    float normalized = sbusConvertToFloat(value);
    return minOutput + (normalized + 1.0f) * 0.5f * (maxOutput - minOutput);
}

/**
 * @brief Parse raw SBUS frame buffer into sbusFrame_t
 * @param buffer Raw 25-byte SBUS frame
 * @param frame Output frame structure
 * @return true if frame is valid
 */
bool sbusParseFrame(const uint8_t *buffer, sbusFrame_t *frame)
{
    /* Validate header */
    if (buffer[0] != SBUS_HEADER)
    {
        return false;
    }

    /* Note: Some receivers don't send the standard footer */
    /* if (buffer[24] != SBUS_FOOTER) return false; */

    /* Parse 16 channels (11 bits each, packed in bytes 1-22) */
    frame->channels[0] = ((buffer[1] | buffer[2] << 8) & 0x07FF);
    frame->channels[1] = ((buffer[2] >> 3 | buffer[3] << 5) & 0x07FF);
    frame->channels[2] = ((buffer[3] >> 6 | buffer[4] << 2 | buffer[5] << 10) & 0x07FF);
    frame->channels[3] = ((buffer[5] >> 1 | buffer[6] << 7) & 0x07FF);
    frame->channels[4] = ((buffer[6] >> 4 | buffer[7] << 4) & 0x07FF);
    frame->channels[5] = ((buffer[7] >> 7 | buffer[8] << 1 | buffer[9] << 9) & 0x07FF);
    frame->channels[6] = ((buffer[9] >> 2 | buffer[10] << 6) & 0x07FF);
    frame->channels[7] = ((buffer[10] >> 5 | buffer[11] << 3) & 0x07FF);
    frame->channels[8] = ((buffer[12] | buffer[13] << 8) & 0x07FF);
    frame->channels[9] = ((buffer[13] >> 3 | buffer[14] << 5) & 0x07FF);
    frame->channels[10] = ((buffer[14] >> 6 | buffer[15] << 2 | buffer[16] << 10) & 0x07FF);
    frame->channels[11] = ((buffer[16] >> 1 | buffer[17] << 7) & 0x07FF);
    frame->channels[12] = ((buffer[17] >> 4 | buffer[18] << 4) & 0x07FF);
    frame->channels[13] = ((buffer[18] >> 7 | buffer[19] << 1 | buffer[20] << 9) & 0x07FF);
    frame->channels[14] = ((buffer[20] >> 2 | buffer[21] << 6) & 0x07FF);
    frame->channels[15] = ((buffer[21] >> 5 | buffer[22] << 3) & 0x07FF);

    /* Parse flags (byte 23) */
    frame->ch17 = (buffer[23] & SBUS_FLAG_CH17) ? true : false;
    frame->ch18 = (buffer[23] & SBUS_FLAG_CH18) ? true : false;
    frame->frameLost = (buffer[23] & SBUS_FLAG_FRAME_LOST) ? true : false;
    frame->failsafe = (buffer[23] & SBUS_FLAG_FAILSAFE) ? true : false;

    frame->timestamp = xTaskGetTickCount();

    return true;
}

/**
 * @brief Read and parse SBUS frame from UART (call this periodically from external task)
 * @param frame Output frame structure (filled if new frame available)
 * @param timeoutMs Maximum time to wait for data
 * @return pdTRUE if a new valid frame was received
 */
BaseType_t sbusReadFrame(sbusFrame_t *frame, uint32_t timeoutMs)
{
    if (!isInit || frame == NULL)
    {
        return pdFALSE;
    }

    uint8_t rxByte;
    TickType_t startTime = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(timeoutMs);

    while ((xTaskGetTickCount() - startTime) < timeout)
    {
        int len = uart_read_bytes(CONFIG_SBUS_UART_NUM, &rxByte, 1, pdMS_TO_TICKS(2));

        if (len > 0)
        {
            TickType_t currentTime = xTaskGetTickCount();

            /* Detect frame start: gap > 2ms indicates new frame */
            if ((currentTime - lastByteTime) > pdMS_TO_TICKS(2) || rxBufferIndex >= SBUS_FRAME_SIZE)
            {
                rxBufferIndex = 0;
            }

            /* Store byte if it's a potential header or we're already receiving */
            if (rxBufferIndex == 0 && rxByte == SBUS_HEADER)
            {
                rxBuffer[rxBufferIndex++] = rxByte;
            }
            else if (rxBufferIndex > 0)
            {
                rxBuffer[rxBufferIndex++] = rxByte;
            }

            lastByteTime = currentTime;

            /* Check if we have a complete frame */
            if (rxBufferIndex == SBUS_FRAME_SIZE)
            {
                if (sbusParseFrame(rxBuffer, frame))
                {
                    /* Update latest frame and state */
                    memcpy(&latestFrame, frame, sizeof(sbusFrame_t));
                    lastFrameTime = currentTime;
                    isAvailable = true;
                    rxBufferIndex = 0;
                    return pdTRUE;
                }
                rxBufferIndex = 0;
            }
        }
    }

    /* Check timeout for signal availability */
    uint32_t currentTime = xTaskGetTickCount();
    if ((currentTime - lastFrameTime) > pdMS_TO_TICKS(SBUS_TIMEOUT_MS))
    {
        isAvailable = false;
    }

    return pdFALSE;
}
