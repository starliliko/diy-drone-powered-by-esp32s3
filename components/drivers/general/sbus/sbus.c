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
#include <stdio.h>
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

/* Debug options - set to 1 to enable detailed debugging */
#define SBUS_DEBUG_RAW_DATA 0 // Print raw UART bytes
#define SBUS_DEBUG_FRAMES 0   // Print parsed frame data (DISABLED for performance)
#define SBUS_DEBUG_CHANNELS 0 // Print all channel values
#define SBUS_DEBUG_STATS 1    // Print frame statistics (every 500 frames)

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

#ifndef CONFIG_SBUS_SIGNAL_INVERTED
#define CONFIG_SBUS_SIGNAL_INVERTED 1 // Default: enable software inversion
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

/* Debug statistics */
#if SBUS_DEBUG_STATS
static uint32_t framesReceived = 0;
static uint32_t framesValid = 0;
static uint32_t framesInvalid = 0;
static uint32_t failsafeCount = 0;
static uint32_t frameLostCount = 0;
#endif

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
        printf("[SBUS] UART driver install failed: %d\n", err);
        return;
    }

    err = uart_param_config(CONFIG_SBUS_UART_NUM, &uart_config);
    if (err != ESP_OK)
    {
        printf("[SBUS] UART param config failed: %d\n", err);
        return;
    }

    /* Set pins */
    err = uart_set_pin(CONFIG_SBUS_UART_NUM, CONFIG_SBUS_TX_PIN, CONFIG_SBUS_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
    {
        printf("[SBUS] UART set pin failed: %d\n", err);
        return;
    }

    /* SBUS uses inverted signal - enable RX inversion if no hardware inverter */
#if CONFIG_SBUS_SIGNAL_INVERTED
    err = uart_set_line_inverse(CONFIG_SBUS_UART_NUM, UART_SIGNAL_RXD_INV);
    if (err != ESP_OK)
    {
        printf("[SBUS] UART set inverse failed: %d\n", err);
        return;
    }
    printf("[SBUS] RX signal inversion: ENABLED (software)\n");
#else
    printf("[SBUS] RX signal inversion: DISABLED (using hardware inverter)\n");
#endif

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
    printf("[SBUS] SBUS initialized on UART%d, RX pin %d\n", CONFIG_SBUS_UART_NUM, CONFIG_SBUS_RX_PIN);
    printf("[SBUS] Debug enabled - RAW:%d FRAMES:%d CHANNELS:%d STATS:%d\n",
           SBUS_DEBUG_RAW_DATA, SBUS_DEBUG_FRAMES, SBUS_DEBUG_CHANNELS, SBUS_DEBUG_STATS);
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
#if SBUS_DEBUG_RAW_DATA
    /* Print raw buffer - use printf for hex dump */
    printf("SBUS raw [%d bytes]: ", SBUS_FRAME_SIZE);
    for (int i = 0; i < SBUS_FRAME_SIZE; i++)
    {
        printf("%02X ", buffer[i]);
    }
    printf("\n");
#endif

    /* Validate header */
    if (buffer[0] != SBUS_HEADER)
    {
#if SBUS_DEBUG_FRAMES
        printf("[SBUS] Invalid header: 0x%02X (expected 0x%02X)\n", buffer[0], SBUS_HEADER);
#endif
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

#if SBUS_DEBUG_FRAMES
    printf("[SBUS] Frame parsed - Flags:0x%02X FS:%d FL:%d CH17:%d CH18:%d\n",
           buffer[23], frame->failsafe, frame->frameLost, frame->ch17, frame->ch18);
#endif

#if SBUS_DEBUG_CHANNELS
    printf("[SBUS] Channels: ");
    for (int i = 0; i < SBUS_NUM_CHANNELS; i++)
    {
        printf("CH%d:%4d ", i, frame->channels[i]);
    }
    printf("\n");
#endif

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
#if SBUS_DEBUG_STATS
                framesReceived++;
#endif
                if (sbusParseFrame(rxBuffer, frame))
                {
#if SBUS_DEBUG_STATS
                    framesValid++;
                    if (frame->failsafe)
                        failsafeCount++;
                    if (frame->frameLost)
                        frameLostCount++;

                    /* Print stats every 500 frames */
                    if (framesReceived % 500 == 0)
                    {
                        printf("[SBUS] Stats: Total=%lu Valid=%lu Invalid=%lu FS=%lu FL=%lu\n",
                               framesReceived, framesValid, framesInvalid, failsafeCount, frameLostCount);
                        printf("[SBUS] Channels: CH0=%4d CH1=%4d CH2=%4d CH3=%4d CH4=%4d CH5=%4d CH6=%4d CH7=%4d\n",
                               frame->channels[0], frame->channels[1], frame->channels[2], frame->channels[3],
                               frame->channels[4], frame->channels[5], frame->channels[6], frame->channels[7]);
                        printf("[SBUS]           CH8=%4d CH9=%4d CH10=%4d CH11=%4d CH12=%4d CH13=%4d CH14=%4d CH15=%4d\n",
                               frame->channels[8], frame->channels[9], frame->channels[10], frame->channels[11],
                               frame->channels[12], frame->channels[13], frame->channels[14], frame->channels[15]);
                    }
#endif

#if SBUS_DEBUG_FRAMES
                    printf("[SBUS] CH0=%d CH1=%d CH2=%d CH3=%d Thr=%d\n",
                           frame->channels[0], frame->channels[1],
                           frame->channels[2], frame->channels[3],
                           sbusConvertToUint16(frame->channels[2]));
#endif

                    /* Update latest frame and state */
                    memcpy(&latestFrame, frame, sizeof(sbusFrame_t));
                    lastFrameTime = currentTime;
                    isAvailable = true;
                    rxBufferIndex = 0;
                    return pdTRUE;
                }
                else
                {
#if SBUS_DEBUG_STATS
                    framesInvalid++;
#endif
                }
                rxBufferIndex = 0;
            }
        }
#if SBUS_DEBUG_RAW_DATA
        else
        {
            /* No data available */
            vTaskDelay(pdMS_TO_TICKS(1));
        }
#endif
    }

    /* Check timeout for signal availability */
    uint32_t currentTime = xTaskGetTickCount();
    if ((currentTime - lastFrameTime) > pdMS_TO_TICKS(SBUS_TIMEOUT_MS))
    {
        isAvailable = false;
    }

    return pdFALSE;
}
