/**
 * @file sbus.h
 * @brief SBUS protocol driver for ESP32
 *
 * SBUS is a serial protocol used by Futaba, FrSky, and many other RC receivers.
 * It uses inverted UART at 100000 baud, 8E2 (8 data bits, even parity, 2 stop bits).
 *
 * Frame format (25 bytes):
 * - Byte 0: Header (0x0F)
 * - Bytes 1-22: 16 channels, 11 bits each (packed)
 * - Byte 23: Flags (CH17, CH18, frame lost, failsafe)
 * - Byte 24: Footer (0x00)
 *
 * Copyright (C) 2024 ESP-Drone Project
 * Licensed under GPL-3.0
 */

#ifndef __SBUS_H__
#define __SBUS_H__

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* SBUS protocol constants */
#define SBUS_FRAME_SIZE 25
#define SBUS_HEADER 0x0F
#define SBUS_FOOTER 0x00
#define SBUS_NUM_CHANNELS 16
#define SBUS_CHANNEL_MIN 172    // Typical min value
#define SBUS_CHANNEL_MAX 1811   // Typical max value
#define SBUS_CHANNEL_CENTER 992 // Typical center value
#define SBUS_BAUDRATE 100000

/* SBUS flags (byte 23) */
#define SBUS_FLAG_CH17 (1 << 0)
#define SBUS_FLAG_CH18 (1 << 1)
#define SBUS_FLAG_FRAME_LOST (1 << 2)
#define SBUS_FLAG_FAILSAFE (1 << 3)

    /**
     * @brief SBUS frame data structure
     */
    typedef struct
    {
        uint16_t channels[SBUS_NUM_CHANNELS]; // 16 channels, 11-bit values (0-2047)
        bool ch17;                            // Digital channel 17
        bool ch18;                            // Digital channel 18
        bool frameLost;                       // Frame lost flag
        bool failsafe;                        // Failsafe active flag
        uint32_t timestamp;                   // Frame receive timestamp (tick count)
    } sbusFrame_t;

    /**
     * @brief Initialize SBUS driver
     *
     * Configures UART with 100000 baud, 8E2.
     * Note: Hardware inverter is used, no software inversion needed.
     */
    void sbusInit(void);

    /**
     * @brief Check if SBUS driver is initialized
     * @return true if initialized
     */
    bool sbusTest(void);

    /**
     * @brief Check if SBUS signal is available (receiving valid frames)
     * @return true if valid frames are being received
     */
    bool sbusIsAvailable(void);

    /**
     * @brief Check if receiver is in failsafe mode
     * @return true if failsafe is active
     */
    bool sbusIsFailsafe(void);

    /**
     * @brief Get the latest SBUS frame
     * @param frame Pointer to store the frame data
     * @return pdTRUE if new frame available, pdFALSE otherwise
     */
    BaseType_t sbusGetFrame(sbusFrame_t *frame);

    /**
     * @brief Get a single channel value
     * @param channel Channel number (0-15)
     * @return Channel value (0-2047), or 0 if channel invalid
     */
    uint16_t sbusGetChannel(uint8_t channel);

    /**
     * @brief Convert SBUS channel value (0-2047) to normalized float (-1.0 to 1.0)
     * @param value SBUS channel value
     * @return Normalized value
     */
    float sbusConvertToFloat(uint16_t value);

    /**
     * @brief Convert SBUS channel value to uint16 for thrust (0-65535)
     * @param value SBUS channel value
     * @return Thrust value
     */
    uint16_t sbusConvertToUint16(uint16_t value);

    /**
     * @brief Convert SBUS value to angle/rate with custom range
     * @param value SBUS channel value
     * @param minOutput Minimum output value
     * @param maxOutput Maximum output value
     * @return Scaled output value
     */
    float sbusConvertToRange(uint16_t value, float minOutput, float maxOutput);

    /**
     * @brief Parse raw SBUS frame buffer into sbusFrame_t
     * @param buffer Raw 25-byte SBUS frame
     * @param frame Output frame structure
     * @return true if frame is valid
     */
    bool sbusParseFrame(const uint8_t *buffer, sbusFrame_t *frame);

    /**
     * @brief Read and parse SBUS frame from UART
     *
     * Call this periodically from external task to receive SBUS frames.
     * This function handles frame synchronization and parsing.
     *
     * @param frame Output frame structure (filled if new frame available)
     * @param timeoutMs Maximum time to wait for data
     * @return pdTRUE if a new valid frame was received
     */
    BaseType_t sbusReadFrame(sbusFrame_t *frame, uint32_t timeoutMs);

#ifdef __cplusplus
}
#endif

#endif /* __SBUS_H__ */
