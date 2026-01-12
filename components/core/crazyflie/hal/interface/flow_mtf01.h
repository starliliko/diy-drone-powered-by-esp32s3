/**
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020 Espressif Systems (Shanghai)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * flow_mtf01.h - MTF01 optical flow & rangefinder HAL interface
 */

#ifndef __FLOW_MTF01_H__
#define __FLOW_MTF01_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize the MTF01 task.
 * This sets up UART and creates the sensor reading task.
 */
void mtf01TaskInit(void);

/**
 * Test if MTF01 task is initialized.
 * @return true if initialized successfully
 */
bool mtf01TaskTest(void);

/**
 * Get the latest distance measurement in millimeters.
 * @return Distance in mm, 0 if invalid
 */
uint32_t mtf01GetDistance(void);

/**
 * Get the latest flow velocity in X direction.
 * @return Flow velocity in cm/s @ 1m height
 */
int16_t mtf01GetFlowVelX(void);

/**
 * Get the latest flow velocity in Y direction.
 * @return Flow velocity in cm/s @ 1m height
 */
int16_t mtf01GetFlowVelY(void);

/**
 * Get the flow quality indicator.
 * @return Quality value (higher is better)
 */
uint8_t mtf01GetFlowQuality(void);

/**
 * Check if the distance measurement is valid.
 * @return true if distance is valid
 */
bool mtf01IsDistanceValid(void);

/**
 * Check if the flow measurement is valid.
 * @return true if flow is valid
 */
bool mtf01IsFlowValid(void);

#endif /* __FLOW_MTF01_H__ */
