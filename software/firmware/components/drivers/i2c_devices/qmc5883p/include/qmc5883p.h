/**
 *
 * ESP-Drone Firmware
 *
 * QMC5883P magnetometer driver
 *
 */
#ifndef QMC5883P_H
#define QMC5883P_H

#include <stdbool.h>
#include <stdint.h>

#include "i2cdev.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * QMC5883P datasheets and Taobao sample code often use 0x58 as the device
 * address. That is the 8-bit bus address (7-bit address left-shifted by one).
 * Our I2C helpers expect the 7-bit address, so the value used in code is 0x2C.
 */
#define QMC5883P_I2C_ADDR_7BIT         0x2C
#define QMC5883P_I2C_ADDR_8BIT         0x58

#define QMC5883P_REG_ID                0x00
#define QMC5883P_REG_DATA_X_LSB        0x01
#define QMC5883P_REG_STATUS            0x09
#define QMC5883P_REG_CONTROL_1         0x0A
#define QMC5883P_REG_CONTROL_2         0x0B
#define QMC5883P_REG_SET_RESET         0x06

#define QMC5883P_CHIP_ID               0x80

#define QMC5883P_STATUS_DATA_READY     0x01

#define QMC5883P_SET_XYZ_SIGN          0x29
#define QMC5883P_SOFT_RESET            0x80
#define QMC5883P_CONTROL2_DEFAULT      0x08
#define QMC5883P_CONTROL1_DEFAULT      0xCD

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} qmc5883p_raw_data_t;

typedef struct
{
    I2C_Dev *I2Cx;
    uint8_t devAddr;
    uint8_t control1;
    uint8_t control2;
    bool isInit;
} qmc5883p_dev_t;

bool qmc5883pInit(qmc5883p_dev_t *dev, I2C_Dev *i2cPort);
bool qmc5883pTest(qmc5883p_dev_t *dev);
bool qmc5883pReadStatus(qmc5883p_dev_t *dev, uint8_t *status);
bool qmc5883pDataReady(qmc5883p_dev_t *dev);
bool qmc5883pReadRaw(qmc5883p_dev_t *dev, qmc5883p_raw_data_t *raw);
bool qmc5883pReadId(qmc5883p_dev_t *dev, uint8_t *chipId);
bool qmc5883pSoftReset(qmc5883p_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif // QMC5883P_H
