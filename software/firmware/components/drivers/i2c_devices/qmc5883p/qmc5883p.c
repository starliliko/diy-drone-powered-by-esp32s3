/**
 *
 * ESP-Drone Firmware
 *
 * QMC5883P magnetometer driver
 *
 */

#include "qmc5883p.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static bool qmc5883pWriteByte(qmc5883p_dev_t *dev, uint8_t reg, uint8_t value);
static bool qmc5883pReadBytes(qmc5883p_dev_t *dev, uint8_t reg, uint8_t len, uint8_t *buffer);

bool qmc5883pReadId(qmc5883p_dev_t *dev, uint8_t *chipId)
{
    if (!dev || !dev->I2Cx || !chipId)
    {
        return false;
    }

    return i2cdevReadByte(dev->I2Cx, dev->devAddr, QMC5883P_REG_ID, chipId);
}

bool qmc5883pSoftReset(qmc5883p_dev_t *dev)
{
    if (!dev || !dev->I2Cx)
    {
        return false;
    }

    return qmc5883pWriteByte(dev, QMC5883P_REG_CONTROL_2, QMC5883P_SOFT_RESET);
}

bool qmc5883pInit(qmc5883p_dev_t *dev, I2C_Dev *i2cPort)
{
    if (!dev || !i2cPort)
    {
        return false;
    }

    dev->I2Cx = i2cPort;
    dev->devAddr = QMC5883P_I2C_ADDR_7BIT;
    dev->control1 = QMC5883P_CONTROL1_DEFAULT;
    dev->control2 = QMC5883P_CONTROL2_DEFAULT;
    dev->isInit = false;

    i2cdevInit(dev->I2Cx);

    uint8_t chipId = 0;
    if (!qmc5883pReadId(dev, &chipId) || chipId != QMC5883P_CHIP_ID)
    {
        return false;
    }

    /*
     * QMC5883P-specific init sequence validated against current QMC5883P
     * module docs:
     *   reg 0x06 <- 0x29
     *   reg 0x0B <- 0x08
     *   reg 0x0A <- 0xCD
     *
     * Note: this is not compatible with legacy QMC5883L code snippets.
     */
    if (!qmc5883pWriteByte(dev, QMC5883P_REG_SET_RESET, QMC5883P_SET_XYZ_SIGN))
    {
        return false;
    }

    if (!qmc5883pWriteByte(dev, QMC5883P_REG_CONTROL_2, dev->control2))
    {
        return false;
    }

    if (!qmc5883pWriteByte(dev, QMC5883P_REG_CONTROL_1, dev->control1))
    {
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
    dev->isInit = true;
    return true;
}

bool qmc5883pTest(qmc5883p_dev_t *dev)
{
    uint8_t chipId = 0;

    if (!dev || !dev->isInit)
    {
        return false;
    }

    if (!qmc5883pReadId(dev, &chipId))
    {
        return false;
    }

    return (chipId == QMC5883P_CHIP_ID);
}

bool qmc5883pReadStatus(qmc5883p_dev_t *dev, uint8_t *status)
{
    if (!dev || !dev->isInit || !status)
    {
        return false;
    }

    return i2cdevReadByte(dev->I2Cx, dev->devAddr, QMC5883P_REG_STATUS, status);
}

bool qmc5883pDataReady(qmc5883p_dev_t *dev)
{
    uint8_t status = 0;

    if (!qmc5883pReadStatus(dev, &status))
    {
        return false;
    }

    return (status & QMC5883P_STATUS_DATA_READY) != 0;
}

bool qmc5883pReadRaw(qmc5883p_dev_t *dev, qmc5883p_raw_data_t *raw)
{
    uint8_t buffer[6];

    if (!dev || !dev->isInit || !raw)
    {
        return false;
    }

    if (!qmc5883pReadBytes(dev, QMC5883P_REG_DATA_X_LSB, sizeof(buffer), buffer))
    {
        return false;
    }

    raw->x = (int16_t)((buffer[1] << 8) | buffer[0]);
    raw->y = (int16_t)((buffer[3] << 8) | buffer[2]);
    raw->z = (int16_t)((buffer[5] << 8) | buffer[4]);

    return true;
}

static bool qmc5883pWriteByte(qmc5883p_dev_t *dev, uint8_t reg, uint8_t value)
{
    return i2cdevWriteByte(dev->I2Cx, dev->devAddr, reg, value);
}

static bool qmc5883pReadBytes(qmc5883p_dev_t *dev, uint8_t reg, uint8_t len, uint8_t *buffer)
{
    return i2cdevReadReg8(dev->I2Cx, dev->devAddr, reg, len, buffer);
}
