#ifndef __SENSORS_BMI088_SPI_MS5611_H__
#define __SENSORS_BMI088_SPI_MS5611_H__

#include "sensors.h"

void sensorsBmi088SpiMs5611Init(void);
bool sensorsBmi088SpiMs5611Test(void);
bool sensorsBmi088SpiMs5611AreCalibrated(void);
bool sensorsBmi088SpiMs5611ManufacturingTest(void);

void sensorsBmi088SpiMs5611Acquire(sensorData_t *sensors, const uint32_t tick);
void sensorsBmi088SpiMs5611WaitDataReady(void);

bool sensorsBmi088SpiMs5611ReadGyro(Axis3f *gyro);
bool sensorsBmi088SpiMs5611ReadAcc(Axis3f *acc);
bool sensorsBmi088SpiMs5611ReadMag(Axis3f *mag);
bool sensorsBmi088SpiMs5611ReadBaro(baro_t *baro);

void sensorsBmi088SpiMs5611SetAccMode(accModes accMode);
void sensorsBmi088SpiMs5611DataAvailableCallback(void);

#endif /* __SENSORS_BMI088_SPI_MS5611_H__ */
