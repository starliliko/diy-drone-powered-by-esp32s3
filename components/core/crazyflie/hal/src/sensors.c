/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2018 Bitcraze AB
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
 * sensors.c - Abstraction layer for sensors on a platform. It acts as a
 * proxy to use the correct sensor based on device type.
 */

#define DEBUG_MODULE "SENSORS"

#include "sensors.h"
#include "platform.h"
#include "debug_cf.h"

#define xstr(s) str(s)
#define str(s) #s

#ifdef SENSOR_INCLUDED_BMI088_BMP388
#include "sensors_bmi088_bmp388.h"
#endif

#ifdef SENSOR_INCLUDED_BMI088_SPI_BMP388
#include "sensors_bmi088_spi_bmp388.h"
#endif

#ifdef SENSOR_INCLUDED_MPU9250_LPS25H
#include "sensors_mpu9250_lps25h.h"
#endif

#ifdef SENSOR_INCLUDED_BOSCH
#include "sensors_bosch.h"
#endif

#ifdef SENSOR_INCLUDED_MPU6050_HMC5883L_MS5611
#include "sensors_mpu6050_hm5883L_ms5611.h"
#endif

#ifdef SENSOR_INCLUDED_BMI088_SPI_MS5611
#include "sensors_bmi088_spi_ms5611.h"
#endif

typedef struct
{
  SensorImplementation_t implements;
  void (*init)(void);                                          // 初始化函数指针
  bool (*test)(void);                                          // 测试函数指针
  bool (*areCalibrated)(void);                                 // 校准检查函数指针
  bool (*manufacturingTest)(void);                             // 生产测试函数指针
  void (*acquire)(sensorData_t *sensors, const uint32_t tick); // 采集数据函数指针
  void (*waitDataReady)(void);                                 // 等待数据准备函数指针
  bool (*readGyro)(Axis3f *gyro);                              // 读取陀螺仪数据函数指针
  bool (*readAcc)(Axis3f *acc);                                // 读取加速度计数据函数指针
  bool (*readMag)(Axis3f *mag);                                // 读取磁力计数据函数指针
  bool (*readBaro)(baro_t *baro);                              // 读取气压计数据函数指针
  void (*setAccMode)(accModes accMode);                        // 设置加速度计模式函数指针
  void (*dataAvailableCallback)(void);                         // 数据可用回调函数指针
} sensorsImplementation_t;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
static void nullFunction(void) {}
#pragma GCC diagnostic pop

static const sensorsImplementation_t sensorImplementations[SensorImplementation_COUNT] = {
#ifdef SENSOR_INCLUDED_BMI088_BMP388
    {
        .implements = SensorImplementation_bmi088_bmp388,
        .init = sensorsBmi088Bmp388Init,
        .test = sensorsBmi088Bmp388Test,
        .areCalibrated = sensorsBmi088Bmp388AreCalibrated,
        .manufacturingTest = sensorsBmi088Bmp388ManufacturingTest,
        .acquire = sensorsBmi088Bmp388Acquire,
        .waitDataReady = sensorsBmi088Bmp388WaitDataReady,
        .readGyro = sensorsBmi088Bmp388ReadGyro,
        .readAcc = sensorsBmi088Bmp388ReadAcc,
        .readMag = sensorsBmi088Bmp388ReadMag,
        .readBaro = sensorsBmi088Bmp388ReadBaro,
        .setAccMode = sensorsBmi088Bmp388SetAccMode,
        .dataAvailableCallback = sensorsBmi088Bmp388DataAvailableCallback,
    },
#endif
#ifdef SENSOR_INCLUDED_BMI088_SPI_BMP388
    {
        .implements = SensorImplementation_bmi088_spi_bmp388,
        .init = sensorsBmi088SpiBmp388Init,
        .test = sensorsBmi088SpiBmp388Test,
        .areCalibrated = sensorsBmi088SpiBmp388AreCalibrated,
        .manufacturingTest = sensorsBmi088SpiBmp388ManufacturingTest,
        .acquire = sensorsBmi088SpiBmp388Acquire,
        .waitDataReady = sensorsBmi088SpiBmp388WaitDataReady,
        .readGyro = sensorsBmi088SpiBmp388ReadGyro,
        .readAcc = sensorsBmi088SpiBmp388ReadAcc,
        .readMag = sensorsBmi088SpiBmp388ReadMag,
        .readBaro = sensorsBmi088SpiBmp388ReadBaro,
        .setAccMode = sensorsBmi088SpiBmp388SetAccMode,
        .dataAvailableCallback = sensorsBmi088SpiBmp388DataAvailableCallback,
    },
#endif
#ifdef SENSOR_INCLUDED_MPU6050_HMC5883L_MS5611
    {
        .implements = SensorImplementation_mpu6050_HMC5883L_MS5611,
        .init = sensorsMpu6050Hmc5883lMs5611Init,
        .test = sensorsMpu6050Hmc5883lMs5611Test,
        .areCalibrated = sensorsMpu6050Hmc5883lMs5611AreCalibrated,
        .manufacturingTest = sensorsMpu6050Hmc5883lMs5611ManufacturingTest,
        .acquire = sensorsMpu6050Hmc5883lMs5611Acquire,
        .waitDataReady = sensorsMpu6050Hmc5883lMs5611WaitDataReady,
        .readGyro = sensorsMpu6050Hmc5883lMs5611ReadGyro,
        .readAcc = sensorsMpu6050Hmc5883lMs5611ReadAcc,
        .readMag = sensorsMpu6050Hmc5883lMs5611ReadMag,
        .readBaro = sensorsMpu6050Hmc5883lMs5611ReadBaro,
        .setAccMode = sensorsMpu6050Hmc5883lMs5611SetAccMode,
        .dataAvailableCallback = nullFunction,
    },
#endif
#ifdef SENSOR_INCLUDED_MPU9250_LPS25H
    {
        .implements = SensorImplementation_mpu9250_lps25h,
        .init = sensorsMpu9250Lps25hInit,
        .test = sensorsMpu9250Lps25hTest,
        .areCalibrated = sensorsMpu9250Lps25hAreCalibrated,
        .manufacturingTest = sensorsMpu9250Lps25hManufacturingTest,
        .acquire = sensorsMpu9250Lps25hAcquire,
        .waitDataReady = sensorsMpu9250Lps25hWaitDataReady,
        .readGyro = sensorsMpu9250Lps25hReadGyro,
        .readAcc = sensorsMpu9250Lps25hReadAcc,
        .readMag = sensorsMpu9250Lps25hReadMag,
        .readBaro = sensorsMpu9250Lps25hReadBaro,
        .setAccMode = sensorsMpu9250Lps25hSetAccMode,
        .dataAvailableCallback = nullFunction,
    },
#endif
#ifdef SENSOR_INCLUDED_BOSCH
    {
        .implements = SensorImplementation_bosch,
        .init = sensorsBoschInit,
        .test = sensorsBoschTest,
        .areCalibrated = sensorsBoschAreCalibrated,
        .manufacturingTest = sensorsBoschManufacturingTest,
        .acquire = sensorsBoschAcquire,
        .waitDataReady = sensorsBoschWaitDataReady,
        .readGyro = sensorsBoschReadGyro,
        .readAcc = sensorsBoschReadAcc,
        .readMag = sensorsBoschReadMag,
        .readBaro = sensorsBoschReadBaro,
        .setAccMode = sensorsBoschSetAccMode,
        .dataAvailableCallback = nullFunction,
    },
#endif
#ifdef SENSOR_INCLUDED_BMI088_SPI_MS5611
    {
        .implements = SensorImplementation_bmi088_spi_ms5611,
        .init = sensorsBmi088SpiMs5611Init,
        .test = sensorsBmi088SpiMs5611Test,
        .areCalibrated = sensorsBmi088SpiMs5611AreCalibrated,
        .manufacturingTest = sensorsBmi088SpiMs5611ManufacturingTest,
        .acquire = sensorsBmi088SpiMs5611Acquire,
        .waitDataReady = sensorsBmi088SpiMs5611WaitDataReady,
        .readGyro = sensorsBmi088SpiMs5611ReadGyro,
        .readAcc = sensorsBmi088SpiMs5611ReadAcc,
        .readMag = sensorsBmi088SpiMs5611ReadMag,
        .readBaro = sensorsBmi088SpiMs5611ReadBaro,
        .setAccMode = sensorsBmi088SpiMs5611SetAccMode,
        .dataAvailableCallback = nullFunction,
    },
#endif
};

static const sensorsImplementation_t *activeImplementation;
static bool isInit = false;
static const sensorsImplementation_t *findImplementation(SensorImplementation_t implementation);

void sensorsInit(void)
{
  if (isInit)
  {
    return;
  }

#ifndef SENSORS_FORCE
  SensorImplementation_t sensorImplementation = platformConfigGetSensorImplementation();
#else
  SensorImplementation_t sensorImplementation = SENSORS_FORCE;
  DEBUG_PRINTD("Forcing sensors to " xstr(SENSORS_FORCE) "\n");
#endif

  activeImplementation = findImplementation(sensorImplementation);

  activeImplementation->init();

  isInit = true;
}

bool sensorsTest(void)
{
  return activeImplementation->test();
}

bool sensorsAreCalibrated(void)
{
  return activeImplementation->areCalibrated();
}

bool sensorsManufacturingTest(void)
{
  return activeImplementation->manufacturingTest(); // 修复函数调用
}

void sensorsAcquire(sensorData_t *sensors, const uint32_t tick)
{
  activeImplementation->acquire(sensors, tick);
}

void sensorsWaitDataReady(void)
{
  activeImplementation->waitDataReady();
}

bool sensorsReadGyro(Axis3f *gyro)
{
  return activeImplementation->readGyro(gyro);
}

bool sensorsReadAcc(Axis3f *acc)
{
  return activeImplementation->readAcc(acc);
}

bool sensorsReadMag(Axis3f *mag)
{
  return activeImplementation->readMag(mag);
}

bool sensorsReadBaro(baro_t *baro)
{
  return activeImplementation->readBaro(baro);
}

void sensorsSetAccMode(accModes accMode)
{
  activeImplementation->setAccMode(accMode);
}

void __attribute__((used)) EXTI14_Callback(void)
{
  activeImplementation->dataAvailableCallback();
}

static const sensorsImplementation_t *findImplementation(SensorImplementation_t implementation)
{
  const sensorsImplementation_t *result = 0;

  for (int i = 0; i < SensorImplementation_COUNT; i++)
  {
    if (sensorImplementations[i].implements == implementation)
    {
      result = &sensorImplementations[i];
      break;
    }
  }

  return result;
}
