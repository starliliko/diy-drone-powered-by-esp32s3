/**
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2018 Bitcraze AB
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
 * Generic platform functionality
 *
 */

/*
通用平台功能实现
这里同时初始化和定义了不同硬件平台的配置
和crazyflie中的sensors.c关联

*/

#include <string.h>

#include "platform.h"
#define DEBUG_MODULE "PLATFORM"
#include "debug_cf.h"

static const platformConfig_t *active_config = 0;

int platformInit(void)
{
    /*platform layer used to support different hardware configuration
     * get plantform type from memory and
     * Initilizes all platform related things
     * */

    int nrOfConfigs;
    const platformConfig_t *configs = platformGetListOfConfigurations(&nrOfConfigs);

    /*init platform configuration, copy the configuration index to active_config */
    int err = platformInitConfiguration(configs, nrOfConfigs);

    if (err != 0)
    {
        DEBUG_PRINT_LOCAL("This firmware is not compatible, abort init");
        return false;
    }

    return platformInitHardware();
}

int platformParseDeviceTypeString(const char *deviceTypeString, char *deviceType)
{
    if (deviceTypeString[0] != '0' || deviceTypeString[1] != ';')
    {
        return 1;
    }

    const int start = 2;
    const int last = start + PLATFORM_DEVICE_TYPE_MAX_LEN - 1;
    int end = 0;

    for (end = start; end <= last; end++)
    {
        if (deviceTypeString[end] == '\0' || deviceTypeString[end] == ';')
        {
            break;
        }
    }

    if (end > last)
    {
        return 1;
    }

    int length = end - start;
    memcpy(deviceType, &deviceTypeString[start], length);
    deviceType[length] = '\0';
    return 0;
}

int platformInitConfiguration(const platformConfig_t *configs, const int nrOfConfigs)
{
#ifndef DEVICE_TYPE_STRING_FORCE
    char deviceTypeString[PLATFORM_DEVICE_TYPE_STRING_MAX_LEN];
    char deviceType[PLATFORM_DEVICE_TYPE_MAX_LEN];
    platformGetDeviceTypeString(deviceTypeString);               //"0;EP20"
    platformParseDeviceTypeString(deviceTypeString, deviceType); // deviceType="EP20"
#else
#define xstr(s) str(s)
#define str(s) #s
    char *deviceType = xstr(DEVICE_TYPE_STRING_FORCE);
#endif

    for (int i = 0; i < nrOfConfigs; i++)
    {
        const platformConfig_t *config = &configs[i];

        if (strcmp(config->deviceType, deviceType) == 0)
        {
            active_config = config;
            DEBUG_PRINT_LOCAL("set active config ");
            return 0;
        }
    }

    return 1;
}

const char *platformConfigGetDeviceType()
{
    return active_config->deviceType;
}

const char *platformConfigGetDeviceTypeName()
{
    return active_config->deviceTypeName;
}

SensorImplementation_t platformConfigGetSensorImplementation()
{
    return active_config->sensorImplementation;
}

bool platformConfigPhysicalLayoutAntennasAreClose()
{
    return active_config->physicalLayoutAntennasAreClose;
}

const MotorPerifDef **platformConfigGetMotorMapping()
{
    return active_config->motorMap;
}

// 平台类型字符串获取（原本在 platform_esp32.c）
#define PLATFORM_INFO_OTP_BLOCK_LEN 32
#define DEFAULT_PLATFORM_STRING "0;EP20"

void platformGetDeviceTypeString(char *deviceTypeString)
{
    // 这里直接使用默认字符串，如需支持 OTP，可自行扩展
    strncpy(deviceTypeString, DEFAULT_PLATFORM_STRING, PLATFORM_INFO_OTP_BLOCK_LEN);
    deviceTypeString[PLATFORM_INFO_OTP_BLOCK_LEN] = '\0';
}

// 平台配置列表（原本在 platform_esp32.c 或 platform_cf2.c）
static const platformConfig_t platformConfigs[] = {
    {
        .deviceType = "EP20",
        .deviceTypeName = "ESP-Drone EP20",
        .sensorImplementation = SensorImplementation_mpu6050_HMC5883L_MS5611,
        .physicalLayoutAntennasAreClose = false,
        .motorMap = NULL, // 根据实际情况填写
    },
    // 可继续添加其他平台配置
};

const platformConfig_t *platformGetListOfConfigurations(int *nrOfConfigs)
{
    *nrOfConfigs = sizeof(platformConfigs) / sizeof(platformConfigs[0]);
    return platformConfigs;
}

// 平台硬件初始化（如有需要可实现，否则直接返回 true）
bool platformInitHardware()
{
    // 可根据实际硬件初始化需求实现
    return true;
}
