# ESP-Drone AI Development Guide

## Project Overview
ESP-Drone is a quadcopter flight controller firmware based on the **Crazyflie** project (GPL3.0), ported to ESP32/ESP32-S2/ESP32-S3 microcontrollers. The system uses **FreeRTOS** and **ESP-IDF v5.0** framework, providing Wi-Fi/ESP-NOW control with stabilization, height-hold, and position-hold flight modes.

## Architecture

### Core Components Structure
```
components/core/crazyflie/     # Flight control logic (from Crazyflie)
├── hal/                       # Hardware abstraction (sensors, LED, WiFi)
├── modules/                   # Flight modules (stabilizer, estimator, controller)
└── utils/                     # Common utilities (debug, filters, math)

components/drivers/            # Device drivers
├── i2c_devices/              # MPU6050, HMC5883L, MS5611, VL53L1
├── spi_devices/              # PMW3901 optical flow
└── general/                  # Motors, WiFi, ADC, Buzzer

components/platform/           # Hardware platform abstraction
components/config/             # Kconfig-based configuration
```

### Critical System Flow
1. **Initialization**: `main.c` → `platformInit()` → `systemLaunch()` → creates `systemTask`
2. **System Task** (`system.c`): Initializes all subsystems (WiFi, sensors, stabilizer, commander)
3. **Stabilizer Loop** (`stabilizer.c`): 1000Hz control loop reading sensors → estimator → controller → motors
4. **Commander** (`commander.c`): Receives setpoints from WiFi/ESP-NOW and manages flight modes
5. **Estimator** (`estimator_kalman.c`): Fuses sensor data for state estimation (attitude, position)
6. **Controller** (`controller_pid.c`): PID control for attitude/position tracking

## Build & Development

### Build Commands (PowerShell)
```powershell
# Set target chip (ESP32/ESP32S2/ESP32S3)
idf.py set-target esp32s3

# Configure project (opens menuconfig)
idf.py menuconfig

# Build firmware
idf.py build

# Flash and monitor
idf.py -p COM3 flash monitor

# Clean build
idf.py fullclean
```

### Configuration System
- Primary config: `main/Kconfig.projbuild` - defines all hardware variants and GPIO mappings
- Hardware variants: `TARGET_ESPLANE_V1` (ESP32), `TARGET_ESPLANE_V2_S2` (ESP32-S2), `TARGET_ESP32_S2_DRONE_V1_2` (ESP32-S2/S3)
- Defaults: `sdkconfig.defaults` and `sdkconfig.defaults.esp32s3`
- Access via Kconfig: `CONFIG_MOTOR01_PIN`, `CONFIG_I2C0_PIN_SDA`, etc.

### Sensor Support Patterns
Multiple sensor combinations supported via `platformConfig_t` in `platform_cf2.c`:
- `SensorImplementation_mpu6050_HMC5883L_MS5611`: I2C-based (MPU6050 IMU + HMC5883L compass + MS5611 barometer)
- Optical flow deck: SPI-based PMW3901 + VL53L1 (for position hold)

When adding sensor support:
1. Create driver in `components/drivers/i2c_devices/` or `spi_devices/`
2. Add sensor implementation in `components/core/crazyflie/hal/src/sensors_*.c`
3. Update `platformConfig_t` in `platform_cf2.c`

## Project-Specific Conventions

### Task Creation Pattern
Use static allocation macros from `static_mem.h`:
```c
STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE);
STATIC_MEM_TASK_CREATE(sensorsTask, sensorsTask, SENSORS_TASK_NAME, NULL, SENSORS_TASK_PRI);
```

### Debug Printing
Use Crazyflie debug system (not ESP_LOG):
```c
#define DEBUG_MODULE "SENSOR"
#include "debug_cf.h"

DEBUG_PRINT("Initialized\n");
DEBUG_PRINT_LOCAL("Local message\n");
```

### Parameter/Logging System
Use Crazyflie's `param.h` and `log.h` for runtime tuning and telemetry:
```c
PARAM_GROUP_START(controller)
PARAM_ADD(PARAM_FLOAT, kp_roll, &kp_roll)
PARAM_GROUP_STOP(controller)

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_GROUP_STOP(stabilizer)
```

### FreeRTOS Configuration
- Dual-core mode: `CONFIG_FREERTOS_UNICORE=n`
- 1000Hz tick: `CONFIG_FREERTOS_HZ=1000` (critical for 1000Hz stabilizer loop)
- Performance optimization: `CONFIG_COMPILER_OPTIMIZATION_PERF=y`

## Component Dependencies
Key `CMakeLists.txt` pattern:
```cmake
set(PLANE_COMPONENT_DIRS "./components/core" "./components/drivers" ...)
set(EXTRA_COMPONENT_DIRS "${EXTRA_COMPONENT_DIRS} ${PLANE_COMPONENT_DIRS}")
```

Components use `idf_component_register()` with explicit `REQUIRES` to manage dependencies. The `crazyflie` component is the largest, requiring: `i2c_bus deck mpu6050 ms5611 hmc5883l pmw3901 vl53l1 platform config led eeprom dsp_lib motors wifi adc esp_timer`

## Control Flow Integration

### Adding New Flight Modes
1. Extend `setpoint_t` in stabilizer interface
2. Modify `commanderGetSetpoint()` to handle new mode
3. Update controller logic in `controller_pid.c` or create new controller
4. Add parameters/logs for tuning

### Wi-Fi Control
- `wifilink.c`: CRTP (Crazy RealTime Protocol) over UDP
- Compatible with Crazyflie mobile apps (iOS/Android) and cfclient
- ESP-NOW support in `espnow_ctrl.c` for gamepad control

## Hardware Abstraction
GPIO pins configured via menuconfig - never hardcode! Access via:
```c
CONFIG_MOTOR01_PIN
CONFIG_I2C0_PIN_SDA
CONFIG_SPI_PIN_MISO
CONFIG_LED_PIN_BLUE
```

Motor PWM uses LEDC peripheral (see `components/drivers/general/motors/`).

## Common Pitfalls
- **FreeRTOS tick rate**: Must be 1000Hz for stabilizer timing
- **Sensor orientation**: Check `sensorsAccAlignToGravity()` for axis alignment
- **Static memory**: All tasks use static allocation - don't use `xTaskCreate()`
- **Legacy STM32 compatibility**: `stm32_legacy.h` provides HAL-like functions for GPIO
- **Chinese comments**: Mix of English/Chinese - both are acceptable

## Testing & Validation
No automated test suite. Validate changes by:
1. Build for all targets: `esp32`, `esp32s2`, `esp32s3`
2. Flash and verify sensor initialization in logs
3. Test flight modes: stabilize → height-hold → position-hold
4. Monitor parameter/log streams via cfclient

## External Resources
- Crazyflie source reference: https://github.com/bitcraze/crazyflie-firmware (tag_2021_01)
- ESP-IDF docs: https://docs.espressif.com/projects/esp-idf/en/v5.0/
- Hardware schematics: `hardware/` directory
- Mobile apps: https://github.com/EspressifApps/ESP-Drone-iOS and ESP-Drone-Android
