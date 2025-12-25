#define DEBUG_MODULE "IMU" // IMU模块

#include <math.h>

#include "sensors_bmi088_spi_ms5611.h"
#include "debug_cf.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "system.h"
#include "configblock.h"
#include "param.h"
#include "log.h"
#include "imu.h"
#include "nvicconf.h"
#include "ledseq.h"
#include "sound.h"
#include "filter.h"
#include "config.h"
#include "i2cdev.h"
#include "ms5611.h"
#include "static_mem.h"
#include "bmi088_spi.h"
#include "bmi088_config.h"
#include "bmi088.h"

#define GYRO_ADD_RAW_AND_VARIANCE_LOG_VALUES // 启用陀螺仪原始值和方差日志记录

#define SENSORS_READ_RATE_HZ 1000 // 传感器读取频率 (1kHz控制循环)
/*
通过硬件中断同步，已经实现IMU数据对齐与时间同步
*/

#define SENSORS_STARTUP_TIME_MS 2000                                     // 传感器启动时间
#define SENSORS_READ_BARO_HZ 50                                          // 气压计读取频率
#define SENSORS_DELAY_BARO (SENSORS_READ_RATE_HZ / SENSORS_READ_BARO_HZ) // 气压计读取延迟

#define SENSORS_BMI088_GYRO_FS_CFG BMI088_CONFIG_GYRO_RANGE
#define SENSORS_BMI088_GYRO_RANGE_DPS 1000.0f                                              // 陀螺仪量程（度每秒）
#define SENSORS_BMI088_DEG_PER_LSB_CFG ((2.0f * SENSORS_BMI088_GYRO_RANGE_DPS) / 65536.0f) // 每LSB对应的度数

#define SENSORS_BMI088_ACCEL_FS_CFG BMI088_CONFIG_ACC_RANGE // 加速度计量程配置
#define SENSORS_BMI088_ACCEL_G 6.0f
#define SENSORS_BMI088_G_PER_LSB_CFG ((2.0f * SENSORS_BMI088_ACCEL_G) / 65536.0f) // 每LSB对应的重力加速度
#define SENSORS_BMI088_1G_IN_LSB (65536.0f / (2.0f * SENSORS_BMI088_ACCEL_G))     // 1G对应的LSB值

#define GYRO_NBR_OF_AXES 3                     // 陀螺仪轴数
#define GYRO_MIN_BIAS_TIMEOUT_MS M2T(2 * 1000) // 陀螺仪最小偏置超时时间
#define SENSORS_NBR_OF_BIAS_SAMPLES 1024       // 偏置样本数量

// BMI088噪声差异：
//   - BMI088噪声密度: 0.014 °/s/√Hz, 带宽116Hz, 灵敏度32.8 LSB/°/s → ~5 LSB RMS
//   - MPU6050噪声密度: 0.005 °/s/√Hz, 带宽42Hz,  灵敏度16.4 LSB/°/s → ~0.5 LSB RMS
// 实测静止时方差: X~50, Y~70, Z~50 (已归一化)
#define GYRO_VARIANCE_BASE 3000                        // 方差阈值（适配BMI088噪声特性）
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE) // 陀螺仪方差阈值X
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE) // 陀螺仪方差阈值Y
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE) // 陀螺仪方差阈值Z

#define SENSORS_ACC_SCALE_SAMPLES 200 // 加速度计比例样本数量

// 参数适合平稳飞行
#define GYRO_LPF_CUTOFF_FREQ 50  // 陀螺仪低通滤波器截止频率
#define ACCEL_LPF_CUTOFF_FREQ 20 // 加速度计低通滤波器截止频率

#define ESP_INTR_FLAG_DEFAULT 0 // 默认中断优先级

typedef struct
{
    Axis3f bias;                                  // 偏置值
    Axis3f variance;                              // 方差值
    Axis3f mean;                                  // 均值
    bool isBiasValueFound;                        // 是否找到零飘值
    bool isBufferFilled;                          // 缓冲区是否已填满
    Axis3i16 *bufHead;                            // 缓冲区头指针
    Axis3i16 buffer[SENSORS_NBR_OF_BIAS_SAMPLES]; // 数据缓冲区
} BiasObj;

// 静态内存分配队列
static QueueHandle_t accelerometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelerometerDataQueue, 1, sizeof(Axis3f));
static QueueHandle_t gyroDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroDataQueue, 1, sizeof(Axis3f));
static QueueHandle_t magnetometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(magnetometerDataQueue, 1, sizeof(Axis3f));
static QueueHandle_t barometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(barometerDataQueue, 1, sizeof(baro_t));

// 静态内存分配信号量
static SemaphoreHandle_t sensorsDataReady;       // 传感器数据就绪
static StaticSemaphore_t sensorsDataReadyBuffer; // 分配缓冲区
static SemaphoreHandle_t dataReady;              // 数据处理就绪
static StaticSemaphore_t dataReadyBuffer;        // 分配缓冲区

static bool isInit = false; // 初始化标志
/*
    传感器数据结构体，包含陀螺仪、加速度计、磁力计和气压计的数据
*/
static sensorData_t sensorData;

static volatile uint64_t imuIntTimestamp; // IMU中断时间戳

static Axis3i16 gyroRaw;  // 原始陀螺仪数据
static Axis3i16 accelRaw; // 原始加速度计数据

NO_DMA_CCM_SAFE_ZERO_INIT static BiasObj gyroBiasRunning; // 陀螺仪零漂校准对象 //不会被DMA访问
static Axis3f gyroBias;                                   // 陀螺仪零偏值
static bool gyroBiasFound = false;                        // 完成陀螺仪零偏校准

static float accScaleSum = 0;         // 加速度计模长累加和
static float accScale = 1;            // 加速度计比例因子（缩放系数） //按实际调整
static bool accScaleFound = false;    // 是否找到加速度计比例因子
static uint32_t accScaleSumCount = 0; // 加速度计比例因子累加计数

// 二阶低通滤波器
static lpf2pData accLpf[3];  // 加速度计低通滤波器数据
static lpf2pData gyroLpf[3]; // 陀螺仪低通滤波器数据

static bool isBarometerPresent = false;               // 气压计存在标志
static uint8_t baroMeasDelayMin = SENSORS_DELAY_BARO; // 气压计最小测量延迟

// 数据就绪标志
static volatile bool bmi088AccDataReady = false;
static volatile bool bmi088GyroDataReady = false;

// 中断计数器（用于调试）
static volatile uint32_t accIntCount = 0;
static volatile uint32_t gyroIntCount = 0;

// IMU安装角度
static float cosPitch;
static float sinPitch;
static float cosRoll;
static float sinRoll;
#define PITCH_CALIB (CONFIG_PITCH_CALIB * 1.0 / 100) // 俯仰校准角度 单位1度
#define ROLL_CALIB (CONFIG_ROLL_CALIB * 1.0 / 100)

// 气压计跳过开关：设为1则跳过气压计检查（可通过地面站修改）
static uint8_t skipBaroCheck = 0;
// CONFIG_PITCH_CALIB  单位0.01度

static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
static bool processAccScale(int16_t ax, int16_t ay, int16_t az);
static void sensorsBiasObjInit(BiasObj *bias);
static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut);
static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z);
static bool sensorsFindBiasValue(BiasObj *bias);
static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out);
static void applyAxis3fLpf(lpf2pData *data, Axis3f *in);
static void sensorsDeviceInit(void);
static void sensorsInterruptInit(void);
static void sensorsScaleBaro(baro_t *baroScaled);

STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE); // 静态内存分配任务

bool sensorsBmi088SpiMs5611ReadGyro(Axis3f *gyro)
{
    return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

bool sensorsBmi088SpiMs5611ReadAcc(Axis3f *acc)
{
    return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}

bool sensorsBmi088SpiMs5611ReadMag(Axis3f *mag)
{
    return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}

bool sensorsBmi088SpiMs5611ReadBaro(baro_t *baro)
{
    return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}
/* 传感器数据获取函数
   从各个传感器读取数据并存储到传感器数据结构体中
*/
// 底层仍然是调用xQueueReceive
void sensorsBmi088SpiMs5611Acquire(sensorData_t *sensors, const uint32_t tick)
{
    sensorsReadGyro(&sensors->gyro);
    sensorsReadAcc(&sensors->acc);
    sensorsReadMag(&sensors->mag);
    sensorsReadBaro(&sensors->baro);
    sensors->interruptTimestamp = sensorData.interruptTimestamp;
}

bool sensorsBmi088SpiMs5611AreCalibrated(void) // 传感器校准状态检查函数
{
    return gyroBiasFound;
}

static void sensorsTask(void *param)
{
    systemWaitStart();

    vTaskDelay(M2T(200)); // 200ms等待系统稳定

    // sensorsDeviceInit(); // 传感器设备初始化

    Axis3f accScaled;

    uint32_t interruptCount = 0;
    while (1)
    {
        if (pdTRUE == xSemaphoreTake(sensorsDataReady, M2T(1000))) // 1秒超时
        {
            interruptCount++;
            sensorData.interruptTimestamp = imuIntTimestamp;

            // 读取BMI088数据
            bmi088_get_gyro_data(&gyroRaw.x, &gyroRaw.y, &gyroRaw.z);
            bmi088_get_accel_data(&accelRaw.x, &accelRaw.y, &accelRaw.z);

            // 数据读取完成后重新启用中断(电平触发模式需要)
            gpio_intr_enable(BMI088_INT1_PIN);
            gpio_intr_enable(BMI088_INT3_PIN);

            // 陀螺仪校准
            gyroBiasFound = processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);

            if (gyroBiasFound)
            {
                processAccScale(accelRaw.x, accelRaw.y, accelRaw.z);
            }

            // 陀螺仪数据处理
            sensorData.gyro.x = (gyroRaw.x - gyroBias.x) * SENSORS_BMI088_DEG_PER_LSB_CFG;
            sensorData.gyro.y = (gyroRaw.y - gyroBias.y) * SENSORS_BMI088_DEG_PER_LSB_CFG;
            sensorData.gyro.z = (gyroRaw.z - gyroBias.z) * SENSORS_BMI088_DEG_PER_LSB_CFG;
            applyAxis3fLpf((lpf2pData *)(&gyroLpf), &sensorData.gyro);

            // 加速度计数据处理
            accScaled.x = accelRaw.x * SENSORS_BMI088_G_PER_LSB_CFG / accScale;
            accScaled.y = accelRaw.y * SENSORS_BMI088_G_PER_LSB_CFG / accScale;
            accScaled.z = accelRaw.z * SENSORS_BMI088_G_PER_LSB_CFG / accScale;

            sensorsAccAlignToGravity(&accScaled, &sensorData.acc);
            applyAxis3fLpf((lpf2pData *)(&accLpf), &sensorData.acc);
        }

        // 读取气压计数据
        if (isBarometerPresent)
        {
            static uint8_t baroMeasDelay = SENSORS_DELAY_BARO;
            if (--baroMeasDelay == 0)
            {
                sensorsScaleBaro(&sensorData.baro);
                baroMeasDelay = baroMeasDelayMin;

                // // 输出气压计数据（每秒一次，50Hz读取 = 每20次输出一次）
                // static uint8_t baroLogCount = 0;
                // if (++baroLogCount >= 50)
                // {
                //     ESP_LOGI("MS5611", "Pressure: %.2f hPa, Temp: %.2f °C, ASL: %.2f m",
                //              sensorData.baro.pressure, sensorData.baro.temperature, sensorData.baro.asl);
                //     baroLogCount = 0;
                // }
            }
        }

        xQueueOverwrite(accelerometerDataQueue, &sensorData.acc);
        xQueueOverwrite(gyroDataQueue, &sensorData.gyro);
        if (isBarometerPresent)
        {
            xQueueOverwrite(barometerDataQueue, &sensorData.baro);
        }

        xSemaphoreGive(dataReady);

        // 喂狗并让出CPU时间，防止看门狗超时
        taskYIELD();
    }
}

void sensorsBmi088SpiMs5611WaitDataReady(void)
{
    xSemaphoreTake(dataReady, portMAX_DELAY);
}

static void sensorsDeviceInit(void)
{
    if (isInit)
        return;

    spiDrvInit(SPI_DRV_HOST_DEFAULT); // 初始化SPI
    i2cdevInit(I2C0_DEV);             // 初始化I2C

    // 等待传感器启动
    vTaskDelay(M2T(SENSORS_STARTUP_TIME_MS)); // 2秒

    // 初始化BMI088 (SPI)
    if (!bmi088_init())
    {
        DEBUG_PRINTI("BMI088 SPI init [FAIL]\n");
        isInit = false;
        return;
    }
    else
    {
        DEBUG_PRINTI("BMI088 SPI init [OK]\n");
    }

    // 初始化MS5611 (I2C) - 可通过 skipBaroCheck 参数跳过检查
    if (ms5611Init(I2C0_DEV))
    {
        isBarometerPresent = true;
        DEBUG_PRINTI("MS5611 I2C init [OK]\n");
        baroMeasDelayMin = SENSORS_DELAY_BARO;
    }
    else
    {
        isBarometerPresent = false;
        DEBUG_PRINTI("MS5611 I2C init [FAIL]\n");
        if (!skipBaroCheck)
        {
            DEBUG_PRINTI("Barometer required! Set imu_sensors.skipBaro=1 to bypass\n");
            isInit = false;
            return;
        }
        DEBUG_PRINTI("skipBaroCheck=1, continuing without barometer\n");
    }

    // 初始化低通滤波器
    for (uint8_t i = 0; i < 3; i++)
    {
        lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
        lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
    }

    // cosPitch = cosf(configblockGetCalibPitch() * (float)M_PI / 180);
    // sinPitch = sinf(configblockGetCalibPitch() * (float)M_PI / 180);
    // cosRoll = cosf(configblockGetCalibRoll() * (float)M_PI / 180);
    // sinRoll = sinf(configblockGetCalibRoll() * (float)M_PI / 180);
    cosPitch = cosf(PITCH_CALIB * (float)M_PI / 180);
    sinPitch = sinf(PITCH_CALIB * (float)M_PI / 180);
    cosRoll = cosf(ROLL_CALIB * (float)M_PI / 180);
    sinRoll = sinf(ROLL_CALIB * (float)M_PI / 180);
    DEBUG_PRINTI("pitch_calib = %f,roll_calib = %f", PITCH_CALIB, ROLL_CALIB);

    isInit = true;
}

static void sensorsTaskInit(void)
{
    accelerometerDataQueue = STATIC_MEM_QUEUE_CREATE(accelerometerDataQueue);
    gyroDataQueue = STATIC_MEM_QUEUE_CREATE(gyroDataQueue);
    magnetometerDataQueue = STATIC_MEM_QUEUE_CREATE(magnetometerDataQueue);
    barometerDataQueue = STATIC_MEM_QUEUE_CREATE(barometerDataQueue);

    STATIC_MEM_TASK_CREATE(sensorsTask, sensorsTask, SENSORS_TASK_NAME, NULL, SENSORS_TASK_PRI);
}

void sensorsBmi088SpiMs5611Init(void)
{
    if (isInit)
    {
        return;
    }

    sensorsBiasObjInit(&gyroBiasRunning);
    sensorsDeviceInit();
    sensorsInterruptInit(); // 添加中断初始化
    sensorsTaskInit();

    isInit = true;
}

// 加速度计中断处理
static void IRAM_ATTR bmi088_acc_int_handler(void *arg)
{
    accIntCount++;
    // 电平触发需要先禁用中断,防止一直触发
    gpio_intr_disable(BMI088_INT1_PIN);

    bmi088AccDataReady = true;

    // 每个传感器独立触发信号量
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    imuIntTimestamp = usecTimestamp();
    xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

// 陀螺仪中断处理
static void IRAM_ATTR bmi088_gyro_int_handler(void *arg)
{
    gyroIntCount++;
    // 电平触发需要先禁用中断,防止一直触发
    gpio_intr_disable(BMI088_INT3_PIN);

    bmi088GyroDataReady = true;

    // 每个传感器独立触发信号量
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    imuIntTimestamp = usecTimestamp();
    xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

static void sensorsInterruptInit(void)
{
    DEBUG_PRINT("BMI088 interrupt init\n");

    // 配置加速度计中断引脚(INT1) - 低电平触发
    gpio_config_t acc_int_conf = {
#if ESP_IDF_VERSION_MAJOR > 4
        .intr_type = GPIO_INTR_LOW_LEVEL, // 低电平触发(数据就绪时拉低)
#else
        .intr_type = GPIO_PIN_INTR_LOLEVEL,
#endif
        .pin_bit_mask = (1ULL << BMI088_INT1_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE, // 使能上拉,保证空闲时维持高电平
    };
    gpio_config(&acc_int_conf);

    // 配置陀螺仪中断引脚(INT3) - 高电平触发
    gpio_config_t gyro_int_conf = {
#if ESP_IDF_VERSION_MAJOR > 4
        .intr_type = GPIO_INTR_HIGH_LEVEL, // 高电平触发(数据就绪时保持高电平)
#else
        .intr_type = GPIO_PIN_INTR_HILEVEL,
#endif
        .pin_bit_mask = (1ULL << BMI088_INT3_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE, // 使能下拉,保证空闲时为低电平
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&gyro_int_conf);

    // 创建二值信号量
    sensorsDataReady = xSemaphoreCreateBinaryStatic(&sensorsDataReadyBuffer);
    dataReady = xSemaphoreCreateBinaryStatic(&dataReadyBuffer);

    // 注册GPIO中断服务
    gpio_set_intr_type(BMI088_INT1_PIN, GPIO_INTR_LOW_LEVEL);
    gpio_set_intr_type(BMI088_INT3_PIN, GPIO_INTR_HIGH_LEVEL);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    // 绑定中断处理函数
    gpio_isr_handler_add(BMI088_INT1_PIN, bmi088_acc_int_handler, NULL);
    gpio_isr_handler_add(BMI088_INT3_PIN, bmi088_gyro_int_handler, NULL);
}

bool sensorsBmi088SpiMs5611Test(void)
{
    bool testStatus = true;

    if (!isInit)
    {
        DEBUG_PRINT("Sensors not initialized\n");
        return false;
    }

    // 测试BMI088
    if (!bmi088_test())
    {
        DEBUG_PRINT("BMI088 test [FAIL]\n");
        testStatus = false;
    }
    else
    {
        DEBUG_PRINT("BMI088 test [OK]\n");
    }

    // 测试MS5611
    if (isBarometerPresent && !ms5611SelfTest())
    {
        DEBUG_PRINT("MS5611 test [FAIL]\n");
        testStatus = false;
    }

    return testStatus;
}

bool sensorsBmi088SpiMs5611ManufacturingTest(void)
{
    return sensorsBmi088SpiMs5611Test();
}

static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
    if (!accScaleFound)
    {
        accScaleSum += sqrtf(powf(ax * SENSORS_BMI088_G_PER_LSB_CFG, 2) +
                             powf(ay * SENSORS_BMI088_G_PER_LSB_CFG, 2) +
                             powf(az * SENSORS_BMI088_G_PER_LSB_CFG, 2));
        accScaleSumCount++;

        if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
        {
            accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
            accScaleFound = true;
        }
    }

    return accScaleFound;
}

static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
    sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

    if (!gyroBiasRunning.isBiasValueFound)
    {
        sensorsFindBiasValue(&gyroBiasRunning);
        if (gyroBiasRunning.isBiasValueFound)
        {
            soundSetEffect(SND_CALIB);
            ledseqRun(&seq_calibrated);
        }
    }

    gyroBiasOut->x = gyroBiasRunning.bias.x;
    gyroBiasOut->y = gyroBiasRunning.bias.y;
    gyroBiasOut->z = gyroBiasRunning.bias.z;

    return gyroBiasRunning.isBiasValueFound;
}

static void sensorsBiasObjInit(BiasObj *bias)
{
    bias->isBufferFilled = false;
    bias->bufHead = bias->buffer;
}

static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut)
{
    uint32_t i;
    int64_t sum[GYRO_NBR_OF_AXES] = {0};
    int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

    for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
    {
        sum[0] += bias->buffer[i].x;
        sum[1] += bias->buffer[i].y;
        sum[2] += bias->buffer[i].z;
        sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
        sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
        sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
    }

    // 方差公式: Var = [Σx² - (Σx)²/N] / N
    // 注意：必须用浮点运算，避免整数除法截断导致负方差
    varOut->x = ((float)sumSq[0] - ((float)sum[0] * (float)sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES) / SENSORS_NBR_OF_BIAS_SAMPLES;
    varOut->y = ((float)sumSq[1] - ((float)sum[1] * (float)sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES) / SENSORS_NBR_OF_BIAS_SAMPLES;
    varOut->z = ((float)sumSq[2] - ((float)sum[2] * (float)sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES) / SENSORS_NBR_OF_BIAS_SAMPLES;

    meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z)
{
    bias->bufHead->x = x;
    bias->bufHead->y = y;
    bias->bufHead->z = z;
    bias->bufHead++;

    if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES])
    {
        bias->bufHead = bias->buffer;
        bias->isBufferFilled = true;
    }
}

static bool sensorsFindBiasValue(BiasObj *bias)
{
    static int32_t varianceSampleTime;
    bool foundBias = false;

    if (bias->isBufferFilled)
    {
        sensorsCalculateVarianceAndMean(bias, &bias->variance, &bias->mean);

        if (bias->variance.x < GYRO_VARIANCE_THRESHOLD_X &&
            bias->variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
            bias->variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
            (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount()))
        {
            varianceSampleTime = xTaskGetTickCount();
            bias->bias.x = bias->mean.x;
            bias->bias.y = bias->mean.y;
            bias->bias.z = bias->mean.z;
            foundBias = true;
            bias->isBiasValueFound = true;
        }
    }

    return foundBias;
}

static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out)
{
    Axis3f rx;
    Axis3f ry;

    // Rotate around x-axis
    rx.x = in->x;
    rx.y = in->y * cosRoll - in->z * sinRoll;
    rx.z = in->y * sinRoll + in->z * cosRoll;

    // Rotate around y-axis
    ry.x = rx.x * cosPitch - rx.z * sinPitch;
    ry.y = rx.y;
    ry.z = -rx.x * sinPitch + rx.z * cosPitch;

    out->x = ry.x;
    out->y = ry.y;
    out->z = ry.z;
}

void sensorsBmi088SpiMs5611SetAccMode(accModes accMode)
{
    switch (accMode)
    {
    case ACC_MODE_PROPTEST:
        for (uint8_t i = 0; i < 3; i++)
        {
            lpf2pInit(&accLpf[i], 1000, 500);
        }
        break;
    case ACC_MODE_FLIGHT:
    default:
        for (uint8_t i = 0; i < 3; i++)
        {
            lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
        }
        break;
    }
}

static void applyAxis3fLpf(lpf2pData *data, Axis3f *in)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
    }
}

static void sensorsScaleBaro(baro_t *baroScaled) // 气压计读取
{
    float pressure, temperature, asl;
    ms5611GetData(&pressure, &temperature, &asl);
    // ms5611GetData 返回的单位：
    // pressure: mbar (即 hPa)
    // temperature: 摄氏度
    // asl: 米
    baroScaled->pressure = pressure;
    baroScaled->temperature = temperature;
    baroScaled->asl = asl;
}

#ifdef GYRO_ADD_RAW_AND_VARIANCE_LOG_VALUES
LOG_GROUP_START(gyro)
LOG_ADD(LOG_INT16, xRaw, &gyroRaw.x)
LOG_ADD(LOG_INT16, yRaw, &gyroRaw.y)
LOG_ADD(LOG_INT16, zRaw, &gyroRaw.z)
LOG_ADD(LOG_FLOAT, xVariance, &gyroBiasRunning.variance.x)
LOG_ADD(LOG_FLOAT, yVariance, &gyroBiasRunning.variance.y)
LOG_ADD(LOG_FLOAT, zVariance, &gyroBiasRunning.variance.z)
LOG_GROUP_STOP(gyro)
#endif

PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS5611, &isBarometerPresent)
PARAM_ADD(PARAM_UINT8, skipBaro, &skipBaroCheck)
PARAM_GROUP_STOP(imu_sensors)
