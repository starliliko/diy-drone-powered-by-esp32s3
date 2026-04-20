#define DEBUG_MODULE "IMU" // IMU 模块

#include <math.h>

#include "sensors_bmi088_spi_ms5611_qmc5883p.h"
#include "debug_cf.h"
#include "esp_log.h"

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
#include "estimator_kalman.h" // 提供 quadIsFlying 状态
#include "estimator_kalman_private.h" // taskEstimatorState
#include "estimator.h"   // estimatorEnqueueYawError
#include "qmc5883p.h"

#define GYRO_ADD_RAW_AND_VARIANCE_LOG_VALUES // 记录陀螺仪原始值和方差
#define GYRO_DYNAMIC_CALIBRATION_ENABLED 1   // 启用陀螺仪动态校准

#define SENSORS_READ_RATE_HZ 1000 // 传感器主循环频率（1 kHz）
/*
 * BMI088 陀螺仪和加速度计按中断驱动读取，任务被唤醒后完成缩放、校准和滤波处理。
*/

#define SENSORS_STARTUP_TIME_MS 2000                                     // 上电后等待传感器稳定
#define SENSORS_READ_BARO_HZ 50                                          // 气压计读取频率
#define SENSORS_DELAY_BARO (SENSORS_READ_RATE_HZ / SENSORS_READ_BARO_HZ) // 气压计读取分频

#define SENSORS_READ_MAG_HZ 50                                            // 磁力计读取频率
#define SENSORS_DELAY_MAG (SENSORS_READ_RATE_HZ / SENSORS_READ_MAG_HZ)   // 磁力计读取分频

#define SENSORS_BMI088_GYRO_FS_CFG BMI088_CONFIG_GYRO_RANGE
#define SENSORS_BMI088_GYRO_RANGE_DPS 2000.0f                                              // 陀螺仪量程（dps）
#define SENSORS_BMI088_DEG_PER_LSB_CFG ((2.0f * SENSORS_BMI088_GYRO_RANGE_DPS) / 65536.0f) // 每 LSB 对应的角速度

#define SENSORS_BMI088_ACCEL_FS_CFG BMI088_CONFIG_ACC_RANGE // 加速度计量程配置
#define SENSORS_BMI088_ACCEL_G 6.0f
#define SENSORS_BMI088_G_PER_LSB_CFG ((2.0f * SENSORS_BMI088_ACCEL_G) / 65536.0f) // 每 LSB 对应的加速度（g）
#define SENSORS_BMI088_1G_IN_LSB (65536.0f / (2.0f * SENSORS_BMI088_ACCEL_G))     // 1 g 对应的 LSB

#define GYRO_NBR_OF_AXES 3                        // 陀螺仪轴数
#define GYRO_MIN_BIAS_TIMEOUT_MS M2T(2 * 1000)    // 允许更新零偏前的最短静止时间
#define GYRO_DYNAMIC_CALIB_DELAY_MS M2T(2 * 1000) // 动态校准开始前的延时
#define GYRO_DYNAMIC_CALIB_ALPHA 0.002f           // 动态校准滤波系数（时间常数约 1 s）
#define SENSORS_NBR_OF_BIAS_SAMPLES 1024          // 零偏估计窗口大小

// BMI088 噪声比 MPU6050 略高，这里适当放宽静止判定阈值。
// 实测静止方差大致在 X~50、Y~70、Z~50 附近，留出一定余量。
// 这些阈值主要用于陀螺仪零偏收敛判断。
// 若后续硬件安装或减振方案变化，可按实测重新调整。
#define GYRO_VARIANCE_BASE 3000                        // 陀螺仪静止方差基准阈值
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE) // X 轴方差阈值
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE) // Y 轴方差阈值
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE) // Z 轴方差阈值

#define SENSORS_ACC_SCALE_SAMPLES 200 // 加速度模长平均样本数

// 软件滤波默认参数
#define GYRO_LPF_CUTOFF_FREQ 25  // Lower gyro software LPF a bit more to suppress motor-induced vibration noise
#define GYRO_NOTCH_DEFAULT_ENABLE 1
#define GYRO_NOTCH_DEFAULT_CENTER_FREQ 105.0f
#define GYRO_NOTCH_DEFAULT_BANDWIDTH 15.0f
#define GYRO_NOTCH2_DEFAULT_ENABLE 0
#define GYRO_NOTCH2_DEFAULT_CENTER_FREQ 124.0f
#define GYRO_NOTCH2_DEFAULT_BANDWIDTH 16.0f
#define ACCEL_LPF_CUTOFF_FREQ 20 // 加速度计软件低通截止频率
#define ACCEL_NOTCH_DEFAULT_ENABLE 0
#define ACCEL_NOTCH_DEFAULT_CENTER_FREQ 120.0f
#define ACCEL_NOTCH_DEFAULT_BANDWIDTH 70.0f

#define ESP_INTR_FLAG_DEFAULT 0 // GPIO ISR 服务默认标志

typedef struct
{
    Axis3f bias;                                  // 当前零偏
    Axis3f variance;                              // 当前方差
    Axis3f mean;                                  // 当前均值
    bool isBiasValueFound;                        // 是否已经找到稳定零偏
    bool isBufferFilled;                          // 环形缓冲区是否填满
    Axis3i16 *bufHead;                            // 环形缓冲区写指针
    Axis3i16 buffer[SENSORS_NBR_OF_BIAS_SAMPLES]; // 零偏估计采样缓冲区
} BiasObj;

// 传感器数据队列
static QueueHandle_t accelerometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelerometerDataQueue, 1, sizeof(Axis3f));
static QueueHandle_t gyroDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroDataQueue, 1, sizeof(Axis3f));
static QueueHandle_t magnetometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(magnetometerDataQueue, 1, sizeof(Axis3f));
static QueueHandle_t barometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(barometerDataQueue, 1, sizeof(baro_t));

// 任务同步信号量
static SemaphoreHandle_t sensorsDataReady;       // 传感器中断唤醒信号
static StaticSemaphore_t sensorsDataReadyBuffer; // 静态信号量存储
static SemaphoreHandle_t dataReady;              // 一帧数据处理完成信号
static StaticSemaphore_t dataReadyBuffer;        // 静态信号量存储

static bool isInit = false; // 初始化完成标记
/*
 * 对外提供的最新一帧传感器数据，由采集任务更新，其他模块按需读取。
*/
static sensorData_t sensorData;

static volatile uint64_t imuIntTimestamp; // IMU 中断时间戳

static Axis3i16 gyroRaw;  // 陀螺仪原始数据
static Axis3i16 accelRaw; // 加速度计原始数据

NO_DMA_CCM_SAFE_ZERO_INIT static BiasObj gyroBiasRunning; // 运行中的陀螺仪零偏估计状态
static Axis3f gyroBias;                                   // 当前陀螺仪零偏
static bool gyroBiasFound = false;                        // 是否已完成零偏校准

#ifdef GYRO_DYNAMIC_CALIBRATION_ENABLED
static uint32_t landedTime = 0;                                // 预留：落地持续时间
static bool isLanded = false;                                  // 预留：当前是否落地
static float gyroDynamicCalibAlpha = GYRO_DYNAMIC_CALIB_ALPHA; // 动态校准更新系数
static uint8_t gyroDynamicCalibEnabled = 1;                    // 是否启用动态校准
#endif

static float accScaleSum = 0;         // 加速度模长累加值
static float accScale = 1;            // 加速度缩放系数
static bool accScaleFound = false;    // 是否已得到缩放系数
static uint32_t accScaleSumCount = 0; // 缩放采样计数

// 软件滤波器状态
static lpf2pData accLpf[3];  // 加速度计低通滤波器
static lpf2pData gyroLpf[3]; // 陀螺仪低通滤波器
static lpf2pData gyroNotch[3];
static lpf2pData gyroNotch2[3];
static lpf2pData accNotch[3];

static uint8_t gyroNotchEnabled = GYRO_NOTCH_DEFAULT_ENABLE;
static float gyroNotchCenterFreq = GYRO_NOTCH_DEFAULT_CENTER_FREQ;
static float gyroNotchBandwidth = GYRO_NOTCH_DEFAULT_BANDWIDTH;
static uint8_t gyroNotch2Enabled = GYRO_NOTCH2_DEFAULT_ENABLE;
static float gyroNotch2CenterFreq = GYRO_NOTCH2_DEFAULT_CENTER_FREQ;
static float gyroNotch2Bandwidth = GYRO_NOTCH2_DEFAULT_BANDWIDTH;
static uint8_t accNotchEnabled = ACCEL_NOTCH_DEFAULT_ENABLE;
static float accNotchCenterFreq = ACCEL_NOTCH_DEFAULT_CENTER_FREQ;
static float accNotchBandwidth = ACCEL_NOTCH_DEFAULT_BANDWIDTH;

static uint8_t gyroNotchEnabledApplied = 0;
static float gyroNotchCenterFreqApplied = 0.0f;
static float gyroNotchBandwidthApplied = 0.0f;
static uint8_t gyroNotch2EnabledApplied = 0;
static float gyroNotch2CenterFreqApplied = 0.0f;
static float gyroNotch2BandwidthApplied = 0.0f;
static uint8_t accNotchEnabledApplied = 0;
static float accNotchCenterFreqApplied = 0.0f;
static float accNotchBandwidthApplied = 0.0f;

static bool isBarometerPresent = false;               // 是否检测到气压计
static uint8_t baroMeasDelayMin = SENSORS_DELAY_BARO; // 气压计最小测量间隔

// ========== QMC5883P 磁力计 ==========
static qmc5883p_dev_t magDev;
static bool isMagnetometerPresent = false;
static uint8_t magMeasDelayMin = SENSORS_DELAY_MAG;

// 硬铁校准偏移 (raw LSB)
static float magOffsetX = 0.0f;
static float magOffsetY = 0.0f;
static float magOffsetZ = 0.0f;
// 软铁校准缩放 (1.0 = 无校正)
static float magScaleX = 1.0f;
static float magScaleY = 1.0f;
static float magScaleZ = 1.0f;
// Yaw 融合参数
static float magYawStdDev = 0.3f;   // 磁力计航向测量噪声标准差 (rad)
static uint8_t magYawEnabled = 1;   // 是否启用磁力计航向融合
static float magDeclination = 0.0f; // 磁偏角 (rad)，东偏为正
// 校准状态机
static uint8_t magCalibrating = 0;  // 1=采集中，设回0自动计算
static uint8_t magCalibPrev = 0;
static float magCalibMinX, magCalibMaxX;
static float magCalibMinY, magCalibMaxY;
static float magCalibMinZ, magCalibMaxZ;
static uint32_t magCalibSamples = 0;
// 磁力计调试用原始数据
static Axis3f magRawDebug;          // 校准后的磁力计数据 (gauss)
static float magHeading = 0.0f;     // 校准后的磁航向 (deg)

// BMI088 数据就绪标志
static volatile bool bmi088AccDataReady = false;
static volatile bool bmi088GyroDataReady = false;

// 中断计数，仅用于调试
static volatile uint32_t accIntCount = 0;
static volatile uint32_t gyroIntCount = 0;

// IMU 安装角补偿
static float cosPitch;
static float sinPitch;
static float cosRoll;
static float sinRoll;
#define PITCH_CALIB (CONFIG_PITCH_CALIB * 1.0 / 100) // pitch 校准值，单位度
#define ROLL_CALIB (CONFIG_ROLL_CALIB * 1.0 / 100)

// 若设为 1，则气压计初始化失败时继续启动系统
static uint8_t skipBaroCheck = 0;
// CONFIG_PITCH_CALIB 的单位为 0.01 度

static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
static bool processAccScale(int16_t ax, int16_t ay, int16_t az);
#ifdef GYRO_DYNAMIC_CALIBRATION_ENABLED
static void gyroDynamicCalibUpdate(int16_t gx, int16_t gy, int16_t gz, bool quadIsFlying);
#endif
static void sensorsBiasObjInit(BiasObj *bias);
static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut);
static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z);
static bool sensorsFindBiasValue(BiasObj *bias);
static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out);
static void applyAxis3fLpf(lpf2pData *data, Axis3f *in);
static void applyAxis3fGyroFilters(Axis3f *in);
static void applyAxis3fAccelFilters(Axis3f *in);
static void refreshImuFilterConfig(void);
static void sensorsDeviceInit(void);
static void sensorsInterruptInit(void);
static void sensorsScaleBaro(baro_t *baroScaled);

STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE); // 传感器任务静态栈

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
/* 从各自的队列中读取最新数据，组合成一帧输出。
 * 这里读取的是上一轮任务已经处理完成的数据。
*/
// 注意：这里读取的是缓存结果，不会阻塞等待新中断
void sensorsBmi088SpiMs5611Acquire(sensorData_t *sensors, const uint32_t tick)
{
    sensorsReadGyro(&sensors->gyro);
    sensorsReadAcc(&sensors->acc);
    sensorsReadMag(&sensors->mag);
    sensorsReadBaro(&sensors->baro);
    sensors->interruptTimestamp = sensorData.interruptTimestamp;
}

bool sensorsBmi088SpiMs5611AreCalibrated(void) // 陀螺仪零偏是否已完成校准
{
    return gyroBiasFound;
}

static void sensorsTask(void *param)
{
    systemWaitStart();

    vTaskDelay(M2T(200)); // 等待系统其余模块稳定

    // sensorsDeviceInit(); // 设备初始化已在 init 阶段完成

    Axis3f accScaled;

    uint32_t interruptCount = 0;
    while (1)
    {
        if (pdTRUE == xSemaphoreTake(sensorsDataReady, M2T(1000))) // 1s timeout
        {
            refreshImuFilterConfig();
            interruptCount++;
            sensorData.interruptTimestamp = imuIntTimestamp;

            // ========================================
            // FIX: Check data-ready flags to prevent double-processing
            // ========================================
            // ACC INT and GYRO INT both give the SAME binary semaphore.
            // Without flag check, EVERY wakeup reads+processes BOTH sensors,
            // causing the gyro LPF to process each raw sample TWICE.
            // This distorts the biquad filter state, effectively doubling
            // the noise bandwidth (80Hz designed -> ~160Hz actual).
            bool newGyro = bmi088GyroDataReady;
            bool newAccel = bmi088AccDataReady;

            // Read SPI only for sensors with new data
            int16_t raw_gx, raw_gy, raw_gz;
            int16_t raw_ax, raw_ay, raw_az;
            if (newGyro) {
                bmi088GyroDataReady = false;
                bmi088_get_gyro_data(&raw_gx, &raw_gy, &raw_gz);
                gyroRaw.x = raw_gx;
                gyroRaw.y = raw_gy;
                gyroRaw.z = raw_gz;
            }
            if (newAccel) {
                bmi088AccDataReady = false;
                bmi088_get_accel_data(&raw_ax, &raw_ay, &raw_az);
                accelRaw.x = raw_ax;
                accelRaw.y = raw_ay;
                accelRaw.z = raw_az;
            }

            // Re-enable interrupts after SPI read complete
            gpio_intr_enable(BMI088_INT1_PIN);
            gpio_intr_enable(BMI088_INT3_PIN);

            // Process gyro data (only when fresh sample available)
            if (newGyro) {
                gyroBiasFound = processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
                if (gyroBiasFound) {
                    gyroDynamicCalibUpdate(gyroRaw.x, gyroRaw.y, gyroRaw.z, quadIsFlying);
                }

                sensorData.gyro.x = -(gyroRaw.y - gyroBias.y) * SENSORS_BMI088_DEG_PER_LSB_CFG;
                sensorData.gyro.y = (gyroRaw.x - gyroBias.x) * SENSORS_BMI088_DEG_PER_LSB_CFG;
                sensorData.gyro.z = -(gyroRaw.z - gyroBias.z) * SENSORS_BMI088_DEG_PER_LSB_CFG;
                applyAxis3fGyroFilters(&sensorData.gyro);
            }

            // Process accel data (only when fresh sample available)
            if (newAccel) {
                if (gyroBiasFound) {
                    processAccScale(accelRaw.x, accelRaw.y, accelRaw.z);
                }
                accScaled.x = -accelRaw.y * SENSORS_BMI088_G_PER_LSB_CFG / accScale;
                accScaled.y = accelRaw.x * SENSORS_BMI088_G_PER_LSB_CFG / accScale;
                accScaled.z = accelRaw.z * SENSORS_BMI088_G_PER_LSB_CFG / accScale;
                sensorsAccAlignToGravity(&accScaled, &sensorData.acc);
                applyAxis3fAccelFilters(&sensorData.acc);
            }
        }

        // 按分频读取气压计
        if (isBarometerPresent)
        {
            static uint8_t baroMeasDelay = SENSORS_DELAY_BARO;
            if (--baroMeasDelay == 0)
            {
                sensorsScaleBaro(&sensorData.baro);
                baroMeasDelay = baroMeasDelayMin;
            }
        }

        // ========== 按分频读取磁力计 ==========
        if (isMagnetometerPresent)
        {
            static uint8_t magMeasDelay = SENSORS_DELAY_MAG;
            if (--magMeasDelay == 0)
            {
                qmc5883p_raw_data_t magRaw;
                if (qmc5883pReadRaw(&magDev, &magRaw))
                {
                    float rawX = (float)magRaw.x;
                    float rawY = (float)magRaw.y;
                    float rawZ = (float)magRaw.z;

                    // --- 校准状态机 ---
                    if (magCalibrating && !magCalibPrev)
                    {
                        // 刚开始校准：初始化 min/max
                        magCalibMinX = magCalibMaxX = rawX;
                        magCalibMinY = magCalibMaxY = rawY;
                        magCalibMinZ = magCalibMaxZ = rawZ;
                        magCalibSamples = 0;
                    }
                    if (magCalibrating)
                    {
                        if (rawX < magCalibMinX) magCalibMinX = rawX;
                        if (rawX > magCalibMaxX) magCalibMaxX = rawX;
                        if (rawY < magCalibMinY) magCalibMinY = rawY;
                        if (rawY > magCalibMaxY) magCalibMaxY = rawY;
                        if (rawZ < magCalibMinZ) magCalibMinZ = rawZ;
                        if (rawZ > magCalibMaxZ) magCalibMaxZ = rawZ;
                        magCalibSamples++;
                    }
                    if (!magCalibrating && magCalibPrev && magCalibSamples > 100)
                    {
                        // 校准结束：计算硬铁偏移和软铁缩放
                        magOffsetX = (magCalibMaxX + magCalibMinX) / 2.0f;
                        magOffsetY = (magCalibMaxY + magCalibMinY) / 2.0f;
                        magOffsetZ = (magCalibMaxZ + magCalibMinZ) / 2.0f;
                        float rangeX = (magCalibMaxX - magCalibMinX) / 2.0f;
                        float rangeY = (magCalibMaxY - magCalibMinY) / 2.0f;
                        float rangeZ = (magCalibMaxZ - magCalibMinZ) / 2.0f;
                        float avgRange = (rangeX + rangeY + rangeZ) / 3.0f;
                        if (rangeX > 1.0f) magScaleX = avgRange / rangeX; else magScaleX = 1.0f;
                        if (rangeY > 1.0f) magScaleY = avgRange / rangeY; else magScaleY = 1.0f;
                        if (rangeZ > 1.0f) magScaleZ = avgRange / rangeZ; else magScaleZ = 1.0f;
                        DEBUG_PRINTI("磁力计校准完成: 偏移(%.0f,%.0f,%.0f) 缩放(%.3f,%.3f,%.3f) 采样数=%lu\n",
                                     magOffsetX, magOffsetY, magOffsetZ,
                                     (double)magScaleX, (double)magScaleY, (double)magScaleZ,
                                     (unsigned long)magCalibSamples);
                    }
                    magCalibPrev = magCalibrating;

                    // --- 硬铁/软铁校正 ---
                    float mx_cal = (rawX - magOffsetX) * magScaleX;
                    float my_cal = (rawY - magOffsetY) * magScaleY;
                    float mz_cal = (rawZ - magOffsetZ) * magScaleZ;

                    // 轴映射：与 IMU 坐标系一致（raw Y→-X, raw X→Y, raw Z→Z）
                    sensorData.mag.x = -my_cal;
                    sensorData.mag.y =  mx_cal;
                    sensorData.mag.z =  mz_cal;
                    magRawDebug = sensorData.mag;

                    // --- 倾斜补偿磁航向 → yawError ---
                    if (magYawEnabled && gyroBiasFound)
                    {
                        float roll_rad  = taskEstimatorState.attitude.roll  * (float)M_PI / 180.0f;
                        float pitch_rad = taskEstimatorState.attitude.pitch * (float)M_PI / 180.0f;

                        float cr = cosf(roll_rad);
                        float sr = sinf(roll_rad);
                        float cp = cosf(pitch_rad);
                        float sp = sinf(pitch_rad);

                        float bx = sensorData.mag.x;
                        float by = sensorData.mag.y;
                        float bz = sensorData.mag.z;

                        // 倾斜补偿：将磁力计投影到水平面
                        float mx_comp =  bx * cp + bz * sp;
                        float my_comp =  bx * sr * sp + by * cr - bz * sr * cp;

                        float heading = atan2f(-my_comp, mx_comp) + magDeclination;

                        // 归一化到 [-π, π]
                        while (heading >  (float)M_PI) heading -= 2.0f * (float)M_PI;
                        while (heading < -(float)M_PI) heading += 2.0f * (float)M_PI;

                        magHeading = heading * 180.0f / (float)M_PI;

                        float est_yaw_rad = taskEstimatorState.attitude.yaw * (float)M_PI / 180.0f;
                        float yaw_error = heading - est_yaw_rad;
                        while (yaw_error >  (float)M_PI) yaw_error -= 2.0f * (float)M_PI;
                        while (yaw_error < -(float)M_PI) yaw_error += 2.0f * (float)M_PI;

                        yawErrorMeasurement_t meas = {
                            .yawError = yaw_error,
                            .stdDev   = magYawStdDev
                        };
                        estimatorEnqueueYawError(&meas);
                    }
                }
                magMeasDelay = magMeasDelayMin;
            }
        }

        xQueueOverwrite(accelerometerDataQueue, &sensorData.acc);
        xQueueOverwrite(gyroDataQueue, &sensorData.gyro);
        if (isBarometerPresent)
        {
            xQueueOverwrite(barometerDataQueue, &sensorData.baro);
        }
        if (isMagnetometerPresent)
        {
            xQueueOverwrite(magnetometerDataQueue, &sensorData.mag);
        }

        xSemaphoreGive(dataReady);
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

    spiDrvInit(SPI_DRV_HOST_DEFAULT); // 初始化 SPI 驱动
    i2cdevInit(BAROMETER_I2C_DEV);    // 初始化 I2C 驱动（MS5611）

    // 等待传感器上电稳定
    vTaskDelay(M2T(SENSORS_STARTUP_TIME_MS)); // 2 s

    // 初始化 BMI088（SPI）
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

    // 初始化 MS5611（I2C）；若允许可在失败时跳过
    if (ms5611Init(BAROMETER_I2C_DEV))
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

    // 初始化 QMC5883P（I2C1）
    i2cdevInit(MAGNETOMETER_I2C_DEV);
    vTaskDelay(M2T(50));

    // I2C1 bus scan for diagnostics
    {
        DEBUG_PRINTI("I2C1 bus scan:\n");
        uint8_t dummy;
        int found = 0;
        for (uint8_t addr = 0x08; addr < 0x78; addr++)
        {
            if (i2cdevRead(MAGNETOMETER_I2C_DEV, addr, 1, &dummy))
            {
                DEBUG_PRINTI("  device found at 0x%02X\n", addr);
                found++;
            }
        }
        if (found == 0)
        {
            DEBUG_PRINTW("  no device found on I2C1!\n");
        }
    }

    if (qmc5883pInit(&magDev, MAGNETOMETER_I2C_DEV))
    {
        isMagnetometerPresent = true;
        DEBUG_PRINTI("QMC5883P I2C init [OK]\n");
        magMeasDelayMin = SENSORS_DELAY_MAG;
    }
    else
    {
        isMagnetometerPresent = false;
        DEBUG_PRINTI("QMC5883P I2C init [FAIL]\n");
    }

    // 初始化默认滤波器参数
    for (uint8_t i = 0; i < 3; i++)
    {
        lpf2pInit(&gyroLpf[i], SENSORS_READ_RATE_HZ, GYRO_LPF_CUTOFF_FREQ);
        lpf2pInit(&accLpf[i], SENSORS_READ_RATE_HZ, ACCEL_LPF_CUTOFF_FREQ);
        notchFilterInit(&gyroNotch[i], SENSORS_READ_RATE_HZ, GYRO_NOTCH_DEFAULT_CENTER_FREQ, GYRO_NOTCH_DEFAULT_BANDWIDTH);
        notchFilterInit(&gyroNotch2[i], SENSORS_READ_RATE_HZ, GYRO_NOTCH2_DEFAULT_CENTER_FREQ, GYRO_NOTCH2_DEFAULT_BANDWIDTH);
        notchFilterInit(&accNotch[i], SENSORS_READ_RATE_HZ, ACCEL_NOTCH_DEFAULT_CENTER_FREQ, ACCEL_NOTCH_DEFAULT_BANDWIDTH);
    }

    gyroNotchEnabledApplied = gyroNotchEnabled;
    gyroNotchCenterFreqApplied = gyroNotchCenterFreq;
    gyroNotchBandwidthApplied = gyroNotchBandwidth;
    gyroNotch2EnabledApplied = gyroNotch2Enabled;
    gyroNotch2CenterFreqApplied = gyroNotch2CenterFreq;
    gyroNotch2BandwidthApplied = gyroNotch2Bandwidth;
    accNotchEnabledApplied = accNotchEnabled;
    accNotchCenterFreqApplied = accNotchCenterFreq;
    accNotchBandwidthApplied = accNotchBandwidth;

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
    sensorsInterruptInit(); // 初始化中断
    sensorsTaskInit();

    isInit = true;
}

// BMI088 加速度计数据就绪中断
static void IRAM_ATTR bmi088_acc_int_handler(void *arg)
{
    accIntCount++;
    // 先关闭中断，避免在 SPI 读取完成前重复进入
    gpio_intr_disable(BMI088_INT1_PIN);

    bmi088AccDataReady = true;

    // 唤醒采集任务处理新数据
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    imuIntTimestamp = usecTimestamp();
    xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

// BMI088 陀螺仪数据就绪中断
static void IRAM_ATTR bmi088_gyro_int_handler(void *arg)
{
    gyroIntCount++;
    // 先关闭中断，避免在 SPI 读取完成前重复进入
    gpio_intr_disable(BMI088_INT3_PIN);

    bmi088GyroDataReady = true;

    // 唤醒采集任务处理新数据
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

    // 配置加速度计中断引脚（INT1，低电平触发）
    gpio_config_t acc_int_conf = {
#if ESP_IDF_VERSION_MAJOR > 4
        .intr_type = GPIO_INTR_LOW_LEVEL, // 数据就绪后保持低电平，直到数据被读取
#else
        .intr_type = GPIO_PIN_INTR_LOLEVEL,
#endif
        .pin_bit_mask = (1ULL << BMI088_INT1_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE, // 上拉输入，防止悬空
    };
    gpio_config(&acc_int_conf);

    // 配置陀螺仪中断引脚（INT3，高电平触发）
    gpio_config_t gyro_int_conf = {
#if ESP_IDF_VERSION_MAJOR > 4
        .intr_type = GPIO_INTR_HIGH_LEVEL, // 数据就绪后保持高电平，直到数据被读取
#else
        .intr_type = GPIO_PIN_INTR_HILEVEL,
#endif
        .pin_bit_mask = (1ULL << BMI088_INT3_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE, // 下拉输入，防止悬空
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&gyro_int_conf);

    // 创建同步信号量
    sensorsDataReady = xSemaphoreCreateBinaryStatic(&sensorsDataReadyBuffer);
    dataReady = xSemaphoreCreateBinaryStatic(&dataReadyBuffer);

    // 安装 GPIO 中断服务
    gpio_set_intr_type(BMI088_INT1_PIN, GPIO_INTR_LOW_LEVEL);
    gpio_set_intr_type(BMI088_INT3_PIN, GPIO_INTR_HIGH_LEVEL);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    // 注册两个中断处理函数
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

    // 自检 BMI088
    if (!bmi088_test())
    {
        DEBUG_PRINT("BMI088 test [FAIL]\n");
        testStatus = false;
    }
    else
    {
        DEBUG_PRINT("BMI088 test [OK]\n");
    }

    // 自检 MS5611
    if (isBarometerPresent && !ms5611SelfTest())
    {
        DEBUG_PRINT("MS5611 test [FAIL]\n");
        testStatus = false;
    }

    // 自检 QMC5883P
    if (isMagnetometerPresent && !qmc5883pTest(&magDev))
    {
        DEBUG_PRINT("QMC5883P test [FAIL]\n");
        testStatus = false;
    }
    else if (isMagnetometerPresent)
    {
        DEBUG_PRINT("QMC5883P test [OK]\n");
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

#ifdef GYRO_DYNAMIC_CALIBRATION_ENABLED
/**
 * 在机体静止时缓慢更新陀螺仪零偏，减少温漂带来的累计误差。
 *
 * 只有在角速度足够小且持续一段时间后才会执行更新。
 *
 * @param gx, gy, gz 陀螺仪原始值（LSB）
 * @param quadIsFlying 当前是否处于飞行状态
 */
static void gyroDynamicCalibUpdate(int16_t gx, int16_t gy, int16_t gz, bool quadIsFlying)
{
    if (!gyroDynamicCalibEnabled)
    {
        return; // 未启用动态校准
    }

    // 计算当前测量值相对静态零偏的偏移
    // 偏移越小，越可能处于静止状态。
    float dx = (float)(gx)-gyroBias.x;
    float dy = (float)(gy)-gyroBias.y;
    float dz = (float)(gz)-gyroBias.z;

    // 估算当前角速度模长
    float gyroMagnitude = sqrtf(dx * dx + dy * dy + dz * dz);

    // 约 5 dps 的静止判定阈值。
    // 2000 dps 量程下，1 LSB 约等于 2000 / 32768 = 0.061 dps。
    // 因此这里取 80 LSB 作为近似阈值。
    const float STATIC_THRESHOLD_LSB = 80.0f; // 约 5 dps

    static uint32_t staticCount = 0;
    const uint32_t STATIC_CONFIRM_COUNT = 800; // 1 kHz 下约 0.8 s 连续静止

    if (gyroMagnitude < STATIC_THRESHOLD_LSB)
    {
        staticCount++;
        if (staticCount > STATIC_CONFIRM_COUNT)
        {
            // 确认静止后，以较小步长更新零偏，避免突变
            // 这样可以在不影响飞行的前提下慢慢跟踪温漂。
            float alpha = gyroDynamicCalibAlpha * 2.0f;
            gyroBias.x += alpha * dx;
            gyroBias.y += alpha * dy;
            gyroBias.z += alpha * dz;

            // 保持饱和，避免计数继续无意义增长
            staticCount = STATIC_CONFIRM_COUNT + 1;
        }
    }
    else
    {
        // 一旦检测到运动，重新开始静止计数
        staticCount = 0;
    }
}
#endif

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

    // 方差公式：Var = [sum(x^2) - (sum(x)^2)/N] / N
    // 这里使用等价形式以避免重复遍历缓冲区。
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
            lpf2pInit(&accLpf[i], SENSORS_READ_RATE_HZ, 500);
        }
        break;
    case ACC_MODE_FLIGHT:
    default:
        for (uint8_t i = 0; i < 3; i++)
        {
            lpf2pInit(&accLpf[i], SENSORS_READ_RATE_HZ, ACCEL_LPF_CUTOFF_FREQ);
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

static void applyAxis3fGyroFilters(Axis3f *in)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        if (gyroNotchEnabled)
        {
            in->axis[i] = notchFilterApply(&gyroNotch[i], in->axis[i]);
        }
        if (gyroNotch2Enabled)
        {
            in->axis[i] = notchFilterApply(&gyroNotch2[i], in->axis[i]);
        }
        in->axis[i] = lpf2pApply(&gyroLpf[i], in->axis[i]);
    }
}

static void applyAxis3fAccelFilters(Axis3f *in)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        if (accNotchEnabled)
        {
            in->axis[i] = notchFilterApply(&accNotch[i], in->axis[i]);
        }
        in->axis[i] = lpf2pApply(&accLpf[i], in->axis[i]);
    }
}

static void refreshImuFilterConfig(void)
{
    const bool gyroChanged =
        gyroNotchEnabledApplied != gyroNotchEnabled ||
        fabsf(gyroNotchCenterFreqApplied - gyroNotchCenterFreq) > 0.01f ||
        fabsf(gyroNotchBandwidthApplied - gyroNotchBandwidth) > 0.01f ||
        gyroNotch2EnabledApplied != gyroNotch2Enabled ||
        fabsf(gyroNotch2CenterFreqApplied - gyroNotch2CenterFreq) > 0.01f ||
        fabsf(gyroNotch2BandwidthApplied - gyroNotch2Bandwidth) > 0.01f;

    if (gyroChanged)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            lpf2pInit(&gyroLpf[i], SENSORS_READ_RATE_HZ, GYRO_LPF_CUTOFF_FREQ);
            notchFilterInit(&gyroNotch[i], SENSORS_READ_RATE_HZ, gyroNotchCenterFreq, gyroNotchBandwidth);
            notchFilterInit(&gyroNotch2[i], SENSORS_READ_RATE_HZ, gyroNotch2CenterFreq, gyroNotch2Bandwidth);
        }

        gyroNotchEnabledApplied = gyroNotchEnabled;
        gyroNotchCenterFreqApplied = gyroNotchCenterFreq;
        gyroNotchBandwidthApplied = gyroNotchBandwidth;
        gyroNotch2EnabledApplied = gyroNotch2Enabled;
        gyroNotch2CenterFreqApplied = gyroNotch2CenterFreq;
        gyroNotch2BandwidthApplied = gyroNotch2Bandwidth;
    }

    const bool accChanged =
        accNotchEnabledApplied != accNotchEnabled ||
        fabsf(accNotchCenterFreqApplied - accNotchCenterFreq) > 0.01f ||
        fabsf(accNotchBandwidthApplied - accNotchBandwidth) > 0.01f;

    if (accChanged)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            lpf2pInit(&accLpf[i], SENSORS_READ_RATE_HZ, ACCEL_LPF_CUTOFF_FREQ);
            notchFilterInit(&accNotch[i], SENSORS_READ_RATE_HZ, accNotchCenterFreq, accNotchBandwidth);
        }

        accNotchEnabledApplied = accNotchEnabled;
        accNotchCenterFreqApplied = accNotchCenterFreq;
        accNotchBandwidthApplied = accNotchBandwidth;
    }
}

static void sensorsScaleBaro(baro_t *baroScaled) // 读取并缩放气压计数据
{
    float pressure, temperature, asl;
    ms5611GetData(&pressure, &temperature, &asl);
    // ms5611GetData 输出单位：
    // pressure: mbar（即 hPa）
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

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &magRawDebug.x)
LOG_ADD(LOG_FLOAT, y, &magRawDebug.y)
LOG_ADD(LOG_FLOAT, z, &magRawDebug.z)
LOG_ADD(LOG_FLOAT, heading, &magHeading)
LOG_GROUP_STOP(mag)

PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS5611, &isBarometerPresent)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, QMC5883P, &isMagnetometerPresent)
PARAM_ADD(PARAM_UINT8, skipBaro, &skipBaroCheck)
PARAM_ADD(PARAM_UINT8, gyroNotchEn, &gyroNotchEnabled)
PARAM_ADD(PARAM_FLOAT, gyroNotchHz, &gyroNotchCenterFreq)
PARAM_ADD(PARAM_FLOAT, gyroNotchBw, &gyroNotchBandwidth)
PARAM_ADD(PARAM_UINT8, gyroNotch2En, &gyroNotch2Enabled)
PARAM_ADD(PARAM_FLOAT, gyroNotch2Hz, &gyroNotch2CenterFreq)
PARAM_ADD(PARAM_FLOAT, gyroNotch2Bw, &gyroNotch2Bandwidth)
PARAM_ADD(PARAM_UINT8, accNotchEn, &accNotchEnabled)
PARAM_ADD(PARAM_FLOAT, accNotchHz, &accNotchCenterFreq)
PARAM_ADD(PARAM_FLOAT, accNotchBw, &accNotchBandwidth)
PARAM_GROUP_STOP(imu_sensors)

PARAM_GROUP_START(mag)
PARAM_ADD(PARAM_UINT8, yawEn, &magYawEnabled)
PARAM_ADD(PARAM_FLOAT, yawStdDev, &magYawStdDev)
PARAM_ADD(PARAM_FLOAT, declination, &magDeclination)
PARAM_ADD(PARAM_FLOAT, offX, &magOffsetX)
PARAM_ADD(PARAM_FLOAT, offY, &magOffsetY)
PARAM_ADD(PARAM_FLOAT, offZ, &magOffsetZ)
PARAM_ADD(PARAM_FLOAT, scaleX, &magScaleX)
PARAM_ADD(PARAM_FLOAT, scaleY, &magScaleY)
PARAM_ADD(PARAM_FLOAT, scaleZ, &magScaleZ)
PARAM_ADD(PARAM_UINT8, calibrate, &magCalibrating)
PARAM_GROUP_STOP(mag)
