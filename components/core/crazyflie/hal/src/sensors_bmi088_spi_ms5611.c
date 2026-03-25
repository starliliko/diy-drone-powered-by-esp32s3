#define DEBUG_MODULE "IMU" // IMU妯″潡

#include <math.h>

#include "sensors_bmi088_spi_ms5611.h"
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
#include "estimator_kalman.h" // 鐢ㄤ簬鑾峰彇quadIsFlying鐘舵€?

#define GYRO_ADD_RAW_AND_VARIANCE_LOG_VALUES // 鍚敤闄€铻轰华鍘熷鍊煎拰鏂瑰樊鏃ュ織璁板綍
#define GYRO_DYNAMIC_CALIBRATION_ENABLED 1   // 鍚敤闄€铻轰华鍔ㄦ€佹牎鍑?

#define SENSORS_READ_RATE_HZ 1000 // 浼犳劅鍣ㄨ鍙栭鐜?(1kHz鎺у埗寰幆)
/*
閫氳繃纭欢涓柇鍚屾锛屽凡缁忓疄鐜癐MU鏁版嵁瀵归綈涓庢椂闂村悓姝?
*/

#define SENSORS_STARTUP_TIME_MS 2000                                     // 浼犳劅鍣ㄥ惎鍔ㄦ椂闂?
#define SENSORS_READ_BARO_HZ 50                                          // 姘斿帇璁¤鍙栭鐜?
#define SENSORS_DELAY_BARO (SENSORS_READ_RATE_HZ / SENSORS_READ_BARO_HZ) // 姘斿帇璁¤鍙栧欢杩?

#define SENSORS_BMI088_GYRO_FS_CFG BMI088_CONFIG_GYRO_RANGE
#define SENSORS_BMI088_GYRO_RANGE_DPS 2000.0f                                              // 闄€铻轰华閲忕▼锛堝害姣忕锛?
#define SENSORS_BMI088_DEG_PER_LSB_CFG ((2.0f * SENSORS_BMI088_GYRO_RANGE_DPS) / 65536.0f) // 姣廘SB瀵瑰簲鐨勫害鏁?

#define SENSORS_BMI088_ACCEL_FS_CFG BMI088_CONFIG_ACC_RANGE // 鍔犻€熷害璁￠噺绋嬮厤缃?
#define SENSORS_BMI088_ACCEL_G 6.0f
#define SENSORS_BMI088_G_PER_LSB_CFG ((2.0f * SENSORS_BMI088_ACCEL_G) / 65536.0f) // 姣廘SB瀵瑰簲鐨勯噸鍔涘姞閫熷害
#define SENSORS_BMI088_1G_IN_LSB (65536.0f / (2.0f * SENSORS_BMI088_ACCEL_G))     // 1G瀵瑰簲鐨凩SB鍊?

#define GYRO_NBR_OF_AXES 3                        // 闄€铻轰华杞存暟
#define GYRO_MIN_BIAS_TIMEOUT_MS M2T(2 * 1000)    // 闄€铻轰华鏈€灏忓亸缃秴鏃舵椂闂?
#define GYRO_DYNAMIC_CALIB_DELAY_MS M2T(2 * 1000) // 鍔ㄦ€佹牎鍑嗗欢杩熸椂闂达紙鐫€闄嗗悗2绉掑紑濮嬶級
#define GYRO_DYNAMIC_CALIB_ALPHA 0.002f           // 鍔ㄦ€佹牎鍑嗕綆閫氭护娉㈢郴鏁帮紙time constant ~1s锛?
#define SENSORS_NBR_OF_BIAS_SAMPLES 1024          // 鍋忕疆鏍锋湰鏁伴噺

// BMI088鍣０宸紓锛?
//   - BMI088鍣０瀵嗗害: 0.014 掳/s/鈭欻z, 甯﹀116Hz, 鐏垫晱搴?2.8 LSB/掳/s 鈫?~5 LSB RMS
//   - MPU6050鍣０瀵嗗害: 0.005 掳/s/鈭欻z, 甯﹀42Hz,  鐏垫晱搴?6.4 LSB/掳/s 鈫?~0.5 LSB RMS
// 瀹炴祴闈欐鏃舵柟宸? X~50, Y~70, Z~50 (宸插綊涓€鍖?
#define GYRO_VARIANCE_BASE 3000                        // 鏂瑰樊闃堝€硷紙閫傞厤BMI088鍣０鐗规€э級
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE) // 闄€铻轰华鏂瑰樊闃堝€糥
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE) // 闄€铻轰华鏂瑰樊闃堝€糦
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE) // 闄€铻轰华鏂瑰樊闃堝€糧

#define SENSORS_ACC_SCALE_SAMPLES 200 // 鍔犻€熷害璁℃瘮渚嬫牱鏈暟閲?

// 鍙傛暟閫傚悎骞崇ǔ椋炶
#define GYRO_LPF_CUTOFF_FREQ 25  // Lower gyro software LPF a bit more to suppress motor-induced vibration noise
#define GYRO_NOTCH_DEFAULT_ENABLE 1
#define GYRO_NOTCH_DEFAULT_CENTER_FREQ 105.0f
#define GYRO_NOTCH_DEFAULT_BANDWIDTH 15.0f
#define GYRO_NOTCH2_DEFAULT_ENABLE 0
#define GYRO_NOTCH2_DEFAULT_CENTER_FREQ 124.0f
#define GYRO_NOTCH2_DEFAULT_BANDWIDTH 16.0f
#define ACCEL_LPF_CUTOFF_FREQ 20 // 鍔犻€熷害璁′綆閫氭护娉㈠櫒鎴棰戠巼
#define ACCEL_NOTCH_DEFAULT_ENABLE 0
#define ACCEL_NOTCH_DEFAULT_CENTER_FREQ 120.0f
#define ACCEL_NOTCH_DEFAULT_BANDWIDTH 70.0f

#define ESP_INTR_FLAG_DEFAULT 0 // 榛樿涓柇浼樺厛绾?

typedef struct
{
    Axis3f bias;                                  // 鍋忕疆鍊?
    Axis3f variance;                              // 鏂瑰樊鍊?
    Axis3f mean;                                  // 鍧囧€?
    bool isBiasValueFound;                        // 鏄惁鎵惧埌闆堕鍊?
    bool isBufferFilled;                          // 缂撳啿鍖烘槸鍚﹀凡濉弧
    Axis3i16 *bufHead;                            // 缂撳啿鍖哄ご鎸囬拡
    Axis3i16 buffer[SENSORS_NBR_OF_BIAS_SAMPLES]; // 鏁版嵁缂撳啿鍖?
} BiasObj;

// 闈欐€佸唴瀛樺垎閰嶉槦鍒?
static QueueHandle_t accelerometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelerometerDataQueue, 1, sizeof(Axis3f));
static QueueHandle_t gyroDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroDataQueue, 1, sizeof(Axis3f));
static QueueHandle_t magnetometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(magnetometerDataQueue, 1, sizeof(Axis3f));
static QueueHandle_t barometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(barometerDataQueue, 1, sizeof(baro_t));

// 闈欐€佸唴瀛樺垎閰嶄俊鍙烽噺
static SemaphoreHandle_t sensorsDataReady;       // 浼犳劅鍣ㄦ暟鎹氨缁?
static StaticSemaphore_t sensorsDataReadyBuffer; // 鍒嗛厤缂撳啿鍖?
static SemaphoreHandle_t dataReady;              // 鏁版嵁澶勭悊灏辩华
static StaticSemaphore_t dataReadyBuffer;        // 鍒嗛厤缂撳啿鍖?

static bool isInit = false; // 鍒濆鍖栨爣蹇?
/*
    浼犳劅鍣ㄦ暟鎹粨鏋勪綋锛屽寘鍚檧铻轰华銆佸姞閫熷害璁°€佺鍔涜鍜屾皵鍘嬭鐨勬暟鎹?
*/
static sensorData_t sensorData;

static volatile uint64_t imuIntTimestamp; // IMU涓柇鏃堕棿鎴?

static Axis3i16 gyroRaw;  // 鍘熷闄€铻轰华鏁版嵁
static Axis3i16 accelRaw; // 鍘熷鍔犻€熷害璁℃暟鎹?

NO_DMA_CCM_SAFE_ZERO_INIT static BiasObj gyroBiasRunning; // 闄€铻轰华闆舵紓鏍″噯瀵硅薄 //涓嶄細琚獶MA璁块棶
static Axis3f gyroBias;                                   // 闄€铻轰华闆跺亸鍊?
static bool gyroBiasFound = false;                        // 瀹屾垚闄€铻轰华闆跺亸鏍″噯

#ifdef GYRO_DYNAMIC_CALIBRATION_ENABLED
static uint32_t landedTime = 0;                                // 鐫€闄嗘椂闂存埑
static bool isLanded = false;                                  // 鐫€闄嗙姸鎬佹爣蹇?
static float gyroDynamicCalibAlpha = GYRO_DYNAMIC_CALIB_ALPHA; // 鍔ㄦ€佹牎鍑嗘护娉㈢郴鏁帮紙鍙皟鍙傛暟锛?
static uint8_t gyroDynamicCalibEnabled = 1;                    // 鍔ㄦ€佹牎鍑嗕娇鑳藉紑鍏?
#endif

static float accScaleSum = 0;         // 鍔犻€熷害璁℃ā闀跨疮鍔犲拰
static float accScale = 1;            // 鍔犻€熷害璁℃瘮渚嬪洜瀛愶紙缂╂斁绯绘暟锛?//鎸夊疄闄呰皟鏁?
static bool accScaleFound = false;    // 鏄惁鎵惧埌鍔犻€熷害璁℃瘮渚嬪洜瀛?
static uint32_t accScaleSumCount = 0; // 鍔犻€熷害璁℃瘮渚嬪洜瀛愮疮鍔犺鏁?

// 浜岄樁浣庨€氭护娉㈠櫒
static lpf2pData accLpf[3];  // 鍔犻€熷害璁′綆閫氭护娉㈠櫒鏁版嵁
static lpf2pData gyroLpf[3]; // 闄€铻轰华浣庨€氭护娉㈠櫒鏁版嵁
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

static bool isBarometerPresent = false;               // 姘斿帇璁″瓨鍦ㄦ爣蹇?
static uint8_t baroMeasDelayMin = SENSORS_DELAY_BARO; // 姘斿帇璁℃渶灏忔祴閲忓欢杩?

// 鏁版嵁灏辩华鏍囧織
static volatile bool bmi088AccDataReady = false;
static volatile bool bmi088GyroDataReady = false;

// 涓柇璁℃暟鍣紙鐢ㄤ簬璋冭瘯锛?
static volatile uint32_t accIntCount = 0;
static volatile uint32_t gyroIntCount = 0;

// IMU瀹夎瑙掑害
static float cosPitch;
static float sinPitch;
static float cosRoll;
static float sinRoll;
#define PITCH_CALIB (CONFIG_PITCH_CALIB * 1.0 / 100) // 淇话鏍″噯瑙掑害 鍗曚綅1搴?
#define ROLL_CALIB (CONFIG_ROLL_CALIB * 1.0 / 100)

// 姘斿帇璁¤烦杩囧紑鍏筹細璁句负1鍒欒烦杩囨皵鍘嬭妫€鏌ワ紙鍙€氳繃鍦伴潰绔欎慨鏀癸級
static uint8_t skipBaroCheck = 0;
// CONFIG_PITCH_CALIB  鍗曚綅0.01搴?

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

STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE); // 闈欐€佸唴瀛樺垎閰嶄换鍔?

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
/* 浼犳劅鍣ㄦ暟鎹幏鍙栧嚱鏁?
   浠庡悇涓紶鎰熷櫒璇诲彇鏁版嵁骞跺瓨鍌ㄥ埌浼犳劅鍣ㄦ暟鎹粨鏋勪綋涓?
*/
// 搴曞眰浠嶇劧鏄皟鐢▁QueueReceive
void sensorsBmi088SpiMs5611Acquire(sensorData_t *sensors, const uint32_t tick)
{
    sensorsReadGyro(&sensors->gyro);
    sensorsReadAcc(&sensors->acc);
    sensorsReadMag(&sensors->mag);
    sensorsReadBaro(&sensors->baro);
    sensors->interruptTimestamp = sensorData.interruptTimestamp;
}

bool sensorsBmi088SpiMs5611AreCalibrated(void) // 浼犳劅鍣ㄦ牎鍑嗙姸鎬佹鏌ュ嚱鏁?
{
    return gyroBiasFound;
}

static void sensorsTask(void *param)
{
    systemWaitStart();

    vTaskDelay(M2T(200)); // 200ms绛夊緟绯荤粺绋冲畾

    // sensorsDeviceInit(); // 浼犳劅鍣ㄨ澶囧垵濮嬪寲

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
            // the noise bandwidth (80Hz designed 鈫?~160Hz actual).
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

        // 璇诲彇姘斿帇璁℃暟鎹?
        if (isBarometerPresent)
        {
            static uint8_t baroMeasDelay = SENSORS_DELAY_BARO;
            if (--baroMeasDelay == 0)
            {
                sensorsScaleBaro(&sensorData.baro);
                baroMeasDelay = baroMeasDelayMin;

                // 杈撳嚭姘斿帇璁℃暟鎹紙姣忕涓€娆★紝50Hz璇诲彇 = 姣?0娆¤緭鍑轰竴娆★級
                // static uint8_t baroLogCount = 0;
                // if (++baroLogCount >= 50)
                // {
                //     printf("%.2f, %.2f, %.2f \n",
                //            sensorData.baro.pressure, sensorData.baro.temperature, sensorData.baro.asl);
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

    spiDrvInit(SPI_DRV_HOST_DEFAULT); // 鍒濆鍖朣PI
    i2cdevInit(I2C0_DEV);             // 鍒濆鍖朓2C

    // 绛夊緟浼犳劅鍣ㄥ惎鍔?
    vTaskDelay(M2T(SENSORS_STARTUP_TIME_MS)); // 2绉?

    // 鍒濆鍖朆MI088 (SPI)
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

    // 鍒濆鍖朚S5611 (I2C) - 鍙€氳繃 skipBaroCheck 鍙傛暟璺宠繃妫€鏌?
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

    // 鍒濆鍖栦綆閫氭护娉㈠櫒
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
    sensorsInterruptInit(); // 娣诲姞涓柇鍒濆鍖?
    sensorsTaskInit();

    isInit = true;
}

// 鍔犻€熷害璁′腑鏂鐞?
static void IRAM_ATTR bmi088_acc_int_handler(void *arg)
{
    accIntCount++;
    // 鐢靛钩瑙﹀彂闇€瑕佸厛绂佺敤涓柇,闃叉涓€鐩磋Е鍙?
    gpio_intr_disable(BMI088_INT1_PIN);

    bmi088AccDataReady = true;

    // 姣忎釜浼犳劅鍣ㄧ嫭绔嬭Е鍙戜俊鍙烽噺
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    imuIntTimestamp = usecTimestamp();
    xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

// 闄€铻轰华涓柇澶勭悊
static void IRAM_ATTR bmi088_gyro_int_handler(void *arg)
{
    gyroIntCount++;
    // 鐢靛钩瑙﹀彂闇€瑕佸厛绂佺敤涓柇,闃叉涓€鐩磋Е鍙?
    gpio_intr_disable(BMI088_INT3_PIN);

    bmi088GyroDataReady = true;

    // 姣忎釜浼犳劅鍣ㄧ嫭绔嬭Е鍙戜俊鍙烽噺
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

    // 閰嶇疆鍔犻€熷害璁′腑鏂紩鑴?INT1) - 浣庣數骞宠Е鍙?
    gpio_config_t acc_int_conf = {
#if ESP_IDF_VERSION_MAJOR > 4
        .intr_type = GPIO_INTR_LOW_LEVEL, // 浣庣數骞宠Е鍙?鏁版嵁灏辩华鏃舵媺浣?
#else
        .intr_type = GPIO_PIN_INTR_LOLEVEL,
#endif
        .pin_bit_mask = (1ULL << BMI088_INT1_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE, // 浣胯兘涓婃媺,淇濊瘉绌洪棽鏃剁淮鎸侀珮鐢靛钩
    };
    gpio_config(&acc_int_conf);

    // 閰嶇疆闄€铻轰华涓柇寮曡剼(INT3) - 楂樼數骞宠Е鍙?
    gpio_config_t gyro_int_conf = {
#if ESP_IDF_VERSION_MAJOR > 4
        .intr_type = GPIO_INTR_HIGH_LEVEL, // 楂樼數骞宠Е鍙?鏁版嵁灏辩华鏃朵繚鎸侀珮鐢靛钩)
#else
        .intr_type = GPIO_PIN_INTR_HILEVEL,
#endif
        .pin_bit_mask = (1ULL << BMI088_INT3_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE, // 浣胯兘涓嬫媺,淇濊瘉绌洪棽鏃朵负浣庣數骞?
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&gyro_int_conf);

    // 鍒涘缓浜屽€间俊鍙烽噺
    sensorsDataReady = xSemaphoreCreateBinaryStatic(&sensorsDataReadyBuffer);
    dataReady = xSemaphoreCreateBinaryStatic(&dataReadyBuffer);

    // 娉ㄥ唽GPIO涓柇鏈嶅姟
    gpio_set_intr_type(BMI088_INT1_PIN, GPIO_INTR_LOW_LEVEL);
    gpio_set_intr_type(BMI088_INT3_PIN, GPIO_INTR_HIGH_LEVEL);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    // 缁戝畾涓柇澶勭悊鍑芥暟
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

    // 娴嬭瘯BMI088
    if (!bmi088_test())
    {
        DEBUG_PRINT("BMI088 test [FAIL]\n");
        testStatus = false;
    }
    else
    {
        DEBUG_PRINT("BMI088 test [OK]\n");
    }

    // 娴嬭瘯MS5611
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

#ifdef GYRO_DYNAMIC_CALIBRATION_ENABLED
/**
 * 闄€铻轰华鍔ㄦ€佹牎鍑嗘洿鏂板嚱鏁?
 * 浣跨敤浣庨€氭护娉㈠櫒鍦ㄩ潤姝㈡椂鎸佺画鏇存柊闆剁偣鍋忓樊锛岄€傚簲娓╂紓鍙樺寲
 * 鏀硅繘锛氬熀浜庨檧铻轰华瀹為檯鏁版嵁妫€娴嬮潤姝㈢姸鎬侊紝鑰屼笉浠呬緷璧栭琛屾爣蹇?
 *
 * @param gx, gy, gz 闄€铻轰华鍘熷璇绘暟锛圠SB锛?
 * @param quadIsFlying 椋炶鐘舵€佹爣蹇?
 */
static void gyroDynamicCalibUpdate(int16_t gx, int16_t gy, int16_t gz, bool quadIsFlying)
{
    if (!gyroDynamicCalibEnabled)
    {
        return; // 鍔ㄦ€佹牎鍑嗘湭浣胯兘
    }

    // 鍩轰簬闄€铻轰华鏁版嵁鍒ゆ柇鏄惁闈欐
    // 璁＄畻闄€铻轰华璇绘暟涓庡綋鍓嶅亸缃殑鍋忓樊
    float dx = (float)(gx)-gyroBias.x;
    float dy = (float)(gy)-gyroBias.y;
    float dz = (float)(gz)-gyroBias.z;

    // 瑙掗€熷害骞呭€硷紙LSB鍗曚綅锛夛紝闈欐鏃跺簲鎺ヨ繎0
    float gyroMagnitude = sqrtf(dx * dx + dy * dy + dz * dz);

    // 闈欐闃堝€硷細绾?5掳/s 瀵瑰簲鐨?LSB 鍊硷紙淇濆畧璁剧疆閬垮厤璇垽锛?
    // 2000DPS閲忕▼涓嬶細1 LSB = 2000/32768 鈮?0.061掳/s
    // 5掳/s 鈮?82 LSB
    const float STATIC_THRESHOLD_LSB = 80.0f; // 绾?5掳/s

    static uint32_t staticCount = 0;
    const uint32_t STATIC_CONFIRM_COUNT = 800; // 绾?0.8绉?@ 1kHz锛堝厖鍒嗙‘璁ら潤姝級

    if (gyroMagnitude < STATIC_THRESHOLD_LSB)
    {
        staticCount++;
        if (staticCount > STATIC_CONFIRM_COUNT)
        {
            // 纭闈欐锛岃繘琛屽亸缃洿鏂?
            // 浣跨敤鏇村ぇ鐨?alpha 鍔犻€熸敹鏁涳紙闈欐鏃跺彲浠ユ洿婵€杩涳級
            float alpha = gyroDynamicCalibAlpha * 2.0f;
            gyroBias.x += alpha * dx;
            gyroBias.y += alpha * dy;
            gyroBias.z += alpha * dz;

            // 闃叉璁℃暟鍣ㄦ孩鍑?
            staticCount = STATIC_CONFIRM_COUNT + 1;
        }
    }
    else
    {
        // 杩愬姩涓紝閲嶇疆闈欐璁℃暟
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

    // 鏂瑰樊鍏紡: Var = [危x虏 - (危x)虏/N] / N
    // 娉ㄦ剰锛氬繀椤荤敤娴偣杩愮畻锛岄伩鍏嶆暣鏁伴櫎娉曟埅鏂鑷磋礋鏂瑰樊
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

static void sensorsScaleBaro(baro_t *baroScaled) // 姘斿帇璁¤鍙?
{
    float pressure, temperature, asl;
    ms5611GetData(&pressure, &temperature, &asl);
    // ms5611GetData 杩斿洖鐨勫崟浣嶏細
    // pressure: mbar (鍗?hPa)
    // temperature: 鎽勬皬搴?
    // asl: 绫?
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
