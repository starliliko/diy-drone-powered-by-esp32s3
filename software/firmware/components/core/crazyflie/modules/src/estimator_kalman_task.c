#include <math.h>
#include <string.h>

#include "estimator_kalman.h"
#include "estimator_kalman_private.h"
#include "estimator_kalman_buffer.h"
#include "estimator_kalman_measurements.h"
#include "estimator_kalman_sync.h"
#include "kalman_supervisor.h"

#include "esp_log.h"
#include "stm32_legacy.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sensors.h"
#include "static_mem.h"
#include "system.h"
#include "param.h"
#include "physicalConstants.h"
#include "rateSupervisor.h"
#include "config.h"

#define WARNING_HOLD_BACK_TIME M2T(2000)

// Thrust is mapped so that 65536 corresponds to 60 grams.
#define CONTROL_TO_ACC (GRAVITY_MAGNITUDE * 60.0f / (CF_MASS * 1000.0f) / 65536.0f)

#define PREDICT_RATE RATE_500_HZ
#define BARO_RATE RATE_50_HZ
#define ACCEL_ATTITUDE_RATE RATE_25_HZ

#define IN_FLIGHT_THRUST_THRESHOLD (GRAVITY_MAGNITUDE * 0.1f)
#define IN_FLIGHT_TIME_THRESHOLD (500)

#ifdef KALMAN_USE_BARO_UPDATE
static const bool useBaroUpdate = true;
#else
static const bool useBaroUpdate = false;
#endif

static bool isInit = false;
static uint32_t lastFlightCmd;
static uint32_t warningBlockTime = 0;
static rateSupervisor_t rateSupervisorContext;

static void kalmanTask(void *parameters);
static bool predictStateForward(uint32_t osTick, float dt);
static bool updateQueuedMeasurements(void);
static bool updateBaroIfReady(uint32_t osTick, uint32_t *nextBaroUpdate);
static bool updateAccelAttitudeIfReady(uint32_t osTick, uint32_t *nextAccelAttitudeUpdate);
static void finalizeStateIfNeeded(bool doneUpdate, uint32_t osTick);
static void externalizeState(uint32_t osTick);

STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(kalmanTask, KALMAN_TASK_STACKSIZE);

void estimatorKalmanTaskInit(void)
{
  estimatorKalmanMeasurementsInit();
  estimatorKalmanSyncInit();

  STATIC_MEM_TASK_CREATE(kalmanTask, kalmanTask, KALMAN_TASK_NAME, NULL, KALMAN_TASK_PRI);

  isInit = true;
}

bool estimatorKalmanTaskTest(void)
{
  return isInit;
}

void estimatorKalman(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick)
{
  (void)tick;

  // This function is called from the stabilizer loop and must return quickly.
  estimatorKalmanSyncLock();

  bool accRead = sensorsReadAcc(&sensors->acc);
  bool gyroRead = sensorsReadGyro(&sensors->gyro);
  bool baroRead = false;

  if (useBaroUpdate)
  {
    baroRead = sensorsReadBaro(&sensors->baro);
  }

  estimatorKalmanBufferAccumulate(sensors, control, accRead, gyroRead, baroRead, useBaroUpdate);

  memcpy(&gyroSnapshot, &sensors->gyro, sizeof(gyroSnapshot));
  memcpy(&accSnapshot, &sensors->acc, sizeof(accSnapshot));
  memcpy(state, &taskEstimatorState, sizeof(state_t));

  estimatorKalmanSyncUnlock();
  estimatorKalmanSyncSignalRun();
}

static void kalmanTask(void *parameters)
{
  (void)parameters;

  systemWaitStart();

  uint32_t lastPrediction = xTaskGetTickCount();
  uint32_t nextPrediction = xTaskGetTickCount();
  uint32_t lastPNUpdate = xTaskGetTickCount();
  uint32_t nextBaroUpdate = xTaskGetTickCount();
  uint32_t nextAccelAttitudeUpdate = xTaskGetTickCount();

  rateSupervisorInit(&rateSupervisorContext, xTaskGetTickCount(), M2T(1000), 490, 510, 1);

  while (true)
  {
    estimatorKalmanSyncWaitForRunSignal();

    if (coreData.resetEstimation)
    {
      estimatorKalmanInit();
      paramSetInt(paramGetVarId("kalman", "resetEstimation"), 0);
    }

    bool doneUpdate = false;
    const uint32_t osTick = xTaskGetTickCount();

#ifdef KALMAN_DECOUPLE_XY
    kalmanCoreDecoupleXY(&coreData);
#endif

    if (osTick >= nextPrediction)
    {
      const float dt = T2S(osTick - lastPrediction);
      if (predictStateForward(osTick, dt))
      {
        lastPrediction = osTick;
        doneUpdate = true;
        STATS_CNT_RATE_EVENT(&predictionCounter);
      }

      nextPrediction = osTick + S2T(1.0f / PREDICT_RATE);
      (void)rateSupervisorValidate(&rateSupervisorContext, T2M(osTick));
    }

    {
      const float dt = T2S(osTick - lastPNUpdate);
      if (dt > 0.0f)
      {
        kalmanCoreAddProcessNoise(&coreData, dt);
        lastPNUpdate = osTick;
      }
    }

    if (updateBaroIfReady(osTick, &nextBaroUpdate))
    {
      doneUpdate = true;
    }

    if (updateAccelAttitudeIfReady(osTick, &nextAccelAttitudeUpdate))
    {
      doneUpdate = true;
    }

    if (updateQueuedMeasurements())
    {
      doneUpdate = true;
    }

    finalizeStateIfNeeded(doneUpdate, osTick);
    externalizeState(osTick);
    STATS_CNT_RATE_EVENT(&updateCounter);
  }
}

static bool predictStateForward(uint32_t osTick, float dt)
{
  Axis3f gyroAverage;
  Axis3f accAverage;
  float thrustAverage;

  estimatorKalmanSyncLock();
  const bool hasPredictionInput = estimatorKalmanBufferGetPredictionInput(&accAverage, &gyroAverage, &thrustAverage, CONTROL_TO_ACC);
  estimatorKalmanSyncUnlock();

  if (!hasPredictionInput)
  {
    return false;
  }

  if (thrustAverage > IN_FLIGHT_THRUST_THRESHOLD)
  {
    lastFlightCmd = osTick;
  }

  quadIsFlying = (osTick - lastFlightCmd) < IN_FLIGHT_TIME_THRESHOLD;
  kalmanCorePredict(&coreData, thrustAverage, &accAverage, &gyroAverage, dt, quadIsFlying);
  return true;
}

static bool updateBaroIfReady(uint32_t osTick, uint32_t *nextBaroUpdate)
{
  if (!useBaroUpdate || osTick <= *nextBaroUpdate)
  {
    return false;
  }

  float baroAslAverage;
  estimatorKalmanSyncLock();
  const bool hasBaroAverage = estimatorKalmanBufferGetBaroAverage(&baroAslAverage);
  estimatorKalmanSyncUnlock();

  if (!hasBaroAverage)
  {
    return false;
  }

  kalmanCoreUpdateWithBaro(&coreData, baroAslAverage, quadIsFlying);
  *nextBaroUpdate = osTick + S2T(1.0f / BARO_RATE);
  STATS_CNT_RATE_EVENT(&baroUpdateCounter);
  return true;
}

static bool updateAccelAttitudeIfReady(uint32_t osTick, uint32_t *nextAccelAttitudeUpdate)
{
  if (osTick <= *nextAccelAttitudeUpdate)
  {
    return false;
  }

  Axis3f accAvg;
  estimatorKalmanSyncLock();
  const bool hasAccelAttAverage = estimatorKalmanBufferGetAccelAttitudeAverage(&accAvg);
  estimatorKalmanSyncUnlock();

  if (!hasAccelAttAverage)
  {
    return false;
  }

  const float accMagAvg = sqrtf(accAvg.x * accAvg.x + accAvg.y * accAvg.y + accAvg.z * accAvg.z);
  const float accelDeviation = fabsf(accMagAvg - 1.0f);
  const float accelAttitudeStdDev = accelAttitudeStdDevBase + accelDeviation * accelAttitudeStdDevSlope;

  kalmanCoreUpdateWithAccel(&coreData, &accAvg, accelAttitudeStdDev);
  *nextAccelAttitudeUpdate = osTick + S2T(1.0f / ACCEL_ATTITUDE_RATE);
  return true;
}

static bool updateQueuedMeasurements(void)
{
  bool doneUpdate = false;

  tofMeasurement_t tof;
  while (estimatorKalmanPopTOF(&tof))
  {
    kalmanCoreUpdateWithTof(&coreData, &tof);
    doneUpdate = true;
  }

  yawErrorMeasurement_t yawError;
  while (estimatorKalmanPopYawError(&yawError))
  {
    kalmanCoreUpdateWithYawError(&coreData, &yawError);
    doneUpdate = true;
  }

  heightMeasurement_t height;
  while (estimatorKalmanPopAbsoluteHeight(&height))
  {
    kalmanCoreUpdateWithAbsoluteHeight(&coreData, &height);
    doneUpdate = true;
  }

  distanceMeasurement_t dist;
  while (estimatorKalmanPopDistanceMeasurement(&dist))
  {
    kalmanCoreUpdateWithDistance(&coreData, &dist);
    doneUpdate = true;
  }

  positionMeasurement_t pos;
  while (estimatorKalmanPopPositionMeasurement(&pos))
  {
    kalmanCoreUpdateWithPosition(&coreData, &pos);
    doneUpdate = true;
  }

  poseMeasurement_t pose;
  while (estimatorKalmanPopPoseMeasurement(&pose))
  {
    kalmanCoreUpdateWithPose(&coreData, &pose);
    doneUpdate = true;
  }

  tdoaMeasurement_t tdoa;
  while (estimatorKalmanPopTDOA(&tdoa))
  {
    kalmanCoreUpdateWithTDOA(&coreData, &tdoa);
    doneUpdate = true;
  }

  velocityMeasurement_t vel;
  while (estimatorKalmanPopVelocity(&vel))
  {
    kalmanCoreUpdateWithVelocity(&coreData, &vel);
    doneUpdate = true;
  }

  return doneUpdate;
}

static void finalizeStateIfNeeded(bool doneUpdate, uint32_t osTick)
{
  if (!doneUpdate)
  {
    return;
  }

  kalmanCoreFinalize(&coreData, osTick);
  STATS_CNT_RATE_EVENT(&finalizeCounter);

  if (!kalmanSupervisorIsStateWithinBounds(&coreData))
  {
    coreData.resetEstimation = true;

    if (osTick > warningBlockTime)
    {
      warningBlockTime = osTick + WARNING_HOLD_BACK_TIME;
      ESP_LOGI("ESTKALMAN", "State out of bounds, resetting");
    }
  }
}

static void externalizeState(uint32_t osTick)
{
  estimatorKalmanSyncLock();
  kalmanCoreExternalizeState(&coreData, &taskEstimatorState, &accSnapshot, osTick);
  estimatorKalmanSyncUnlock();
}
