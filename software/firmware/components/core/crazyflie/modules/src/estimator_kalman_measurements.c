#include "estimator_kalman_measurements.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"

#include "cfassert.h"
#include "statsCnt.h"

// Distance-to-point measurements
static xQueueHandle distDataQueue;
STATIC_MEM_QUEUE_ALLOC(distDataQueue, 10, sizeof(distanceMeasurement_t));

// Direct measurements of Crazyflie position
static xQueueHandle posDataQueue;
STATIC_MEM_QUEUE_ALLOC(posDataQueue, 10, sizeof(positionMeasurement_t));

// Direct measurements of Crazyflie pose
static xQueueHandle poseDataQueue;
STATIC_MEM_QUEUE_ALLOC(poseDataQueue, 10, sizeof(poseMeasurement_t));

// Measurements of a UWB Tx/Rx
static xQueueHandle tdoaDataQueue;
STATIC_MEM_QUEUE_ALLOC(tdoaDataQueue, 10, sizeof(tdoaMeasurement_t));

// Measurements of velocity from MTF01 optical flow
static xQueueHandle velocityDataQueue;
STATIC_MEM_QUEUE_ALLOC(velocityDataQueue, 10, sizeof(velocityMeasurement_t));

// Measurements of TOF from laser sensor
static xQueueHandle tofDataQueue;
STATIC_MEM_QUEUE_ALLOC(tofDataQueue, 10, sizeof(tofMeasurement_t));

// Absolute height measurement along the room Z
static xQueueHandle heightDataQueue;
STATIC_MEM_QUEUE_ALLOC(heightDataQueue, 10, sizeof(heightMeasurement_t));

static xQueueHandle yawErrorDataQueue;
STATIC_MEM_QUEUE_ALLOC(yawErrorDataQueue, 10, sizeof(yawErrorMeasurement_t));

// Statistics for external measurements fed into the estimator.
statsCntRateLogger_t measurementAppendedCounter = {
    .logByFunction = {.data = &measurementAppendedCounter, .aquireFloat = statsCntRateLogHandler},
    .rateCounter = {.intervalMs = 1000, .count = 0, .latestCount = 0, .latestAveragingMs = 0, .latestRate = 0}};
statsCntRateLogger_t measurementNotAppendedCounter = {
    .logByFunction = {.data = &measurementNotAppendedCounter, .aquireFloat = statsCntRateLogHandler},
    .rateCounter = {.intervalMs = 1000, .count = 0, .latestCount = 0, .latestAveragingMs = 0, .latestRate = 0}};

static bool appendMeasurement(xQueueHandle queue, void *measurement)
{
  portBASE_TYPE result;
  bool isInInterrupt = 0; // TODO:= (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

  if (isInInterrupt)
  {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    result = xQueueSendFromISR(queue, measurement, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
      portYIELD();
    }
  }
  else
  {
    result = xQueueSend(queue, measurement, 0);
  }

  if (result == pdTRUE)
  {
    STATS_CNT_RATE_EVENT(&measurementAppendedCounter);
    return true;
  }
  else
  {
    STATS_CNT_RATE_EVENT(&measurementNotAppendedCounter);
    return false;
  }
}

void estimatorKalmanMeasurementsInit(void)
{
  distDataQueue = STATIC_MEM_QUEUE_CREATE(distDataQueue);
  posDataQueue = STATIC_MEM_QUEUE_CREATE(posDataQueue);
  poseDataQueue = STATIC_MEM_QUEUE_CREATE(poseDataQueue);
  tdoaDataQueue = STATIC_MEM_QUEUE_CREATE(tdoaDataQueue);
  velocityDataQueue = STATIC_MEM_QUEUE_CREATE(velocityDataQueue);
  tofDataQueue = STATIC_MEM_QUEUE_CREATE(tofDataQueue);
  heightDataQueue = STATIC_MEM_QUEUE_CREATE(heightDataQueue);
  yawErrorDataQueue = STATIC_MEM_QUEUE_CREATE(yawErrorDataQueue);
}

void estimatorKalmanMeasurementsReset(void)
{
  // Preserve existing behavior: only queues reset by estimatorKalmanInit() are cleared.
  xQueueReset(distDataQueue);
  xQueueReset(posDataQueue);
  xQueueReset(poseDataQueue);
  xQueueReset(tdoaDataQueue);
  xQueueReset(velocityDataQueue);
  xQueueReset(tofDataQueue);
}

bool estimatorKalmanPopDistanceMeasurement(distanceMeasurement_t *dist)
{
  return (pdTRUE == xQueueReceive(distDataQueue, dist, 0));
}

bool estimatorKalmanPopPositionMeasurement(positionMeasurement_t *pos)
{
  return (pdTRUE == xQueueReceive(posDataQueue, pos, 0));
}

bool estimatorKalmanPopPoseMeasurement(poseMeasurement_t *pose)
{
  return (pdTRUE == xQueueReceive(poseDataQueue, pose, 0));
}

bool estimatorKalmanPopTDOA(tdoaMeasurement_t *uwb)
{
  return (pdTRUE == xQueueReceive(tdoaDataQueue, uwb, 0));
}

bool estimatorKalmanPopVelocity(velocityMeasurement_t *vel)
{
  return (pdTRUE == xQueueReceive(velocityDataQueue, vel, 0));
}

bool estimatorKalmanPopTOF(tofMeasurement_t *tof)
{
  return (pdTRUE == xQueueReceive(tofDataQueue, tof, 0));
}

bool estimatorKalmanPopAbsoluteHeight(heightMeasurement_t *height)
{
  return (pdTRUE == xQueueReceive(heightDataQueue, height, 0));
}

bool estimatorKalmanPopYawError(yawErrorMeasurement_t *error)
{
  return (pdTRUE == xQueueReceive(yawErrorDataQueue, error, 0));
}

bool estimatorKalmanEnqueueTDOA(const tdoaMeasurement_t *uwb)
{
  ASSERT(estimatorKalmanTaskTest());
  return appendMeasurement(tdoaDataQueue, (void *)uwb);
}

bool estimatorKalmanEnqueuePosition(const positionMeasurement_t *pos)
{
  ASSERT(estimatorKalmanTaskTest());
  return appendMeasurement(posDataQueue, (void *)pos);
}

bool estimatorKalmanEnqueuePose(const poseMeasurement_t *pose)
{
  ASSERT(estimatorKalmanTaskTest());
  return appendMeasurement(poseDataQueue, (void *)pose);
}

bool estimatorKalmanEnqueueDistance(const distanceMeasurement_t *dist)
{
  ASSERT(estimatorKalmanTaskTest());
  return appendMeasurement(distDataQueue, (void *)dist);
}

bool estimatorKalmanEnqueueVelocity(const velocityMeasurement_t *vel)
{
  ASSERT(estimatorKalmanTaskTest());
  return appendMeasurement(velocityDataQueue, (void *)vel);
}

bool estimatorKalmanEnqueueTOF(const tofMeasurement_t *tof)
{
  ASSERT(estimatorKalmanTaskTest());
  return appendMeasurement(tofDataQueue, (void *)tof);
}

bool estimatorKalmanEnqueueAbsoluteHeight(const heightMeasurement_t *height)
{
  ASSERT(estimatorKalmanTaskTest());
  return appendMeasurement(heightDataQueue, (void *)height);
}

bool estimatorKalmanEnqueueYawError(const yawErrorMeasurement_t *error)
{
  ASSERT(estimatorKalmanTaskTest());
  return appendMeasurement(yawErrorDataQueue, (void *)error);
}
