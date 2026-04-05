#include "estimator_kalman_buffer.h"

#include "cf_math.h"
#include "physicalConstants.h"

static Axis3f accAccumulator;
static float thrustAccumulator;
static Axis3f gyroAccumulator;
static float baroAslAccumulator;
static uint32_t accAccumulatorCount;
static uint32_t thrustAccumulatorCount;
static uint32_t gyroAccumulatorCount;
static uint32_t baroAccumulatorCount;
static Axis3f accAttAccumulator;
static uint32_t accAttAccumulatorCount;

void estimatorKalmanBufferReset(void)
{
  accAccumulator = (Axis3f){.axis = {0}};
  gyroAccumulator = (Axis3f){.axis = {0}};
  thrustAccumulator = 0.0f;
  baroAslAccumulator = 0.0f;

  accAccumulatorCount = 0;
  gyroAccumulatorCount = 0;
  thrustAccumulatorCount = 0;
  baroAccumulatorCount = 0;

  accAttAccumulator = (Axis3f){.axis = {0}};
  accAttAccumulatorCount = 0;
}

void estimatorKalmanBufferAccumulate(const sensorData_t *sensors,
                                     const control_t *control,
                                     bool accRead,
                                     bool gyroRead,
                                     bool baroRead,
                                     bool useBaroUpdate)
{
  if (accRead)
  {
    accAccumulator.x += sensors->acc.x;
    accAccumulator.y += sensors->acc.y;
    accAccumulator.z += sensors->acc.z;
    accAccumulatorCount++;

    accAttAccumulator.x += sensors->acc.x;
    accAttAccumulator.y += sensors->acc.y;
    accAttAccumulator.z += sensors->acc.z;
    accAttAccumulatorCount++;
  }

  if (gyroRead)
  {
    gyroAccumulator.x += sensors->gyro.x;
    gyroAccumulator.y += sensors->gyro.y;
    gyroAccumulator.z += sensors->gyro.z;
    gyroAccumulatorCount++;
  }

  thrustAccumulator += control->thrust;
  thrustAccumulatorCount++;

  if (useBaroUpdate && baroRead)
  {
    baroAslAccumulator += sensors->baro.asl;
    baroAccumulatorCount++;
  }
}

bool estimatorKalmanBufferGetPredictionInput(Axis3f *accAverage,
                                             Axis3f *gyroAverage,
                                             float *thrustAverage,
                                             float controlToAcc)
{
  if (gyroAccumulatorCount == 0 || accAccumulatorCount == 0 || thrustAccumulatorCount == 0)
  {
    return false;
  }

  gyroAverage->x = gyroAccumulator.x * DEG_TO_RAD / gyroAccumulatorCount;
  gyroAverage->y = gyroAccumulator.y * DEG_TO_RAD / gyroAccumulatorCount;
  gyroAverage->z = gyroAccumulator.z * DEG_TO_RAD / gyroAccumulatorCount;

  accAverage->x = accAccumulator.x * GRAVITY_MAGNITUDE / accAccumulatorCount;
  accAverage->y = accAccumulator.y * GRAVITY_MAGNITUDE / accAccumulatorCount;
  accAverage->z = accAccumulator.z * GRAVITY_MAGNITUDE / accAccumulatorCount;

  *thrustAverage = thrustAccumulator * controlToAcc / thrustAccumulatorCount;

  accAccumulator = (Axis3f){.axis = {0}};
  accAccumulatorCount = 0;
  gyroAccumulator = (Axis3f){.axis = {0}};
  gyroAccumulatorCount = 0;
  thrustAccumulator = 0.0f;
  thrustAccumulatorCount = 0;

  return true;
}

bool estimatorKalmanBufferGetBaroAverage(float *baroAslAverage)
{
  if (baroAccumulatorCount == 0)
  {
    return false;
  }

  *baroAslAverage = baroAslAccumulator / baroAccumulatorCount;
  baroAslAccumulator = 0.0f;
  baroAccumulatorCount = 0;
  return true;
}

bool estimatorKalmanBufferGetAccelAttitudeAverage(Axis3f *accAverage)
{
  if (accAttAccumulatorCount == 0)
  {
    return false;
  }

  accAverage->x = accAttAccumulator.x / accAttAccumulatorCount;
  accAverage->y = accAttAccumulator.y / accAttAccumulatorCount;
  accAverage->z = accAttAccumulator.z / accAttAccumulatorCount;

  accAttAccumulator = (Axis3f){.axis = {0}};
  accAttAccumulatorCount = 0;
  return true;
}
