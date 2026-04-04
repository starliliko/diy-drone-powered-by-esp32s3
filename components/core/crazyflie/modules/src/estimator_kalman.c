/**
 * Public Kalman estimator state and API glue.
 *
 * The heavy RTOS/task orchestration lives in estimator_kalman_task.c.
 * This file now focuses on:
 * - shared estimator state exposed to other modules
 * - public helper APIs
 * - log and param registration
 */

#include <string.h>

#include "static_mem.h"

#include "estimator_kalman.h"
#include "estimator_kalman_private.h"
#include "estimator_kalman_buffer.h"
#include "estimator_kalman_measurements.h"
#include "estimator_kalman_sync.h"
#include "kalman_core.h"

#include "log.h"
#include "param.h"
#include "statsCnt.h"

#define ONE_SECOND 1000

NO_DMA_CCM_SAFE_ZERO_INIT kalmanCoreData_t coreData;

float accelAttitudeStdDevBase = 0.15f;
float accelAttitudeStdDevSlope = 5.0f;
bool quadIsFlying = false;

state_t taskEstimatorState;
Axis3f gyroSnapshot;
Axis3f accSnapshot;

STATS_CNT_RATE_DEFINE(updateCounter, ONE_SECOND);
STATS_CNT_RATE_DEFINE(predictionCounter, ONE_SECOND);
STATS_CNT_RATE_DEFINE(baroUpdateCounter, ONE_SECOND);
STATS_CNT_RATE_DEFINE(finalizeCounter, ONE_SECOND);

void estimatorKalmanInit(void)
{
  estimatorKalmanMeasurementsReset();

  estimatorKalmanSyncLock();
  estimatorKalmanBufferReset();
  kalmanCoreInit(&coreData);
  estimatorKalmanSyncUnlock();
}

bool estimatorKalmanIsBaroCalibrated(void)
{
  return kalmanCoreIsBaroCalibrated();
}

bool estimatorKalmanTest(void)
{
  return estimatorKalmanTaskTest();
}

void estimatorKalmanGetEstimatedPos(point_t *pos)
{
  estimatorKalmanSyncLock();
  pos->x = coreData.S[KC_STATE_X];
  pos->y = coreData.S[KC_STATE_Y];
  pos->z = coreData.S[KC_STATE_Z];
  estimatorKalmanSyncUnlock();
}

void estimatorKalmanGetEstimatedRot(float *rotationMatrix)
{
  estimatorKalmanSyncLock();
  memcpy(rotationMatrix, coreData.R, 9 * sizeof(float));
  estimatorKalmanSyncUnlock();
}

LOG_GROUP_START(kalman_states)
LOG_ADD(LOG_FLOAT, ox, &coreData.S[KC_STATE_X])
LOG_ADD(LOG_FLOAT, oy, &coreData.S[KC_STATE_Y])
LOG_ADD(LOG_FLOAT, vx, &coreData.S[KC_STATE_PX])
LOG_ADD(LOG_FLOAT, vy, &coreData.S[KC_STATE_PY])
LOG_GROUP_STOP(kalman_states)

LOG_GROUP_START(kalman)
LOG_ADD(LOG_UINT8, inFlight, &quadIsFlying)
LOG_ADD(LOG_FLOAT, stateX, &coreData.S[KC_STATE_X])
LOG_ADD(LOG_FLOAT, stateY, &coreData.S[KC_STATE_Y])
LOG_ADD(LOG_FLOAT, stateZ, &coreData.S[KC_STATE_Z])
LOG_ADD(LOG_FLOAT, statePX, &coreData.S[KC_STATE_PX])
LOG_ADD(LOG_FLOAT, statePY, &coreData.S[KC_STATE_PY])
LOG_ADD(LOG_FLOAT, statePZ, &coreData.S[KC_STATE_PZ])
LOG_ADD(LOG_FLOAT, stateD0, &coreData.S[KC_STATE_D0])
LOG_ADD(LOG_FLOAT, stateD1, &coreData.S[KC_STATE_D1])
LOG_ADD(LOG_FLOAT, stateD2, &coreData.S[KC_STATE_D2])
LOG_ADD(LOG_FLOAT, varX, &coreData.P[KC_STATE_X][KC_STATE_X])
LOG_ADD(LOG_FLOAT, varY, &coreData.P[KC_STATE_Y][KC_STATE_Y])
LOG_ADD(LOG_FLOAT, varZ, &coreData.P[KC_STATE_Z][KC_STATE_Z])
LOG_ADD(LOG_FLOAT, varPX, &coreData.P[KC_STATE_PX][KC_STATE_PX])
LOG_ADD(LOG_FLOAT, varPY, &coreData.P[KC_STATE_PY][KC_STATE_PY])
LOG_ADD(LOG_FLOAT, varPZ, &coreData.P[KC_STATE_PZ][KC_STATE_PZ])
LOG_ADD(LOG_FLOAT, varD0, &coreData.P[KC_STATE_D0][KC_STATE_D0])
LOG_ADD(LOG_FLOAT, varD1, &coreData.P[KC_STATE_D1][KC_STATE_D1])
LOG_ADD(LOG_FLOAT, varD2, &coreData.P[KC_STATE_D2][KC_STATE_D2])
LOG_ADD(LOG_FLOAT, q0, &coreData.q[0])
LOG_ADD(LOG_FLOAT, q1, &coreData.q[1])
LOG_ADD(LOG_FLOAT, q2, &coreData.q[2])
LOG_ADD(LOG_FLOAT, q3, &coreData.q[3])
STATS_CNT_RATE_LOG_ADD(rtUpdate, &updateCounter)
STATS_CNT_RATE_LOG_ADD(rtPred, &predictionCounter)
STATS_CNT_RATE_LOG_ADD(rtBaro, &baroUpdateCounter)
STATS_CNT_RATE_LOG_ADD(rtFinal, &finalizeCounter)
STATS_CNT_RATE_LOG_ADD(rtApnd, &measurementAppendedCounter)
STATS_CNT_RATE_LOG_ADD(rtRej, &measurementNotAppendedCounter)
LOG_GROUP_STOP(kalman)

PARAM_GROUP_START(kalman)
PARAM_ADD(PARAM_UINT8, resetEstimation, &coreData.resetEstimation)
PARAM_ADD(PARAM_UINT8, quadIsFlying, &quadIsFlying)
PARAM_ADD(PARAM_FLOAT, accAttStd0, &accelAttitudeStdDevBase)
PARAM_ADD(PARAM_FLOAT, accAttStdK, &accelAttitudeStdDevSlope)
PARAM_GROUP_STOP(kalman)
