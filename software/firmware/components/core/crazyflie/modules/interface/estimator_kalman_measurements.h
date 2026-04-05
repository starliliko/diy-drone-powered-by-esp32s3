#pragma once

#include <stdbool.h>

#include "estimator_kalman.h"
#include "statsCnt.h"

void estimatorKalmanMeasurementsInit(void);
void estimatorKalmanMeasurementsReset(void);

bool estimatorKalmanPopDistanceMeasurement(distanceMeasurement_t *dist);
bool estimatorKalmanPopPositionMeasurement(positionMeasurement_t *pos);
bool estimatorKalmanPopPoseMeasurement(poseMeasurement_t *pose);
bool estimatorKalmanPopTDOA(tdoaMeasurement_t *uwb);
bool estimatorKalmanPopVelocity(velocityMeasurement_t *vel);
bool estimatorKalmanPopTOF(tofMeasurement_t *tof);
bool estimatorKalmanPopAbsoluteHeight(heightMeasurement_t *height);
bool estimatorKalmanPopYawError(yawErrorMeasurement_t *error);

extern statsCntRateLogger_t measurementAppendedCounter;
extern statsCntRateLogger_t measurementNotAppendedCounter;
