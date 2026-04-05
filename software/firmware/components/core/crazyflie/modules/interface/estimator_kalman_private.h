#pragma once

#include "kalman_core.h"
#include "stabilizer_types.h"
#include "statsCnt.h"

extern kalmanCoreData_t coreData;

extern float accelAttitudeStdDevBase;
extern float accelAttitudeStdDevSlope;
extern bool quadIsFlying;

extern state_t taskEstimatorState;
extern Axis3f gyroSnapshot;
extern Axis3f accSnapshot;

extern statsCntRateLogger_t updateCounter;
extern statsCntRateLogger_t predictionCounter;
extern statsCntRateLogger_t baroUpdateCounter;
extern statsCntRateLogger_t finalizeCounter;
