#pragma once

#include <stdbool.h>

#include "stabilizer_types.h"

void estimatorKalmanBufferReset(void);
void estimatorKalmanBufferAccumulate(const sensorData_t *sensors,
                                     const control_t *control,
                                     bool accRead,
                                     bool gyroRead,
                                     bool baroRead,
                                     bool useBaroUpdate);

bool estimatorKalmanBufferGetPredictionInput(Axis3f *accAverage,
                                             Axis3f *gyroAverage,
                                             float *thrustAverage,
                                             float controlToAcc);

bool estimatorKalmanBufferGetBaroAverage(float *baroAslAverage);
bool estimatorKalmanBufferGetAccelAttitudeAverage(Axis3f *accAverage);
