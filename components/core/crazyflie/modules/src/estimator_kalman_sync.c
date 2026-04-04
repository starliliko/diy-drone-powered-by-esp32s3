#include "estimator_kalman_sync.h"

#include "FreeRTOS.h"
#include "semphr.h"

// Keep RTOS synchronization details out of estimator_kalman.c so the Kalman
// flow reads as predict/update logic instead of task plumbing.
static SemaphoreHandle_t runTaskSemaphore;
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;

void estimatorKalmanSyncInit(void)
{
  vSemaphoreCreateBinary(runTaskSemaphore);
  dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);
}

void estimatorKalmanSyncLock(void)
{
  xSemaphoreTake(dataMutex, portMAX_DELAY);
}

void estimatorKalmanSyncUnlock(void)
{
  xSemaphoreGive(dataMutex);
}

void estimatorKalmanSyncWaitForRunSignal(void)
{
  xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);
}

void estimatorKalmanSyncSignalRun(void)
{
  xSemaphoreGive(runTaskSemaphore);
}
