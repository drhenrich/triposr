#pragma once

#include "FreeRTOS.h"

typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);

void vTaskDelay(TickType_t);
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t, const char *, uint32_t,
                                   void *, unsigned, TaskHandle_t *, int core);
