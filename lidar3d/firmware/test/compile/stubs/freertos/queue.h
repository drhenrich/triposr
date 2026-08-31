#pragma once

#include "FreeRTOS.h"

struct QueueDefinition;
typedef struct QueueDefinition *QueueHandle_t;

QueueHandle_t xQueueCreate(unsigned length, unsigned itemSize);
BaseType_t xQueueSend(QueueHandle_t, const void *, TickType_t);
BaseType_t xQueueReceive(QueueHandle_t, void *, TickType_t);
void xQueueReset(QueueHandle_t);
