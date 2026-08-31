// Attrappe von FreeRTOS - siehe ../Arduino.h.
#pragma once

#include <stdint.h>

typedef uint32_t TickType_t;
typedef int BaseType_t;

#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define pdMS_TO_TICKS(ms) (static_cast<TickType_t>(ms))
#define portMAX_DELAY (static_cast<TickType_t>(0xFFFFFFFF))
