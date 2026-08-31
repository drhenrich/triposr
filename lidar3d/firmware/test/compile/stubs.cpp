// Rumpfdefinitionen zu den Attrappen. Sie werden gebraucht, damit der
// Uebersetzungstest bis zum Binden kommt - aufgerufen wird hier nichts.
#include "stubs/Arduino.h"
#include "stubs/WiFi.h"
#include "stubs/driver/uart.h"
#include "stubs/esp_timer.h"
#include "stubs/freertos/queue.h"
#include "stubs/freertos/task.h"

SerialStub Serial;
WiFiClass WiFi;

int64_t esp_timer_get_time() { return 0; }

QueueHandle_t xQueueCreate(unsigned, unsigned) { return nullptr; }
BaseType_t xQueueSend(QueueHandle_t, const void *, TickType_t) { return pdTRUE; }
BaseType_t xQueueReceive(QueueHandle_t, void *, TickType_t) { return pdFALSE; }
void xQueueReset(QueueHandle_t) {}
void vTaskDelay(TickType_t) {}
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t, const char *, uint32_t,
                                   void *, unsigned, TaskHandle_t *, int) {
  return pdPASS;
}

esp_err_t uart_driver_install(uart_port_t, int, int, int, void *, int) { return ESP_OK; }
esp_err_t uart_param_config(uart_port_t, const uart_config_t *) { return ESP_OK; }
esp_err_t uart_set_pin(uart_port_t, int, int, int, int) { return ESP_OK; }
esp_err_t uart_set_mode(uart_port_t, uart_mode_t) { return ESP_OK; }
esp_err_t uart_flush_input(uart_port_t) { return ESP_OK; }
esp_err_t uart_wait_tx_done(uart_port_t, TickType_t) { return ESP_OK; }
int uart_write_bytes(uart_port_t, const char *, size_t n) { return static_cast<int>(n); }
int uart_read_bytes(uart_port_t, uint8_t *, uint32_t, TickType_t) { return 0; }

// setup()/loop() kommen aus main.cpp - hier nur ein Einstiegspunkt, damit
// gebunden werden kann.
void setup();
void loop();
int main() { return 0; }
