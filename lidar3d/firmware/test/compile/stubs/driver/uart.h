// Attrappe des IDF-UART-Treibers - siehe ../Arduino.h.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "../esp_err.h"
#include "../freertos/FreeRTOS.h"

typedef enum { UART_NUM_0 = 0, UART_NUM_1 = 1, UART_NUM_2 = 2 } uart_port_t;
typedef enum { UART_DATA_8_BITS = 3 } uart_word_length_t;
typedef enum { UART_PARITY_DISABLE = 0 } uart_parity_t;
typedef enum { UART_STOP_BITS_1 = 1 } uart_stop_bits_t;
typedef enum { UART_HW_FLOWCTRL_DISABLE = 0 } uart_hw_flowcontrol_t;
typedef enum { UART_SCLK_DEFAULT = 0 } uart_sclk_t;
typedef enum { UART_MODE_UART = 0, UART_MODE_RS485_HALF_DUPLEX = 1 } uart_mode_t;

#define UART_PIN_NO_CHANGE (-1)

typedef struct {
  int baud_rate;
  uart_word_length_t data_bits;
  uart_parity_t parity;
  uart_stop_bits_t stop_bits;
  uart_hw_flowcontrol_t flow_ctrl;
  uint8_t rx_flow_ctrl_thresh;
  uart_sclk_t source_clk;
} uart_config_t;

esp_err_t uart_driver_install(uart_port_t, int rxBuf, int txBuf, int queueSize,
                              void *queue, int flags);
esp_err_t uart_param_config(uart_port_t, const uart_config_t *);
esp_err_t uart_set_pin(uart_port_t, int tx, int rx, int rts, int cts);
esp_err_t uart_set_mode(uart_port_t, uart_mode_t);
esp_err_t uart_flush_input(uart_port_t);
esp_err_t uart_wait_tx_done(uart_port_t, TickType_t);
int uart_write_bytes(uart_port_t, const char *, size_t);
int uart_read_bytes(uart_port_t, uint8_t *, uint32_t, TickType_t);
