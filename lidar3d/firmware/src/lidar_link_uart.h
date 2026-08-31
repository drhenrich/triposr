// LidarLink ueber die UART des ESP32 - der Weg ohne USB-Adapter.
#pragma once

#include <driver/uart.h>
#include <freertos/FreeRTOS.h>

#include "lidar_link.h"
#include "uart_compat.h"

namespace nwl {

class UartLidarLink : public LidarLink {
 public:
  bool begin(uart_port_t port, int rxPin, int txPin, int baudrate, int rxBuffer) {
    port_ = port;

    uart_config_t cfg = {};
    cfg.baud_rate = baudrate;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = kUartSourceClk;

    if (uart_driver_install(port_, rxBuffer, 0, 0, nullptr, 0) != ESP_OK) return false;
    if (uart_param_config(port_, &cfg) != ESP_OK) return false;
    if (uart_set_pin(port_, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) !=
        ESP_OK) {
      return false;
    }
    uart_flush_input(port_);
    ready_ = true;
    return true;
  }

  bool write(const uint8_t *data, size_t len) override {
    return uart_write_bytes(port_, reinterpret_cast<const char *>(data), len) ==
           static_cast<int>(len);
  }

  size_t read(uint8_t *out, size_t maxLen, uint32_t timeoutMs) override {
    int got = uart_read_bytes(port_, out, static_cast<uint32_t>(maxLen),
                              pdMS_TO_TICKS(timeoutMs));
    return got > 0 ? static_cast<size_t>(got) : 0;
  }

  void flushInput() override { uart_flush_input(port_); }

  bool connected() const override { return ready_; }

 private:
  uart_port_t port_ = UART_NUM_1;
  bool ready_ = false;
};

}  // namespace nwl
