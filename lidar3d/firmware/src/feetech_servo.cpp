#include "feetech_servo.h"

#include "uart_compat.h"

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace nwl {

using namespace feetech;

bool FeetechServo::begin(uart_port_t port, int rxPin, int txPin, int dirPin,
                         int baudrate, uint8_t id) {
  port_ = port;
  id_ = id;

  uart_config_t cfg = {};
  cfg.baud_rate = baudrate;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = kUartSourceClk;

  if (uart_driver_install(port_, 512, 512, 0, nullptr, 0) != ESP_OK) return false;
  if (uart_param_config(port_, &cfg) != ESP_OK) return false;

  int rts = (dirPin >= 0) ? dirPin : UART_PIN_NO_CHANGE;
  if (uart_set_pin(port_, txPin, rxPin, rts, UART_PIN_NO_CHANGE) != ESP_OK) {
    return false;
  }
  if (dirPin >= 0) {
    // Der Treiber schaltet DE/RE selbst und blendet das eigene Echo aus.
    if (uart_set_mode(port_, UART_MODE_RS485_HALF_DUPLEX) != ESP_OK) return false;
  }

  uart_flush_input(port_);
  parser_.reset();
  return ping();
}

bool FeetechServo::writeOnly(const uint8_t *request, size_t length) {
  return uart_write_bytes(port_, reinterpret_cast<const char *>(request), length) ==
         static_cast<int>(length);
}

bool FeetechServo::transact(const uint8_t *request, size_t length,
                            StatusPacket &response, uint8_t expectParams,
                            uint32_t timeoutMs) {
  uart_flush_input(port_);
  parser_.reset();
  if (!writeOnly(request, length)) return false;
  uart_wait_tx_done(port_, pdMS_TO_TICKS(20));

  // Das eigene Echo zuerst wegwerfen - Begruendung in feetech_bus.h.
  EchoFilter echo;
  echo.reset(request, length);
  uint8_t forward[kMaxPacketSize + 1];

  int64_t deadline = esp_timer_get_time() + static_cast<int64_t>(timeoutMs) * 1000;
  while (esp_timer_get_time() < deadline) {
    uint8_t byte;
    int got = uart_read_bytes(port_, &byte, 1, pdMS_TO_TICKS(2));
    if (got != 1) continue;

    size_t n = echo.push(byte, forward);
    for (size_t i = 0; i < n; ++i) {
      if (!parser_.push(forward[i], response)) continue;
      // Am Bus koennen mehrere Servos haengen; nur die eigene ID zaehlt.
      if (response.id != id_) continue;
      if (response.paramCount < expectParams) continue;
      return true;
    }
    continue;
  }
  ++timeouts_;
  return false;
}

bool FeetechServo::ping() {
  uint8_t request[kMaxPacketSize];
  size_t n = buildPing(request, id_);
  StatusPacket response;
  return transact(request, n, response, 0, 50);
}

bool FeetechServo::readModel(uint16_t &model) {
  uint8_t request[kMaxPacketSize];
  size_t n = buildRead(request, id_, kRegModelL, 2);
  StatusPacket response;
  if (!transact(request, n, response, 2, 50)) return false;
  model = get16(response.params);
  return true;
}

bool FeetechServo::setMode(uint8_t mode) {
  uint8_t request[kMaxPacketSize];
  size_t n = buildWrite8(request, id_, kRegMode, mode);
  StatusPacket response;
  return transact(request, n, response, 0, 50);
}

bool FeetechServo::setTorque(bool enabled) {
  uint8_t request[kMaxPacketSize];
  size_t n = buildWrite8(request, id_, kRegTorqueEnable, enabled ? 1 : 0);
  StatusPacket response;
  return transact(request, n, response, 0, 50);
}

bool FeetechServo::setAngleLimits(uint16_t minCounts, uint16_t maxCounts) {
  // MIN_ANGLE_LIMIT (9/10) und MAX_ANGLE_LIMIT (11/12) liegen zusammenhaengend.
  uint8_t data[4];
  put16(data, minCounts);
  put16(data + 2, maxCounts);
  uint8_t request[kMaxPacketSize];
  size_t n = buildWrite(request, id_, kRegMinAngleLimitL, data, sizeof(data));
  StatusPacket response;
  return transact(request, n, response, 0, 50);
}

bool FeetechServo::moveTo(uint16_t counts, uint16_t speed, uint8_t acceleration) {
  uint8_t request[kMaxPacketSize];
  size_t n = buildMove(request, id_, counts, speed, acceleration);
  StatusPacket response;
  return transact(request, n, response, 0, 30);
}

bool FeetechServo::readPosition(int32_t &counts) {
  uint8_t request[kMaxPacketSize];
  size_t n = buildRead(request, id_, kRegPresentPositionL, 2);
  StatusPacket response;
  if (!transact(request, n, response, 2, 30)) return false;
  counts = decodeSigned(get16(response.params));
  return true;
}

bool FeetechServo::readMoving(bool &moving) {
  uint8_t request[kMaxPacketSize];
  size_t n = buildRead(request, id_, kRegMoving, 1);
  StatusPacket response;
  if (!transact(request, n, response, 1, 30)) return false;
  moving = response.params[0] != 0;
  return true;
}

bool FeetechServo::readTemperature(uint8_t &celsius) {
  uint8_t request[kMaxPacketSize];
  size_t n = buildRead(request, id_, kRegPresentTemperature, 1);
  StatusPacket response;
  if (!transact(request, n, response, 1, 30)) return false;
  celsius = response.params[0];
  return true;
}

}  // namespace nwl
