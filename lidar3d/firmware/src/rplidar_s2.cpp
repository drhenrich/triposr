#include "rplidar_s2.h"

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

#include "../include/config.h"

namespace nwl {

static const uint8_t kSyncByte = 0xA5;
static const uint8_t kSyncByte2 = 0x5A;

bool RPLidarS2::begin(uart_port_t port, int rxPin, int txPin, int baudrate,
                      int rxBuffer) {
  port_ = port;

  uart_config_t cfg = {};
  cfg.baud_rate = baudrate;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;

  if (uart_driver_install(port_, rxBuffer, 0, 0, nullptr, 0) != ESP_OK) return false;
  if (uart_param_config(port_, &cfg) != ESP_OK) return false;
  if (uart_set_pin(port_, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) !=
      ESP_OK) {
    return false;
  }
  uart_flush_input(port_);
  parser_.reset();
  return true;
}

bool RPLidarS2::sendCommand(uint8_t command, const uint8_t *payload, uint8_t len) {
  uint8_t frame[64];
  size_t n = 0;
  frame[n++] = kSyncByte;
  frame[n++] = command;
  if (payload != nullptr) {
    if (static_cast<size_t>(len) + 4 > sizeof(frame)) return false;
    frame[n++] = len;
    memcpy(frame + n, payload, len);
    n += len;
    uint8_t checksum = 0;
    for (size_t i = 0; i < n; ++i) checksum ^= frame[i];
    frame[n++] = checksum;
  }
  return uart_write_bytes(port_, reinterpret_cast<const char *>(frame), n) ==
         static_cast<int>(n);
}

bool RPLidarS2::readDescriptor(uint32_t &length, uint8_t &dataType,
                               uint32_t timeoutMs) {
  uint8_t raw[7];
  int got = uart_read_bytes(port_, raw, sizeof(raw), pdMS_TO_TICKS(timeoutMs));
  if (got != static_cast<int>(sizeof(raw))) return false;
  if (raw[0] != kSyncByte || raw[1] != kSyncByte2) return false;
  uint32_t word = static_cast<uint32_t>(raw[2]) | (static_cast<uint32_t>(raw[3]) << 8) |
                  (static_cast<uint32_t>(raw[4]) << 16) |
                  (static_cast<uint32_t>(raw[5]) << 24);
  length = word & 0x3FFFFFFF;
  dataType = raw[6];
  return true;
}

void RPLidarS2::stop() {
  sendCommand(kCmdStop, nullptr, 0);
  vTaskDelay(pdMS_TO_TICKS(20));
  uart_flush_input(port_);
  parser_.reset();
}

bool RPLidarS2::setMotorRpm(uint16_t rpm) {
  uint8_t payload[2] = {static_cast<uint8_t>(rpm), static_cast<uint8_t>(rpm >> 8)};
  return sendCommand(kCmdMotorSpeed, payload, sizeof(payload));
}

bool RPLidarS2::getConf(uint32_t confType, const uint8_t *extra, uint8_t extraLen,
                        uint8_t *out, size_t outLen, size_t &written) {
  uint8_t payload[8];
  payload[0] = static_cast<uint8_t>(confType);
  payload[1] = static_cast<uint8_t>(confType >> 8);
  payload[2] = static_cast<uint8_t>(confType >> 16);
  payload[3] = static_cast<uint8_t>(confType >> 24);
  uint8_t len = 4;
  if (extra != nullptr && extraLen > 0) {
    if (static_cast<size_t>(4 + extraLen) > sizeof(payload)) return false;
    memcpy(payload + 4, extra, extraLen);
    len = static_cast<uint8_t>(4 + extraLen);
  }
  if (!sendCommand(kCmdGetLidarConf, payload, len)) return false;

  uint32_t respLen = 0;
  uint8_t dataType = 0;
  if (!readDescriptor(respLen, dataType, 500)) return false;
  if (dataType != kAnsGetLidarConf) return false;

  uint8_t body[64];
  if (respLen > sizeof(body)) return false;
  int got = uart_read_bytes(port_, body, respLen, pdMS_TO_TICKS(500));
  if (got != static_cast<int>(respLen) || respLen < 4) return false;

  // Die ersten 4 Byte spiegeln den angefragten Typ.
  written = respLen - 4;
  if (written > outLen) written = outLen;
  memcpy(out, body + 4, written);
  return true;
}

int RPLidarS2::startDenseScan() {
  stop();

  uint8_t buf[8];
  size_t n = 0;
  if (!getConf(kConfScanModeTypical, nullptr, 0, buf, sizeof(buf), n) || n < 2) {
    return -1;
  }
  uint16_t mode = static_cast<uint16_t>(buf[0] | (buf[1] << 8));

  uint8_t modeBytes[2] = {static_cast<uint8_t>(mode), static_cast<uint8_t>(mode >> 8)};
  if (!getConf(kConfScanModeAnsType, modeBytes, sizeof(modeBytes), buf, sizeof(buf), n) ||
      n < 1) {
    return -1;
  }
  // Nur der Dense-Modus passt bei 32000 Messungen/s durch die 1-Mbaud-UART.
  if (buf[0] != kAnsDenseCapsuled) return -1;

  uint8_t payload[5] = {static_cast<uint8_t>(mode), 0, 0, 0, 0};
  if (!sendCommand(kCmdExpressScan, payload, sizeof(payload))) return -1;

  uint32_t respLen = 0;
  uint8_t dataType = 0;
  if (!readDescriptor(respLen, dataType, 500)) return -1;
  if (dataType != kAnsDenseCapsuled) return -1;

  parser_.reset();
  return static_cast<int>(mode);
}

void RPLidarS2::poll(CapsuleSink sink, void *ctx, uint32_t waitMs) {
  int got = uart_read_bytes(port_, chunk_, sizeof(chunk_), pdMS_TO_TICKS(waitMs));
  if (got <= 0) return;
  // uart_read_bytes kehrt zurueck, sobald der Puffer voll ist oder die Zeit um
  // ist; das letzte Byte ist also gerade eingetroffen.
  int64_t now = esp_timer_get_time();
  parser_.feed(chunk_, static_cast<size_t>(got), now, LIDAR_BYTE_TIME_NS, sink, ctx);
}

}  // namespace nwl
