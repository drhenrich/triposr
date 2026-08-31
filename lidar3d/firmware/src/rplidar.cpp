#include "rplidar.h"

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

#include "../include/config.h"

namespace nwl {

static const uint8_t kSyncByte = 0xA5;
static const uint8_t kSyncByte2 = 0x5A;

void RPLidar::begin(LidarLink *link) {
  link_ = link;
  parser_.reset();
  scanParser_.reset();
}

bool RPLidar::sendCommand(uint8_t command, const uint8_t *payload, uint8_t len) {
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
  return link_ != nullptr && link_->write(frame, n);
}

bool RPLidar::readDescriptor(uint32_t &length, uint8_t &dataType,
                               uint32_t timeoutMs) {
  if (link_ == nullptr) return false;
  uint8_t raw[7];
  // Der Deskriptor kann in mehreren Haeppchen ankommen, besonders ueber USB.
  size_t got = 0;
  const uint32_t slice = timeoutMs / 4 + 1;
  for (int attempt = 0; attempt < 4 && got < sizeof(raw); ++attempt) {
    got += link_->read(raw + got, sizeof(raw) - got, slice);
  }
  if (got != sizeof(raw)) return false;
  if (raw[0] != kSyncByte || raw[1] != kSyncByte2) return false;
  uint32_t word = static_cast<uint32_t>(raw[2]) | (static_cast<uint32_t>(raw[3]) << 8) |
                  (static_cast<uint32_t>(raw[4]) << 16) |
                  (static_cast<uint32_t>(raw[5]) << 24);
  length = word & 0x3FFFFFFF;
  dataType = raw[6];
  return true;
}

void RPLidar::stop() {
  sendCommand(kCmdStop, nullptr, 0);
  vTaskDelay(pdMS_TO_TICKS(20));
  if (link_ != nullptr) link_->flushInput();
  parser_.reset();
  scanParser_.reset();
  standard_ = false;
}

bool RPLidar::setMotorRpm(uint16_t rpm) {
  uint8_t payload[2] = {static_cast<uint8_t>(rpm), static_cast<uint8_t>(rpm >> 8)};
  return sendCommand(kCmdMotorSpeed, payload, sizeof(payload));
}

bool RPLidar::getConf(uint32_t confType, const uint8_t *extra, uint8_t extraLen,
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
  size_t got = 0;
  for (int attempt = 0; attempt < 4 && got < respLen; ++attempt) {
    got += link_->read(body + got, respLen - got, 125);
  }
  if (got != respLen || respLen < 4) return false;

  // Die ersten 4 Byte spiegeln den angefragten Typ.
  written = respLen - 4;
  if (written > outLen) written = outLen;
  memcpy(out, body + 4, written);
  return true;
}

int RPLidar::startDenseScan() {
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
  standard_ = false;
  return static_cast<int>(mode);
}

bool RPLidar::startStandardScan() {
  if (link_ == nullptr) return false;
  stop();

  if (!link_->canWrite()) {
    // Kein Rueckkanal: nichts anfordern, einfach zuhoeren. Der Parser
    // synchronisiert sich ueber die Pruefbits selbst, also schadet der
    // Versuch nichts - er gelingt nur dann, wenn der C1 von sich aus scannt.
    scanParser_.reset();
    standard_ = true;
    return true;
  }

  // Keine Modusabfrage noetig: der einfache Scan ist bei jedem RPLIDAR da,
  // und beim C1 ist er ohnehin der einzige, den es gibt.
  if (!sendCommand(kCmdScan, nullptr, 0)) return false;

  uint32_t respLen = 0;
  uint8_t dataType = 0;
  if (!readDescriptor(respLen, dataType, 500)) return false;
  if (dataType != kAnsMeasurement) return false;
  if (respLen != kStandardNodeSize) return false;

  scanParser_.reset();
  standard_ = true;
  return true;
}

void RPLidar::pollScan(ScanSampleSink sink, void *ctx, uint32_t waitMs) {
  if (link_ == nullptr) return;
  size_t got = link_->read(chunk_, sizeof(chunk_), waitMs);
  if (got == 0) return;
  scanParser_.feed(chunk_, got, sink, ctx);
}

void RPLidar::poll(CapsuleSink sink, void *ctx, uint32_t waitMs) {
  if (link_ == nullptr) return;
  size_t got = link_->read(chunk_, sizeof(chunk_), waitMs);
  if (got == 0) return;
  // read() kehrt zurueck, sobald etwas da ist oder die Zeit um ist; das letzte
  // Byte ist also gerade eingetroffen.
  int64_t now = esp_timer_get_time();
  parser_.feed(chunk_, got, now, LIDAR_BYTE_TIME_NS, sink, ctx);
}

}  // namespace nwl
