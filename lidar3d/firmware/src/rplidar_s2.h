// UART-Anbindung des RPLIDAR S2 an den ESP32-S3.
//
// Die Protokolllogik steckt in dense_capsule.h und ist dort nativ getestet.
// Diese Klasse macht nur den Port auf, fuehrt den Handshake und schiebt die
// gelesenen Bytes mit einem Zeitstempel in den Parser.
#pragma once

#include <driver/uart.h>
#include <stdint.h>

#include "dense_capsule.h"

namespace nwl {

// Kommandobytes des Slamtec-Protokolls
static const uint8_t kCmdStop = 0x25;
static const uint8_t kCmdReset = 0x40;
static const uint8_t kCmdExpressScan = 0x82;
static const uint8_t kCmdGetLidarConf = 0x84;
static const uint8_t kCmdMotorSpeed = 0xA8;

static const uint8_t kAnsDenseCapsuled = 0x85;
static const uint8_t kAnsGetLidarConf = 0x20;

static const uint32_t kConfScanModeTypical = 0x7C;
static const uint32_t kConfScanModeAnsType = 0x75;

class RPLidarS2 {
 public:
  bool begin(uart_port_t port, int rxPin, int txPin, int baudrate, int rxBuffer);

  void stop();
  bool setMotorRpm(uint16_t rpm);

  // Fragt den typischen Scanmodus ab, prueft dass er Dense-Capsules liefert,
  // und startet den Express-Scan. Gibt bei Erfolg die Modusnummer zurueck,
  // sonst -1.
  int startDenseScan();

  // Liest, was da ist, und ruft sink fuer jede vollstaendige Capsule.
  // Blockiert hoechstens waitMs.
  void poll(CapsuleSink sink, void *ctx, uint32_t waitMs);

  uint32_t checksumErrors() const { return parser_.checksumErrors(); }
  uint32_t resyncs() const { return parser_.resyncs(); }

 private:
  bool sendCommand(uint8_t command, const uint8_t *payload, uint8_t len);
  bool readDescriptor(uint32_t &length, uint8_t &dataType, uint32_t timeoutMs);
  bool getConf(uint32_t confType, const uint8_t *extra, uint8_t extraLen,
               uint8_t *out, size_t outLen, size_t &written);

  uart_port_t port_ = UART_NUM_1;
  CapsuleParser parser_;
  uint8_t chunk_[512];
};

}  // namespace nwl
