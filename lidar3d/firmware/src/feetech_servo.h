// UART-Anbindung des STS3215 an den ESP32-S3.
//
// Die Paketlogik steckt in feetech_bus.h und ist dort nativ getestet; hier
// kommen nur Port, Halbduplex und Zeitueberwachung dazu.
//
// Halbduplex: liegt SERVO_DIR_PIN >= 0, schaltet der ESP32 die Richtung
// selbst um (UART_MODE_RS485_HALF_DUPLEX schaltet den Empfang waehrend des
// Sendens ab, sodass kein Echo im Puffer landet). Sonst wird ein externer
// Bustreiber vorausgesetzt, etwa das Feetech-Adapterboard FE-URT-1.
#pragma once

#include <driver/uart.h>
#include <stdint.h>

#include "feetech_bus.h"

namespace nwl {

class FeetechServo {
 public:
  bool begin(uart_port_t port, int rxPin, int txPin, int dirPin, int baudrate,
             uint8_t id);

  bool ping();
  bool setMode(uint8_t mode);
  bool setTorque(bool enabled);
  bool setAngleLimits(uint16_t minCounts, uint16_t maxCounts);

  // Fahrbefehl. speed in Zaehlwerten/s, acceleration in 100 Zaehlwerten/s^2.
  bool moveTo(uint16_t counts, uint16_t speed, uint8_t acceleration);

  bool readPosition(int32_t &counts);
  bool readMoving(bool &moving);
  bool readTemperature(uint8_t &celsius);

  uint32_t timeouts() const { return timeouts_; }
  uint32_t checksumErrors() const { return parser_.checksumErrors(); }

 private:
  // Sendet und wartet auf die Antwort. expectParams == 0 heisst: nur
  // Statusquittung erwartet.
  bool transact(const uint8_t *request, size_t length,
                feetech::StatusPacket &response, uint8_t expectParams,
                uint32_t timeoutMs);
  bool writeOnly(const uint8_t *request, size_t length);

  uart_port_t port_ = UART_NUM_2;
  uint8_t id_ = 1;
  feetech::StatusParser parser_;
  uint32_t timeouts_ = 0;
};

}  // namespace nwl
