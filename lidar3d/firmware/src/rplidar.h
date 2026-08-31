// UART-Anbindung eines RPLIDAR an den ESP32-S3.
//
// Zwei Betriebsarten, je nach Geraet:
//
//   C1  einfacher Scanmodus (Antworttyp 0x81), 5 Byte je Messung.
//       5000 Messungen/s ergeben 25 kB/s - von 46 kB/s bei 460800 Baud.
//   S2  dense-capsuled Express-Scan (0x85), 40 Messungen in 84 Byte.
//       32000 Messungen/s waeren als Einzelmessungen 160 kB/s und passten
//       nicht durch die 1-Mbaud-UART; der dichte Modus ist dort Pflicht.
//
// Die Protokolllogik steckt in standard_scan.h und dense_capsule.h und ist
// dort nativ getestet. Diese Klasse macht nur den Port auf, fuehrt den
// Handshake und schiebt die gelesenen Bytes in den passenden Parser.
#pragma once

#include <driver/uart.h>
#include <stdint.h>

#include "dense_capsule.h"
#include "standard_scan.h"

namespace nwl {

// Kommandobytes des Slamtec-Protokolls
static const uint8_t kCmdScan = 0x20;
static const uint8_t kCmdStop = 0x25;
static const uint8_t kCmdReset = 0x40;
static const uint8_t kCmdExpressScan = 0x82;
static const uint8_t kCmdGetLidarConf = 0x84;
static const uint8_t kCmdMotorSpeed = 0xA8;

static const uint8_t kAnsMeasurement = 0x81;
static const uint8_t kAnsDenseCapsuled = 0x85;
static const uint8_t kAnsGetLidarConf = 0x20;

static const uint32_t kConfScanModeTypical = 0x7C;
static const uint32_t kConfScanModeAnsType = 0x75;

class RPLidar {
 public:
  bool begin(uart_port_t port, int rxPin, int txPin, int baudrate, int rxBuffer);

  void stop();
  bool setMotorRpm(uint16_t rpm);

  // Startet den einfachen Scanmodus (C1). Gibt true zurueck, wenn das Geraet
  // mit dem Antworttyp 0x81 bestaetigt. Braucht keine Modusabfrage - der
  // einfache Modus ist bei jedem RPLIDAR vorhanden.
  bool startStandardScan();

  // Fragt den typischen Scanmodus ab, prueft dass er Dense-Capsules liefert,
  // und startet den Express-Scan (S2). Gibt bei Erfolg die Modusnummer
  // zurueck, sonst -1.
  int startDenseScan();

  // Liest, was da ist, und ruft sink fuer jede vollstaendige Capsule.
  // Blockiert hoechstens waitMs. Nur nach startDenseScan sinnvoll.
  void poll(CapsuleSink sink, void *ctx, uint32_t waitMs);

  // Dasselbe fuer den einfachen Scanmodus: sink je Einzelmessung.
  void pollScan(ScanSampleSink sink, void *ctx, uint32_t waitMs);

  uint32_t checksumErrors() const {
    return standard_ ? scanParser_.checksumErrors() : parser_.checksumErrors();
  }
  uint32_t resyncs() const {
    return standard_ ? scanParser_.resyncs() : parser_.resyncs();
  }

  // True, wenn der einfache Scanmodus laeuft (C1) statt der Dense-Capsules.
  bool usesStandardScan() const { return standard_; }

 private:
  bool sendCommand(uint8_t command, const uint8_t *payload, uint8_t len);
  bool readDescriptor(uint32_t &length, uint8_t &dataType, uint32_t timeoutMs);
  bool getConf(uint32_t confType, const uint8_t *extra, uint8_t extraLen,
               uint8_t *out, size_t outLen, size_t &written);

  uart_port_t port_ = UART_NUM_1;
  CapsuleParser parser_;
  StandardScanParser scanParser_;
  bool standard_ = false;
  uint8_t chunk_[512];
};

}  // namespace nwl
