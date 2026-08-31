// LidarLink ueber den USB-Host-Port des ESP32-S3.
//
// Der C1 haengt mit seinem USB-C-Adapter am ESP32. Der Adapter ist ein
// CDC-Seriell-Wandler, das Geraet meldet sich also als serielles USB-Geraet -
// derselbe Bytestrom wie an der UART, nur ein anderer Weg dorthin.
//
// ACHTUNG, angenommene API. Gebraucht wird die Arduino-Bibliothek EspUsbHost.
// Diese Datei ist bewusst die einzige Stelle, die sie anfasst, und hier steht,
// worauf sie sich verlaesst:
//
//   belegt (aus einem laufenden Sketch uebernommen):
//     EspUsbHost::begin()
//     EspUsbHostCdcSerial(EspUsbHost&), ::begin(baud)
//     ::available(), ::read(), ::connected()
//
//   angenommen, nicht belegt:
//     EspUsbHost::task()          - so benutzen es die Beispiele der
//                                   Bibliothek; ohne regelmaessigen Aufruf
//                                   laeuft der USB-Stack nicht weiter.
//     ::write(const uint8_t*, size_t)
//                                 - vorhanden, wenn die Klasse von Stream
//                                   erbt (wovon available()/read() ausgehen).
//
// Uebersetzt eine dieser Zeilen nicht, ist der Schaden hier eingegrenzt: es
// ist eine Datei, und die Protokolllogik dahinter bleibt unberuehrt. Kann
// nicht gesendet werden, faellt nur das Anfordern des Scans weg - siehe
// canWrite() und rplidar.cpp.
#pragma once

#include <EspUsbHost.h>

#include <Arduino.h>

#include "lidar_link.h"

namespace nwl {

class UsbLidarLink : public LidarLink {
 public:
  // Startet den USB-Host und oeffnet den CDC-Port. Das Geraet meldet sich
  // erst nach ein paar Millisekunden an, deshalb wartet begin() darauf -
  // aber nicht endlos: ohne LiDAR soll die Firmware weiterlaufen und den
  // Grund melden, nicht stehenbleiben.
  bool begin(int baudrate, uint32_t waitMs) {
    host_.begin();
    cdc_.begin(baudrate);

    const uint32_t deadline = millis() + waitMs;
    while (millis() < deadline) {
      host_.task();
      if (cdc_.connected()) {
        ready_ = true;
        return true;
      }
      delay(5);
    }
    return false;
  }

  bool write(const uint8_t *data, size_t len) override {
    if (!cdc_.connected()) return false;
    return cdc_.write(data, len) == len;
  }

  size_t read(uint8_t *out, size_t maxLen, uint32_t timeoutMs) override {
    const uint32_t deadline = millis() + timeoutMs;
    size_t got = 0;
    do {
      // Der USB-Stack laeuft nur weiter, solange er drankommt.
      host_.task();
      while (got < maxLen && cdc_.available() > 0) {
        int value = cdc_.read();
        if (value < 0) break;
        out[got++] = static_cast<uint8_t>(value);
      }
      if (got > 0) return got;
      delay(1);
    } while (millis() < deadline);
    return got;
  }

  void flushInput() override {
    host_.task();
    while (cdc_.available() > 0) cdc_.read();
  }

  bool connected() const override { return ready_ && cdc_.connected(); }

 private:
  EspUsbHost host_;
  EspUsbHostCdcSerial cdc_{host_};
  bool ready_ = false;
};

}  // namespace nwl
