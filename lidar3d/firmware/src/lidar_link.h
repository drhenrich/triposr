// Woher die Bytes des LiDAR kommen - unabhaengig davon, was sie bedeuten.
//
// Der C1 laesst sich auf zwei Wegen anschliessen:
//
//   USB    ueber den mitgelieferten USB-C-Adapter an den USB-Host-Port des
//          ESP32-S3. Der Adapter ist ein CDC-Seriell-Wandler, das Geraet
//          meldet sich also als serielles USB-Geraet.
//   UART   direkt an die TTL-Pins, ohne Adapter.
//
// Beides liefert denselben Bytestrom. Deshalb steht hier eine schmale
// Schnittstelle davor, und die Protokolllogik (standard_scan.h,
// dense_capsule.h, rplidar.cpp) muss den Unterschied nicht kennen - sie ist
// nativ getestet und bleibt davon unberuehrt.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace nwl {

class LidarLink {
 public:
  virtual ~LidarLink() {}

  // Bytes an den LiDAR schicken. Gibt true zurueck, wenn alle raus sind.
  virtual bool write(const uint8_t *data, size_t len) = 0;

  // Bis zu maxLen Bytes lesen, hoechstens timeoutMs lang warten.
  // Rueckgabe: Anzahl gelesener Bytes, 0 wenn nichts kam.
  virtual size_t read(uint8_t *out, size_t maxLen, uint32_t timeoutMs) = 0;

  // Alles verwerfen, was noch im Eingang liegt.
  virtual void flushInput() = 0;

  // Ist der LiDAR ueberhaupt da? Bei UART immer true - dort gibt es kein
  // Anstecken und Abziehen. Am USB haengt es daran, ob sich ein Geraet
  // angemeldet hat.
  virtual bool connected() const = 0;

  // Kann dieser Weg ueberhaupt senden? Am USB-Host haengt das an der
  // Bibliothek; ohne Senden laesst sich der Scan nicht anfordern, und die
  // Firmware muss darauf hoffen, dass der C1 von selbst losscannt.
  virtual bool canWrite() const { return true; }
};

}  // namespace nwl
