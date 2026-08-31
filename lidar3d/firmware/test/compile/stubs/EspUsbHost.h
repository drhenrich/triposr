// Attrappe der Arduino-Bibliothek EspUsbHost.
//
// Sie bildet genau die API nach, von der lidar_link_usb.h ausgeht - und macht
// diese Annahme damit sichtbar und nachlesbar an einer Stelle. Uebersetzt der
// echte Build nicht, gehoert der Unterschied hierher uebertragen; dann faellt
// er beim naechsten Mal schon hier auf.
//
// Belegt aus einem laufenden Sketch: begin(), available(), read(), connected().
// Angenommen: task() und write().
#pragma once

#include <stddef.h>
#include <stdint.h>

class EspUsbHost {
 public:
  void begin() {}
  void task() {}
};

class EspUsbHostCdcSerial {
 public:
  explicit EspUsbHostCdcSerial(EspUsbHost &) {}
  void begin(int) {}
  int available() { return 0; }
  int read() { return -1; }
  size_t write(const uint8_t *, size_t n) { return n; }
  bool connected() const { return false; }
};
