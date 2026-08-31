// Attrappe der Arduino-API. Nur so viel, wie die Firmware anfasst.
//
// Zweck ist ausschliesslich der Uebersetzungstest: hier laeuft nichts, hier
// wird nur geprueft, ob der Code syntaktisch und typmaessig stimmt. Siehe
// ../README.md.
#pragma once

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef uint8_t byte;

class SerialStub {
 public:
  void begin(unsigned long) {}
  void print(const char *) {}
  void print(char) {}
  void println() {}
  void println(const char *) {}
  int printf(const char *, ...) { return 0; }
};

extern SerialStub Serial;

inline void delay(unsigned long) {}
inline unsigned long millis() { return 0; }
