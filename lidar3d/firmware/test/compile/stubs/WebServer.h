// Attrappe von WebServer.h aus dem ESP32-Arduino-Core - siehe Arduino.h.
#pragma once

#include <stdint.h>

class WebServer {
 public:
  explicit WebServer(uint16_t) {}
  void on(const char *, void (*)()) {}
  void onNotFound(void (*)()) {}
  void begin() {}
  void handleClient() {}
  void send_P(int, const char *, const char *) {}
};
