// Attrappe von WebSocketsServer.h (arduinoWebSockets, links2004).
// Nur die vier Aufrufe, die web_stream.cpp benutzt.
#pragma once

#include <stddef.h>
#include <stdint.h>

typedef enum {
  WStype_ERROR,
  WStype_DISCONNECTED,
  WStype_CONNECTED,
  WStype_TEXT,
  WStype_BIN,
} WStype_t;

class WebSocketsServer {
 public:
  explicit WebSocketsServer(uint16_t) {}
  void begin() {}
  void loop() {}
  void onEvent(void (*)(uint8_t, WStype_t, uint8_t *, size_t)) {}
  bool broadcastTXT(const char *) { return true; }
  bool broadcastBIN(const uint8_t *, size_t) { return true; }
};
