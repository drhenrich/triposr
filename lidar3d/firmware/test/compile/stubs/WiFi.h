// Attrappe von WiFi.h - siehe Arduino.h.
#pragma once

#include <stdint.h>

#include "Arduino.h"

class IPAddressStub {
 public:
  class Str {
   public:
    const char *c_str() const { return "0.0.0.0"; }
  };
  Str toString() const { return Str(); }
};

enum WiFiModeStub { WIFI_AP, WIFI_STA };
enum WiFiStatusStub { WL_CONNECTED = 3, WL_DISCONNECTED = 6 };

class WiFiClass {
 public:
  void mode(WiFiModeStub) {}
  bool softAP(const char *, const char * = nullptr) { return true; }
  IPAddressStub softAPIP() { return IPAddressStub(); }
  void begin(const char *, const char * = nullptr) {}
  int status() { return WL_CONNECTED; }
  IPAddressStub localIP() { return IPAddressStub(); }
};

extern WiFiClass WiFi;

class WiFiClient {
 public:
  operator bool() const { return connected_; }
  bool connected() const { return connected_; }
  int available() { return 0; }
  int read() { return -1; }
  int write(const uint8_t *, size_t n) { return static_cast<int>(n); }
  void setNoDelay(bool) {}
  void stop() {}
  IPAddressStub remoteIP() { return IPAddressStub(); }

 private:
  bool connected_ = false;
};

class WiFiServer {
 public:
  explicit WiFiServer(uint16_t) {}
  void begin() {}
  WiFiClient available() { return WiFiClient(); }
};
