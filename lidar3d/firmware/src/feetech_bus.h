// Feetech SCS/STS Busprotokoll (STS3215), reine Paketlogik.
//
// Hardwarefrei und nativ getestet - die UART-Anbindung steckt in
// feetech_servo.cpp. Das Protokoll aehnelt Dynamixel 1.0:
//
//   Anfrage:  FF FF <ID> <LEN> <INSTR> <PARAM...> <CHK>
//   Antwort:  FF FF <ID> <LEN> <ERR>   <PARAM...> <CHK>
//
//   LEN = Anzahl Parameter + 2
//   CHK = ~(ID + LEN + INSTR + Summe(PARAM)) & 0xFF
//
// Registeradressen gegen SCServo `SMS_STS.h` geprueft. Das ist wichtig:
// eine falsche Adresse schreibt im Zweifel ID (5) oder Baudrate (6) und
// macht das Servo unerreichbar.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace nwl {
namespace feetech {

static const uint8_t kHeaderByte = 0xFF;
static const size_t kMaxPacketSize = 32;

enum Instruction : uint8_t {
  kPing = 0x01,
  kRead = 0x02,
  kWrite = 0x03,
  kRegWrite = 0x04,
  kAction = 0x05,
  kReset = 0x06,
  kSyncWrite = 0x83,
};

// SMS/STS-Registerkarte. EPROM-Register (< 40) moeglichst nicht schreiben.
enum Reg : uint8_t {
  kRegModelL = 3,
  kRegId = 5,
  kRegBaudRate = 6,
  kRegMinAngleLimitL = 9,
  kRegMaxAngleLimitL = 11,
  kRegOfsL = 31,
  kRegMode = 33,
  kRegTorqueEnable = 40,
  kRegAcc = 41,
  kRegGoalPositionL = 42,
  kRegGoalTimeL = 44,
  kRegGoalSpeedL = 46,
  kRegTorqueLimitL = 48,
  kRegLock = 55,
  kRegPresentPositionL = 56,
  kRegPresentSpeedL = 58,
  kRegPresentLoadL = 60,
  kRegPresentVoltage = 62,
  kRegPresentTemperature = 63,
  kRegMoving = 66,
  kRegPresentCurrentL = 69,
};

enum Mode : uint8_t {
  kModePosition = 0,  // 0..360 Grad absolut - das brauchen wir
  kModeWheel = 1,     // Drehzahlregelung, endlos
  kModePwm = 2,
  kModeStep = 3,
};

// 16-Bit-Werte der STS-Reihe liegen little endian auf dem Bus
// (End == 0 in der SCServo-Bibliothek, im Gegensatz zur aelteren SCS-Reihe).
inline void put16(uint8_t *out, uint16_t value) {
  out[0] = static_cast<uint8_t>(value);
  out[1] = static_cast<uint8_t>(value >> 8);
}

inline uint16_t get16(const uint8_t *in) {
  return static_cast<uint16_t>(in[0] | (in[1] << 8));
}

// Drehzahl und Last kommen als Betrag mit Vorzeichenbit 15 zurueck,
// nicht im Zweierkomplement.
inline int16_t decodeSigned(uint16_t raw) {
  if (raw & 0x8000) return static_cast<int16_t>(-(raw & 0x7FFF));
  return static_cast<int16_t>(raw);
}

inline uint8_t checksum(const uint8_t *packet, size_t len) {
  // Ueber ID, LEN, INSTR und alle Parameter - also ab Index 2 bis vor die
  // Pruefsumme.
  uint32_t sum = 0;
  for (size_t i = 2; i < len; ++i) sum += packet[i];
  return static_cast<uint8_t>(~sum);
}

// --- Anfragen bauen; Rueckgabe ist die Paketlaenge ------------------------

inline size_t buildWrite(uint8_t *out, uint8_t id, uint8_t reg,
                         const uint8_t *data, uint8_t count) {
  out[0] = kHeaderByte;
  out[1] = kHeaderByte;
  out[2] = id;
  out[3] = static_cast<uint8_t>(count + 3);  // reg + data, plus 2
  out[4] = kWrite;
  out[5] = reg;
  if (count > 0) memcpy(out + 6, data, count);
  size_t len = 6 + count;
  out[len] = checksum(out, len);
  return len + 1;
}

inline size_t buildWrite8(uint8_t *out, uint8_t id, uint8_t reg, uint8_t value) {
  return buildWrite(out, id, reg, &value, 1);
}

inline size_t buildWrite16(uint8_t *out, uint8_t id, uint8_t reg, uint16_t value) {
  uint8_t data[2];
  put16(data, value);
  return buildWrite(out, id, reg, data, 2);
}

inline size_t buildRead(uint8_t *out, uint8_t id, uint8_t reg, uint8_t count) {
  out[0] = kHeaderByte;
  out[1] = kHeaderByte;
  out[2] = id;
  out[3] = 4;  // reg + count, plus 2
  out[4] = kRead;
  out[5] = reg;
  out[6] = count;
  out[7] = checksum(out, 7);
  return 8;
}

inline size_t buildPing(uint8_t *out, uint8_t id) {
  out[0] = kHeaderByte;
  out[1] = kHeaderByte;
  out[2] = id;
  out[3] = 2;
  out[4] = kPing;
  out[5] = checksum(out, 5);
  return 6;
}

// Position, Geschwindigkeit und Beschleunigung in einem Schreibvorgang:
// die Register 41..47 liegen zusammenhaengend.
inline size_t buildMove(uint8_t *out, uint8_t id, uint16_t position,
                        uint16_t speed, uint8_t acceleration) {
  uint8_t data[7];
  data[0] = acceleration;  // 41 ACC
  put16(data + 1, position);  // 42/43 GOAL_POSITION
  put16(data + 3, 0);         // 44/45 GOAL_TIME, 0 = ueber Drehzahl geregelt
  put16(data + 5, speed);     // 46/47 GOAL_SPEED
  return buildWrite(out, id, kRegAcc, data, sizeof(data));
}

// --- Antworten lesen ------------------------------------------------------

struct StatusPacket {
  uint8_t id;
  uint8_t error;
  uint8_t paramCount;
  uint8_t params[kMaxPacketSize];
};

// Byte-Strom -> Statuspakete. Synchronisiert auf FF FF und prueft die
// Pruefsumme; der Bus ist halbduplex, also kommt haeufig das eigene Echo
// zuerst zurueck und muss weggeworfen werden.
class StatusParser {
 public:
  StatusParser() { reset(); }

  void reset() {
    fill_ = 0;
    checksumErrors_ = 0;
  }

  // true, sobald `out` ein gueltiges Paket enthaelt.
  bool push(uint8_t b, StatusPacket &out) {
    if (fill_ < 2) {
      // Auf FF FF warten.
      if (b == kHeaderByte) {
        buf_[fill_++] = b;
      } else {
        fill_ = 0;
      }
      return false;
    }
    // Weitere 0xFF direkt nach dem Kopf gehoeren noch zum Kopf: die ID 0xFF
    // ist die Broadcast-Adresse und antwortet nie.
    if (fill_ == 2 && b == kHeaderByte) return false;

    if (fill_ >= sizeof(buf_)) {
      fill_ = 0;
      return false;
    }
    buf_[fill_++] = b;

    // Ab Index 3 steht die Laenge; das Paket ist 4 + LEN Byte lang.
    if (fill_ < 4) return false;
    size_t total = 4 + buf_[3];
    if (total > sizeof(buf_)) {
      fill_ = 0;
      return false;
    }
    if (fill_ < total) return false;

    bool ok = checksum(buf_, total - 1) == buf_[total - 1];
    if (ok) {
      out.id = buf_[2];
      out.error = buf_[4];
      out.paramCount = static_cast<uint8_t>(buf_[3] - 2);
      memcpy(out.params, buf_ + 5, out.paramCount);
    } else {
      ++checksumErrors_;
    }
    fill_ = 0;
    return ok;
  }

  uint32_t checksumErrors() const { return checksumErrors_; }

 private:
  uint8_t buf_[kMaxPacketSize];
  size_t fill_;
  uint32_t checksumErrors_;
};

}  // namespace feetech
}  // namespace nwl
