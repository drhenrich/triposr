// Dekoder fuer die "dense capsuled" Scandaten des RPLIDAR S2.
//
// Bewusst frei von Arduino- und IDF-Abhaengigkeiten, damit die Logik nativ
// getestet werden kann (siehe firmware/test/native). Gegenstueck auf der
// Hostseite: host/scan3d/rplidar.py - beide muessen dieselben Winkel liefern.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace nwl {

static const int kDenseCabinCount = 40;
static const int kDenseCapsuleSize = 4 + 2 * kDenseCabinCount;  // 84 Byte
static const uint8_t kExpSync1 = 0xA;  // oberes Nibble von Byte 0
static const uint8_t kExpSync2 = 0x5;  // oberes Nibble von Byte 1

struct DenseCapsule {
  uint16_t startAngleQ6;  // 0 .. 360*64-1
  bool startFlag;         // Bit 15: LiDAR meldet Beginn einer Umdrehung
  uint16_t distanceMm[kDenseCabinCount];
  int64_t timestampUs;  // Ankunft des letzten Bytes dieser Capsule
};

// Die 40 Messungen einer Capsule, mit fertig interpoliertem Winkelraster.
struct CapsuleSpan {
  uint16_t alphaStartQ6;
  int32_t alphaIncQ16;
  const uint16_t *distanceMm;  // zeigt in die zurueckgehaltene Capsule
  int64_t timestampUs;
  int8_t revolutionIndex;  // -1, wenn in dieser Capsule kein Nulldurchgang liegt
};

typedef void (*CapsuleSink)(void *ctx, const DenseCapsule &capsule);

// Byte-Stream -> Capsules. Synchronisiert auf das Nibble-Muster 0xA / 0x5 und
// prueft die XOR-Pruefsumme; bei einem Fehler wird byteweise resynchronisiert.
class CapsuleParser {
 public:
  CapsuleParser() { reset(); }

  void reset() {
    fill_ = 0;
    checksumErrors_ = 0;
    resyncs_ = 0;
    memset(&capsule_, 0, sizeof(capsule_));
  }

  // tLastByteUs: Ankunftszeit des letzten Bytes in data.
  // byteTimeNs: Dauer eines Bytes auf der Leitung (bei 1 Mbaud, 8N1: 10000 ns).
  void feed(const uint8_t *data, size_t len, int64_t tLastByteUs,
            uint32_t byteTimeNs, CapsuleSink sink, void *ctx) {
    for (size_t i = 0; i < len; ++i) {
      if (!push(data[i])) continue;
      int64_t trailing = static_cast<int64_t>(len - 1 - i);
      capsule_.timestampUs = tLastByteUs - (trailing * byteTimeNs) / 1000;
      sink(ctx, capsule_);
    }
  }

  uint32_t checksumErrors() const { return checksumErrors_; }
  uint32_t resyncs() const { return resyncs_; }

 private:
  // Ein Byte einschieben; true, sobald capsule_ eine gueltige Capsule enthaelt.
  bool push(uint8_t b) {
    if (fill_ == 0 && (b >> 4) != kExpSync1) {
      ++resyncs_;
      return false;
    }
    if (fill_ == 1 && (b >> 4) != kExpSync2) {
      ++resyncs_;
      // Das Byte koennte selbst der Beginn einer Capsule sein.
      fill_ = ((b >> 4) == kExpSync1) ? 1 : 0;
      if (fill_ == 1) buf_[0] = b;
      return false;
    }
    buf_[fill_++] = b;
    if (fill_ < kDenseCapsuleSize) return false;

    if (checksumOk()) {
      unpack();
      fill_ = 0;
      return true;
    }
    ++checksumErrors_;
    slideAndResync();
    return false;
  }

  bool checksumOk() const {
    uint8_t expected = (buf_[0] & 0x0F) | ((buf_[1] & 0x0F) << 4);
    uint8_t actual = 0;
    for (int i = 2; i < kDenseCapsuleSize; ++i) actual ^= buf_[i];
    return expected == actual;
  }

  // Um ein Byte weiterschieben und die naechste gueltige Sync-Position suchen.
  void slideAndResync() {
    int start = 1;
    while (start < fill_ - 1) {
      if ((buf_[start] >> 4) == kExpSync1 && (buf_[start + 1] >> 4) == kExpSync2) {
        break;
      }
      ++start;
    }
    resyncs_ += static_cast<uint32_t>(start);
    memmove(buf_, buf_ + start, static_cast<size_t>(fill_ - start));
    fill_ -= start;
  }

  void unpack() {
    uint16_t raw = static_cast<uint16_t>(buf_[2] | (buf_[3] << 8));
    capsule_.startAngleQ6 = raw & 0x7FFF;
    capsule_.startFlag = (raw & 0x8000) != 0;
    for (int i = 0; i < kDenseCabinCount; ++i) {
      capsule_.distanceMm[i] =
          static_cast<uint16_t>(buf_[4 + 2 * i] | (buf_[5 + 2 * i] << 8));
    }
  }

  uint8_t buf_[kDenseCapsuleSize];
  int fill_;
  DenseCapsule capsule_;
  uint32_t checksumErrors_;
  uint32_t resyncs_;
};

// Der Winkel einer Messung ergibt sich erst aus dem Startwinkel der *naechsten*
// Capsule. Der Dekoder haelt deshalb immer eine Capsule zurueck.
class CapsuleDecoder {
 public:
  CapsuleDecoder() { reset(); }

  void reset() {
    hasPrev_ = false;
    pendingRevolution_ = false;
    memset(&prev_, 0, sizeof(prev_));
  }

  // Naechste Capsule einspeisen; fuellt out mit den Messungen der vorigen.
  // false, solange noch keine Vorgaengercapsule vorliegt.
  bool push(const DenseCapsule &current, CapsuleSpan &out) {
    if (!hasPrev_) {
      prev_ = current;
      hasPrev_ = true;
      return false;
    }

    int32_t curQ8 = static_cast<int32_t>(current.startAngleQ6) << 2;
    int32_t prevQ8 = static_cast<int32_t>(prev_.startAngleQ6) << 2;
    int32_t diffQ8 = curQ8 - prevQ8;
    if (prevQ8 > curQ8) diffQ8 += (360 << 8);
    int32_t incQ16 = (diffQ8 << 8) / kDenseCabinCount;

    out.alphaStartQ6 = prev_.startAngleQ6;
    out.alphaIncQ16 = incQ16;
    out.distanceMm = prev_.distanceMm;
    out.timestampUs = prev_.timestampUs;
    out.revolutionIndex = revolutionIndex(prevQ8 > curQ8,
                                          static_cast<int32_t>(prev_.startAngleQ6) << 10,
                                          incQ16);

    prev_ = current;
    return true;
  }

 private:
  // Index der Messung, bei der die 360-Grad-Grenze ueberschritten wird.
  // Faellt sie hinter die letzte Messung, wird sie vorgetragen.
  int8_t revolutionIndex(bool wrapped, int32_t startQ16, int32_t incQ16) {
    int8_t index = -1;
    if (pendingRevolution_) {
      index = 0;
      pendingRevolution_ = false;
    }
    if (!wrapped || incQ16 <= 0) return index;

    int32_t remaining = (360 << 16) - startQ16;
    int32_t at = (remaining + incQ16 - 1) / incQ16;  // aufrunden
    if (at >= kDenseCabinCount) {
      pendingRevolution_ = true;
    } else if (index < 0) {
      index = static_cast<int8_t>(at);
    }
    return index;
  }

  DenseCapsule prev_;
  bool hasPrev_;
  bool pendingRevolution_;
};

}  // namespace nwl
