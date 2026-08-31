// Frameprotokoll ESP32-S3 -> Host. Gegenstueck: host/scan3d/stream.py.
// Alle Felder little endian; das ist auf ESP32 und auf x86/ARM-Hosts nativ.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "dense_capsule.h"
#include "standard_scan.h"

namespace nwl {

static const uint16_t kMagic = 0x4E57;  // 'NW'
static const size_t kHeaderSize = 8;

enum FrameType : uint8_t {
  kFrameHello = 0,
  kFrameCapsule = 1,  // S2: 40 Messungen auf gleichmaessigem Winkelraster
  kFrameStatus = 2,
  kFrameScan = 3,     // C1: Messungen mit eigenem Winkel je Stueck
};

enum FrameFlags : uint8_t {
  kFlagNewRevolution = 1 << 0,
  kFlagSweepActive = 1 << 1,
  kFlagSweepReversed = 1 << 2,
};

enum SweepState : uint8_t {
  kStateIdle = 0,
  kStateHoming = 1,
  kStateSweeping = 2,
  kStateReturning = 3,
};

static const size_t kCapsulePayloadSize = 16 + 2 * kDenseCabinCount;  // 96
static const size_t kCapsuleFrameSize = kHeaderSize + kCapsulePayloadSize;  // 104
static const size_t kHelloPayloadSize = 20;
static const size_t kStatusPayloadSize = 20;

// Scanframe (C1): Kopf mit Gierwinkel und Anzahl, dann je Messung 4 Byte.
// Der einfache Scanmodus liefert zu jeder Messung ihren eigenen Winkel, und
// der ist nicht gleichmaessig verteilt - anders als beim S2 laesst er sich
// also nicht aus Startwinkel und Schrittweite rekonstruieren. Deshalb geht er
// mit ueber die Leitung: 4 Byte * 5000/s = 20 kB/s, unkritisch.
static const int kScanMaxSamples = 32;
static const size_t kScanHeadSize = 12;
static const size_t kScanSampleSize = 4;
static const size_t kScanMaxPayloadSize =
    kScanHeadSize + kScanSampleSize * kScanMaxSamples;  // 140
static const size_t kScanMaxFrameSize = kHeaderSize + kScanMaxPayloadSize;  // 148

static const size_t kMaxFrameSize =
    kCapsuleFrameSize > kScanMaxFrameSize ? kCapsuleFrameSize : kScanMaxFrameSize;

namespace detail {
inline void put16(uint8_t *p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
}
inline void put32(uint8_t *p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
  p[2] = static_cast<uint8_t>(v >> 16);
  p[3] = static_cast<uint8_t>(v >> 24);
}
}  // namespace detail

inline size_t writeHeader(uint8_t *out, uint8_t type, uint8_t flags, uint16_t seq,
                          uint16_t payloadLen) {
  detail::put16(out, kMagic);
  out[2] = type;
  out[3] = flags;
  detail::put16(out + 4, seq);
  detail::put16(out + 6, payloadLen);
  return kHeaderSize;
}

// Eine Capsule mit zugeordnetem Gierwinkel serialisieren.
// yawStartQ16 / yawEndQ16 sind die Gierwinkel der ersten Messung und der
// ersten Messung der naechsten Capsule.
inline size_t writeCapsuleFrame(uint8_t *out, uint16_t seq, uint8_t flags,
                                const CapsuleSpan &span, uint32_t yawStartQ16,
                                uint32_t yawEndQ16) {
  writeHeader(out, kFrameCapsule, flags, seq,
              static_cast<uint16_t>(kCapsulePayloadSize));
  uint8_t *p = out + kHeaderSize;
  detail::put32(p + 0, yawStartQ16);
  detail::put32(p + 4, yawEndQ16);
  detail::put32(p + 8, static_cast<uint32_t>(span.alphaIncQ16));
  detail::put16(p + 12, span.alphaStartQ6);
  detail::put16(p + 14, 0);  // reserviert
  for (int i = 0; i < kDenseCabinCount; ++i) {
    detail::put16(p + 16 + 2 * i, span.distanceMm[i]);
  }
  return kCapsuleFrameSize;
}

// Eine Gruppe Messungen aus dem einfachen Scanmodus serialisieren.
// Eine Gruppe endet spaetestens nach kScanMaxSamples und immer an einer
// Umdrehungsgrenze - so gehoert ein Frame nie zu zwei Umdrehungen, und
// kFlagNewRevolution gilt fuer den ganzen Frame.
inline size_t writeScanFrame(uint8_t *out, uint16_t seq, uint8_t flags,
                             const ScanSample *samples, int count,
                             uint32_t yawStartQ16, uint32_t yawEndQ16) {
  if (count > kScanMaxSamples) count = kScanMaxSamples;
  const size_t payloadLen = kScanHeadSize + kScanSampleSize * count;
  writeHeader(out, kFrameScan, flags, seq, static_cast<uint16_t>(payloadLen));
  uint8_t *p = out + kHeaderSize;
  detail::put32(p + 0, yawStartQ16);
  detail::put32(p + 4, yawEndQ16);
  detail::put16(p + 8, static_cast<uint16_t>(count));
  detail::put16(p + 10, 0);  // reserviert
  for (int i = 0; i < count; ++i) {
    detail::put16(p + kScanHeadSize + kScanSampleSize * i, samples[i].angleQ6);
    detail::put16(p + kScanHeadSize + kScanSampleSize * i + 2, samples[i].distanceMm);
  }
  return kHeaderSize + payloadLen;
}

inline size_t writeHelloFrame(uint8_t *out, uint16_t seq, uint16_t fwVersion,
                              uint16_t lidarRpm, int32_t offsetRadialUm,
                              int32_t offsetAxialUm, uint32_t yawMinQ16,
                              uint32_t yawMaxQ16) {
  writeHeader(out, kFrameHello, 0, seq, static_cast<uint16_t>(kHelloPayloadSize));
  uint8_t *p = out + kHeaderSize;
  detail::put16(p + 0, fwVersion);
  detail::put16(p + 2, lidarRpm);
  detail::put32(p + 4, static_cast<uint32_t>(offsetRadialUm));
  detail::put32(p + 8, static_cast<uint32_t>(offsetAxialUm));
  detail::put32(p + 12, yawMinQ16);
  detail::put32(p + 16, yawMaxQ16);
  return kHeaderSize + kHelloPayloadSize;
}

inline size_t writeStatusFrame(uint8_t *out, uint16_t seq, uint16_t sweepIndex,
                               uint8_t state, uint32_t yawQ16, uint32_t capsules,
                               uint32_t checksumErrors, uint32_t droppedFrames) {
  writeHeader(out, kFrameStatus, 0, seq, static_cast<uint16_t>(kStatusPayloadSize));
  uint8_t *p = out + kHeaderSize;
  detail::put16(p + 0, sweepIndex);
  p[2] = state;
  p[3] = 0;  // reserviert
  detail::put32(p + 4, yawQ16);
  detail::put32(p + 8, capsules);
  detail::put32(p + 12, checksumErrors);
  detail::put32(p + 16, droppedFrames);
  return kHeaderSize + kStatusPayloadSize;
}

}  // namespace nwl
