// Winkelumrechnungen in Festkomma. Hardwarefrei, nativ getestet.
//
// Q16 heisst hier durchgaengig: Grad * 65536. Das ist die Einheit, in der
// Gierwinkel im Streamprotokoll stehen (siehe stream_proto.h).
#pragma once

#include <stdint.h>

namespace nwl {

inline int32_t degToQ16(double deg) {
  return static_cast<int32_t>(deg * 65536.0 + (deg >= 0 ? 0.5 : -0.5));
}

inline double q16ToDeg(int32_t q16) { return q16 / 65536.0; }

// Der STS3215 hat einen 12-Bit-Absolutencoder: 4096 Zaehlwerte auf 360 Grad,
// also 0.0879 Grad je Zaehlwert.
inline int32_t countsToQ16(int32_t counts, int32_t countsPerRev) {
  return static_cast<int32_t>((static_cast<int64_t>(counts) * (360LL << 16)) /
                              countsPerRev);
}

inline int32_t q16ToCounts(int32_t q16, int32_t countsPerRev) {
  int64_t scaled = static_cast<int64_t>(q16) * countsPerRev;
  int64_t half = (360LL << 16) / 2;
  if (scaled >= 0) {
    return static_cast<int32_t>((scaled + half) / (360LL << 16));
  }
  return static_cast<int32_t>((scaled - half) / (360LL << 16));
}

}  // namespace nwl
