// Gierwinkel als Funktion der Zeit - reine Festkommamathematik, ohne Hardware.
//
// Waehrend eines Sweeps erzeugt LEDC eine feste STEP-Frequenz. Der Gierwinkel
// ist damit exakt linear in der Zeit, und jede Messung laesst sich ueber ihren
// Zeitstempel einem Winkel zuordnen - ohne Schritte zaehlen zu muessen.
//
// Zur Groessenordnung: bei 10 deg/s entspricht 1 ms Latenz 0.01 deg. Die
// Zeitsynchronisation ist deshalb unkritisch, solange die Rate stimmt.
#pragma once

#include <stdint.h>

namespace nwl {

// Grad je Mikrosekunde, skaliert mit 2^32.
inline int32_t degPerUsQ32(double degPerStep, double stepHz) {
  return static_cast<int32_t>(degPerStep * stepHz / 1e6 * 4294967296.0 + 0.5);
}

// Schritte je Umdrehung der Gierachse aus Motor, Microstepping und Untersetzung.
inline double degreesPerStep(int fullStepsPerRev, int microsteps, double gearRatio) {
  return 360.0 / (static_cast<double>(fullStepsPerRev) * microsteps * gearRatio);
}

// STEP-Frequenz fuer eine gewuenschte Gierrate.
inline double stepHzForRate(double degPerSecond, double degPerStep) {
  return degPerSecond / degPerStep;
}

struct YawModel {
  int64_t t0Us = 0;
  int32_t yaw0Q16 = 0;
  int32_t ratePerUsQ32 = 0;  // Betrag der Gierrate
  int8_t direction = 1;      // +1 oder -1

  void start(int64_t nowUs, int32_t startYawQ16, int32_t rateQ32, int8_t dir) {
    t0Us = nowUs;
    yaw0Q16 = startYawQ16;
    ratePerUsQ32 = rateQ32;
    direction = dir;
  }

  int32_t atQ16(int64_t tUs) const {
    int64_t dt = tUs - t0Us;
    int64_t deltaQ16 = (dt * static_cast<int64_t>(ratePerUsQ32)) >> 16;
    return yaw0Q16 + static_cast<int32_t>(direction * deltaQ16);
  }

  // Zeitpunkt, zu dem ein Zielwinkel erreicht wird (fuer das Sweep-Ende).
  int64_t timeForQ16(int32_t targetQ16) const {
    if (ratePerUsQ32 == 0) return t0Us;
    int64_t deltaQ16 = static_cast<int64_t>(targetQ16 - yaw0Q16) * direction;
    if (deltaQ16 < 0) return t0Us;
    return t0Us + ((deltaQ16 << 16) / ratePerUsQ32);
  }
};

inline int32_t degToQ16(double deg) {
  return static_cast<int32_t>(deg * 65536.0 + (deg >= 0 ? 0.5 : -0.5));
}

inline double q16ToDeg(int32_t q16) { return q16 / 65536.0; }

}  // namespace nwl
