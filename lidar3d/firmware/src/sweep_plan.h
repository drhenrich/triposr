// Welche Gierwinkel ein Sweep anfaehrt. Hardwarefrei, nativ getestet.
//
// Der Bereich ist halboffen: [start, end). Grund ist die Kernaussage des
// Aufbaus - der LiDAR misst in seiner Ebene volle 360 Grad, also decken
// Gierwinkel 0 und 180 Grad exakt dieselbe Ebene ab. Waere das Ende
// eingeschlossen, wuerde die letzte Ebene die erste doppelt messen.
#pragma once

#include <stdint.h>

#include "angle_util.h"

namespace nwl {

struct SweepPlan {
  int32_t startQ16 = 0;
  int32_t endQ16 = degToQ16(180.0);
  int32_t stepQ16 = degToQ16(1.0);

  uint16_t planeCount() const {
    if (stepQ16 <= 0 || endQ16 <= startQ16) return 0;
    int64_t span = static_cast<int64_t>(endQ16) - startQ16;
    return static_cast<uint16_t>(span / stepQ16);
  }

  int32_t yawForPlane(uint16_t index) const {
    return startQ16 + static_cast<int32_t>(index) * stepQ16;
  }
};

}  // namespace nwl
