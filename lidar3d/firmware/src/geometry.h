// Kugelkoordinaten -> kartesische Punkte, auf dem Geraet.
//
// Spiegelt host/scan3d/geometry.py und ios/LidarKit/Geometry.swift; alle drei
// liefern dieselben Zahlen, geprueft in firmware/test/native gegen dieselben
// Werte wie der Pythontest.
//
// Gebraucht wird das, seit die Punkte als fertige Koordinaten an den Browser
// gehen: eine Webseite auf dem iPhone soll nicht die Einbaulage kennen
// muessen.
//
// Aufbau: der 2D-LiDAR steht hochkant, seine Scanebene enthaelt also die
// Drehachse. Damit ist eine Messung nichts anderes als eine Kugelkoordinate:
//
//     alpha = Polarwinkel, vom LiDAR (0 Grad = entlang +Z, nach oben)
//     psi   = Azimut, vom Servo
//     r     = Distanz
//
// Weil der LiDAR in seiner Ebene volle 360 Grad abdeckt, genuegen 180 Grad
// Gieren fuer die ganze Kugel - deshalb braucht der Aufbau keinen Schleifring.
//
// DIE HAEUFIGSTE FEHLERQUELLE ist alphaZeroDeg. Der LiDAR zaehlt seine Winkel
// ab einer Marke am Gehaeuse; hochkant montiert zeigt die zur Seite statt nach
// oben, beim C1 also um rund 90 Grad versetzt. Steht hier 0, wird der Abstand
// zur Decke als Radius verrechnet und beim Drehen zu einem Zylinder
// verschmiert - der Raum sieht rund aus. Siehe docs/02-geometrie.md.
#pragma once

#include <math.h>
#include <stdint.h>

namespace nwl {

struct MountGeometry {
  // Abstand des optischen Zentrums von der Drehachse, senkrecht dazu, in mm.
  float offsetRadialMm = 0.0f;
  // Hoehe des optischen Zentrums entlang der Drehachse, in mm.
  float offsetAxialMm = 0.0f;
  // Der LiDAR-Winkel, der nach oben zeigt.
  float alphaZeroDeg = 0.0f;
  // -1, wenn der LiDAR-Winkel entgegen der gewuenschten Richtung laeuft.
  float alphaSign = 1.0f;
  // Gierwinkel, der als Azimut 0 gilt.
  float yawZeroDeg = 0.0f;
  float yawSign = 1.0f;
};

struct Point3 {
  float x;
  float y;
  float z;
};

// Eine Messung in Meter-Weltkoordinaten. Z ist die Drehachse und zeigt hoch.
inline Point3 toCartesian(float distanceMm, float alphaDeg, float yawDeg,
                          const MountGeometry &mount) {
  const float degToRad = 3.14159265358979323846f / 180.0f;
  const float a = mount.alphaSign * (alphaDeg - mount.alphaZeroDeg) * degToRad;
  // In der Scanebene: u radial von der Achse weg, w entlang der Achse.
  // u darf negativ werden - genau deshalb genuegen 180 Grad Gieren.
  const float u = mount.offsetRadialMm + distanceMm * sinf(a);
  const float w = mount.offsetAxialMm + distanceMm * cosf(a);

  const float psi = mount.yawSign * (yawDeg - mount.yawZeroDeg) * degToRad;
  Point3 p;
  p.x = u * cosf(psi) / 1000.0f;
  p.y = u * sinf(psi) / 1000.0f;
  p.z = w / 1000.0f;
  return p;
}

// Gueltigkeitsfenster einer Messung, in mm.
struct RangeFilter {
  // Untergrenze oberhalb der Blindzone (Datenblatt: 50 mm) plus Reserve.
  float minMm = 150.0f;
  // Reichweite des C1; der S2 kaeme auf 30000.
  float maxMm = 12000.0f;

  bool accepts(float distanceMm) const {
    return distanceMm > 0.0f && distanceMm >= minMm && distanceMm <= maxMm;
  }
};

}  // namespace nwl
