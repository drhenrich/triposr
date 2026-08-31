// Gierachse mit dem Feetech STS3215: Schritt und Halt statt Dauerfahrt.
//
// Warum nicht kontinuierlich drehen: 10 Grad/s waeren 3.7 Prozent der
// Leerlaufdrehzahl des Servos. So weit unten regelt ein 1:345-Getriebe
// schlecht (Stick-Slip, Grenzzyklen der Lageregelung). Stattdessen wird je
// Scanebene angefahren, eingerastet und genau eine LiDAR-Umdrehung erfasst.
//
// Das ist nicht nur ein Ausweichmanoever, es ist der einfachere Entwurf:
//
//  * Der Gierwinkel wird am Absolutencoder *gemessen*, nicht aus der Zeit
//    hochgerechnet. Verlorene Schritte gibt es als Fehlerbild nicht mehr.
//  * Kein Endschalter, kein Referenzieren - der Encoder ist absolut.
//  * Keine Zeitstempel-Interpolation zwischen Messung und Winkel.
//  * Waehrend der Messung steht die Achse still, also keine Bewegungsunschaerfe
//    und keine Getriebevibration im Sensor.
//
// Preis: ein Sweep dauert rund 27 statt 18 Sekunden.
#pragma once

#include <stdint.h>

#include "feetech_servo.h"
#include "stream_proto.h"
#include "sweep_plan.h"

namespace nwl {

class YawAxis {
 public:
  bool begin();

  void startSweep();
  void stop();

  // Zustandsautomat; oft genug aufrufen (jede Millisekunde reicht).
  void update();

  // Von main.cpp, sobald eine vollstaendige LiDAR-Umdrehung erfasst ist.
  void planeCaptured();

  SweepState state() const;
  // true, solange an der aktuellen Ebene gemessen werden darf.
  bool captureActive() const { return phase_ == kCapturing; }

  // Gemessener Gierwinkel der Ebene, an der gerade gemessen wird.
  int32_t planeYawQ16() const { return planeYawQ16_; }
  int32_t yawQ16Now() const { return lastYawQ16_; }

  uint16_t planeIndex() const { return planeIndex_; }
  uint16_t planeCount() const { return plan_.planeCount(); }
  uint16_t sweepIndex() const { return sweepIndex_; }

  bool servoOk() const { return servoOk_; }
  uint32_t servoTimeouts() const { return servo_.timeouts(); }
  // 0, wenn die Abfrage fehlgeschlagen ist.
  uint16_t servoModel() const { return servoModel_; }

 private:
  enum Phase : uint8_t {
    kIdle,
    kApproaching,  // faehrt die naechste Ebene an
    kSettling,     // Getriebe zur Ruhe kommen lassen
    kCapturing,    // steht still, LiDAR misst
    kReturning,    // zurueck auf den Anfang
  };

  void gotoPlane(uint16_t index);
  void gotoCounts(int32_t counts, uint16_t speed);
  bool pollPosition(int32_t &counts);
  bool arrived(int32_t counts) const;

  FeetechServo servo_;
  SweepPlan plan_;
  Phase phase_ = kIdle;
  bool servoOk_ = false;
  uint16_t servoModel_ = 0;

  uint16_t planeIndex_ = 0;
  uint16_t sweepIndex_ = 0;
  int32_t goalCounts_ = 0;
  int32_t planeYawQ16_ = 0;
  int32_t lastYawQ16_ = 0;

  int64_t phaseStartUs_ = 0;
  int64_t deadlineUs_ = 0;
  int64_t lastPollUs_ = 0;
};

}  // namespace nwl
