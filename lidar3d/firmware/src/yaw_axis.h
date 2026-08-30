// Gierachse: TMC2209 im Step/Dir-Betrieb, Schrittimpulse aus LEDC.
//
// LEDC erzeugt eine feste STEP-Frequenz in Hardware. Damit laeuft die Achse
// exakt gleichfoermig, die CPU hat null Last, und der Gierwinkel ist eine
// lineare Funktion der Zeit (yaw_model.h) - es muss nichts gezaehlt werden.
#pragma once

#include <stdint.h>

#include "stream_proto.h"
#include "yaw_model.h"

namespace nwl {

class YawAxis {
 public:
  // true, wenn der TMC2209 ueber UART geantwortet hat. Bei false laeuft die
  // Achse trotzdem, aber mit den Defaultwerten des Treibers.
  bool begin();

  void startHoming();
  void startSweep();
  void startReturn();
  void stop();

  // Zustandsautomat; oft genug aufrufen (jede Millisekunde reicht).
  void update();

  SweepState state() const { return state_; }
  bool sweepActive() const { return state_ == kStateSweeping; }
  uint16_t sweepIndex() const { return sweepIndex_; }

  // Gierwinkel zum Zeitpunkt tUs (Mikrosekunden, esp_timer-Basis).
  int32_t yawQ16At(int64_t tUs) const;
  int32_t yawQ16Now() const;

  double degreesPerStep() const { return degPerStep_; }

 private:
  void beginMove(double rateDegS, int8_t direction, int32_t fromQ16);
  void enableStepper(bool on);
  bool endstopTriggered() const;

  SweepState state_ = kStateIdle;
  YawModel model_;
  int32_t parkedYawQ16_ = 0;  // gueltiger Winkel, solange die Achse steht
  int32_t targetQ16_ = 0;
  int64_t moveDeadlineUs_ = 0;
  uint16_t sweepIndex_ = 0;
  double degPerStep_ = 0.0;
  int stepPinAttached_ = -1;
};

}  // namespace nwl
