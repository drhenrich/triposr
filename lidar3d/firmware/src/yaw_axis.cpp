#include "yaw_axis.h"

#include <Arduino.h>
#include <esp_timer.h>

#include "../include/config.h"

namespace nwl {

bool YawAxis::begin() {
  plan_.startQ16 = degToQ16(YAW_MIN_DEG);
  plan_.endQ16 = degToQ16(YAW_MAX_DEG);
  plan_.stepQ16 = degToQ16(YAW_PLANE_STEP_DEG);

  servoOk_ = servo_.begin(static_cast<uart_port_t>(SERVO_UART_NUM), SERVO_RX_PIN,
                          SERVO_TX_PIN, SERVO_DIR_PIN, SERVO_BAUDRATE, SERVO_ID);
  if (!servoOk_) return false;

  // Lagemodus: 0..360 Grad absolut. Der Encoder ist absolut, also gibt es
  // weder Referenzfahrt noch Endschalter.
  if (!servo_.setMode(feetech::kModePosition)) servoOk_ = false;
  if (!servo_.setTorque(true)) servoOk_ = false;

  int32_t counts = 0;
  if (servo_.readPosition(counts)) {
    lastYawQ16_ = countsToQ16(counts, SERVO_COUNTS_PER_REV);
    planeYawQ16_ = lastYawQ16_;
  }
  return servoOk_;
}

void YawAxis::gotoCounts(int32_t counts, uint16_t speed) {
  if (counts < 0) counts = 0;
  if (counts > SERVO_COUNTS_PER_REV - 1) counts = SERVO_COUNTS_PER_REV - 1;
  goalCounts_ = counts;
  servo_.moveTo(static_cast<uint16_t>(counts), speed, SERVO_ACCELERATION);
  phase_ = kApproaching;
  phaseStartUs_ = esp_timer_get_time();
  deadlineUs_ = phaseStartUs_ + SERVO_MOVE_TIMEOUT_MS * 1000LL;
}

void YawAxis::gotoPlane(uint16_t index) {
  planeIndex_ = index;
  gotoCounts(q16ToCounts(plan_.yawForPlane(index), SERVO_COUNTS_PER_REV),
             SERVO_MOVE_SPEED);
}

void YawAxis::startSweep() {
  if (!servoOk_) return;
  ++sweepIndex_;
  gotoPlane(0);
}

void YawAxis::stop() {
  phase_ = kIdle;
}

SweepState YawAxis::state() const {
  switch (phase_) {
    case kIdle:
      return kStateIdle;
    case kReturning:
      return kStateReturning;
    case kApproaching:
      // Die Anfahrt auf die erste Ebene meldet sich als Referenzieren; fuer
      // den Host sieht ein Sweep damit aus wie bisher.
      return planeIndex_ == 0 ? kStateHoming : kStateSweeping;
    default:
      return kStateSweeping;
  }
}

bool YawAxis::arrived(int32_t counts) const {
  int32_t error = counts - goalCounts_;
  if (error < 0) error = -error;
  return error <= SERVO_ARRIVE_TOLERANCE_COUNTS;
}

bool YawAxis::pollPosition(int32_t &counts) {
  int64_t now = esp_timer_get_time();
  // Den Bus nicht schneller als noetig belasten - er teilt sich die CPU mit
  // dem LiDAR-Strom.
  if (now - lastPollUs_ < SERVO_POLL_INTERVAL_MS * 1000LL) return false;
  lastPollUs_ = now;
  if (!servo_.readPosition(counts)) return false;
  lastYawQ16_ = countsToQ16(counts, SERVO_COUNTS_PER_REV);
  return true;
}

void YawAxis::planeCaptured() {
  if (phase_ != kCapturing) return;

  uint16_t next = static_cast<uint16_t>(planeIndex_ + 1);
  if (next >= plan_.planeCount()) {
    // Fertig. Zurueck auf den Anfang, damit der naechste Sweep sofort starten
    // kann; die Rueckfahrt ist der einzige Moment, in dem Getriebespiel
    // ueberhaupt eine Rolle spielt.
    gotoCounts(q16ToCounts(plan_.startQ16, SERVO_COUNTS_PER_REV),
               SERVO_RETURN_SPEED);
    phase_ = kReturning;
    return;
  }
  gotoPlane(next);
}

void YawAxis::update() {
  if (phase_ == kIdle) return;

  int64_t now = esp_timer_get_time();

  switch (phase_) {
    case kApproaching:
    case kReturning: {
      int32_t counts = 0;
      bool haveCounts = pollPosition(counts);
      if (haveCounts && arrived(counts)) {
        if (phase_ == kReturning) {
          phase_ = kIdle;
        } else {
          phase_ = kSettling;
          phaseStartUs_ = now;
        }
        return;
      }
      if (now > deadlineUs_) {
        // Servo antwortet nicht oder die Achse haengt - lieber abbrechen als
        // eine Wolke mit falschen Winkeln aufnehmen.
        phase_ = kIdle;
      }
      break;
    }

    case kSettling:
      if (now - phaseStartUs_ >= PLANE_SETTLE_MS * 1000LL) {
        // Den tatsaechlichen Winkel uebernehmen, nicht den befohlenen.
        // Getriebespiel und Regelabweichung stehen damit in den Daten.
        int32_t counts = 0;
        lastPollUs_ = 0;  // eine Messung erzwingen
        if (pollPosition(counts)) {
          planeYawQ16_ = countsToQ16(counts, SERVO_COUNTS_PER_REV);
        } else {
          planeYawQ16_ = plan_.yawForPlane(planeIndex_);
        }
        phase_ = kCapturing;
        phaseStartUs_ = now;
      }
      break;

    case kCapturing:
      // Normalerweise beendet main.cpp die Ebene, sobald eine volle
      // LiDAR-Umdrehung erfasst ist. Bleiben die Umlaufmarken aus, geht es
      // nach Ablauf der Zeit trotzdem weiter.
      if (now - phaseStartUs_ >= PLANE_CAPTURE_TIMEOUT_MS * 1000LL) {
        planeCaptured();
      }
      break;

    default:
      break;
  }
}

}  // namespace nwl
