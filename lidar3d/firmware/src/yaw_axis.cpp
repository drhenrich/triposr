#include "yaw_axis.h"

#include <Arduino.h>
#include <TMCStepper.h>
#include <esp_timer.h>

#include "../include/config.h"

namespace nwl {

static TMC2209Stepper g_driver(&Serial2, TMC_RSENSE, TMC_ADDRESS);

bool YawAxis::begin() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(ENDSTOP_PIN, ENDSTOP_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
  enableStepper(false);

  Serial2.begin(TMC_BAUDRATE, SERIAL_8N1, TMC_RX_PIN, TMC_TX_PIN);
  g_driver.begin();
  g_driver.toff(4);
  g_driver.rms_current(TMC_RMS_CURRENT_MA);
  g_driver.microsteps(TMC_MICROSTEPS);
  // StealthChop: leise und vibrationsarm. Vibration ueber das Gehaeuse ist der
  // wichtigste Stoerfaktor fuer die Distanzmessung waehrend der Fahrt.
  g_driver.en_spreadCycle(false);
  g_driver.pwm_autoscale(true);
  g_driver.intpol(true);

  degPerStep_ = degreesPerStep(MOTOR_FULL_STEPS, TMC_MICROSTEPS, GEAR_RATIO);
  parkedYawQ16_ = degToQ16(YAW_MIN_DEG);
  model_.start(esp_timer_get_time(), parkedYawQ16_, 0, +1);

  // Antwortet der Treiber ueber UART? version() liefert 0x21 beim TMC2209.
  return g_driver.version() == 0x21;
}

void YawAxis::enableStepper(bool on) {
  digitalWrite(ENABLE_PIN, on ? LOW : HIGH);  // EN ist low-aktiv
}

bool YawAxis::endstopTriggered() const {
  int level = digitalRead(ENDSTOP_PIN);
  return ENDSTOP_ACTIVE_LOW ? (level == LOW) : (level == HIGH);
}

void YawAxis::beginMove(double rateDegS, int8_t direction, int32_t fromQ16) {
  digitalWrite(DIR_PIN, direction > 0 ? HIGH : LOW);
  enableStepper(true);

  double hz = stepHzForRate(rateDegS, degPerStep_);
  // Richtungswechsel und Freigabe brauchen beim TMC2209 etwas Vorlauf.
  delayMicroseconds(200);

  if (stepPinAttached_ != STEP_PIN) {
    ledcAttach(STEP_PIN, static_cast<uint32_t>(hz + 0.5), 8);
    stepPinAttached_ = STEP_PIN;
  } else {
    ledcChangeFrequency(STEP_PIN, static_cast<uint32_t>(hz + 0.5), 8);
  }
  ledcWrite(STEP_PIN, 128);  // 50 % Tastverhaeltnis

  model_.start(esp_timer_get_time(), fromQ16, degPerUsQ32(degPerStep_, hz), direction);
  moveDeadlineUs_ = esp_timer_get_time() + YAW_MOVE_TIMEOUT_MS * 1000LL;
}

void YawAxis::stop() {
  if (stepPinAttached_ == STEP_PIN) ledcWrite(STEP_PIN, 0);
  parkedYawQ16_ = yawQ16Now();
  enableStepper(false);
  state_ = kStateIdle;
}

void YawAxis::startHoming() {
  state_ = kStateHoming;
  // Richtung auf den Endschalter zu; der Winkel ist waehrend des Homings
  // unbekannt und wird am Anschlag gesetzt.
  beginMove(YAW_HOMING_RATE_DEG_S, -1, degToQ16(YAW_MAX_DEG));
}

void YawAxis::startSweep() {
  sweepIndex_++;
  state_ = kStateSweeping;
  targetQ16_ = degToQ16(YAW_MAX_DEG);
  beginMove(YAW_SWEEP_RATE_DEG_S, +1, degToQ16(YAW_MIN_DEG));
}

void YawAxis::startReturn() {
  state_ = kStateReturning;
  targetQ16_ = degToQ16(YAW_MIN_DEG);
  beginMove(YAW_RETURN_RATE_DEG_S, -1, yawQ16Now());
}

int32_t YawAxis::yawQ16At(int64_t tUs) const {
  if (state_ == kStateIdle) return parkedYawQ16_;
  return model_.atQ16(tUs);
}

int32_t YawAxis::yawQ16Now() const { return yawQ16At(esp_timer_get_time()); }

void YawAxis::update() {
  if (state_ == kStateIdle) return;

  int64_t now = esp_timer_get_time();
  if (now > moveDeadlineUs_) {
    // Endschalter defekt oder Achse blockiert - lieber stehen bleiben.
    stop();
    parkedYawQ16_ = degToQ16(YAW_MIN_DEG);
    return;
  }

  switch (state_) {
    case kStateHoming:
      if (endstopTriggered()) {
        stop();
        parkedYawQ16_ = degToQ16(YAW_MIN_DEG);
      }
      break;

    case kStateSweeping:
      if (model_.atQ16(now) >= targetQ16_) {
        stop();
        parkedYawQ16_ = targetQ16_;
      }
      break;

    case kStateReturning:
      // Der Endschalter hat das letzte Wort, der Winkel ist nur die Schaetzung.
      if (endstopTriggered() || model_.atQ16(now) <= targetQ16_) {
        stop();
        parkedYawQ16_ = degToQ16(YAW_MIN_DEG);
      }
      break;

    default:
      break;
  }
}

}  // namespace nwl
