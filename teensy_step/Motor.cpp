#include "Motor.h"
#include "Settings.h"

// constructor
Motor::Motor(byte pin_cs, byte pin_step, byte pin_dir, byte pin_enable, byte pin_sleep, Settings_union &_settings)
    : PIN_CS(pin_cs),
      PIN_STEP(pin_step),
      PIN_DIR(pin_dir),
      PIN_ENABLE(pin_enable),
      PIN_SLEEP(pin_sleep),
      settings(&_settings),
      // init children
      stepper{ AccelStepper::DRIVER, pin_step, pin_dir },
      sd{} {
}

void Motor::init() {
    SPI.begin();
    sd.setChipSelectPin(PIN_CS);

    // Drive the STEP and DIR pins low initially.
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);
    pinMode(PIN_SLEEP, OUTPUT);

    digitalWrite(PIN_STEP, LOW);
    digitalWrite(PIN_DIR, LOW);
    digitalWrite(PIN_ENABLE, LOW);
    digitalWrite(PIN_SLEEP, HIGH);

    delay(1);
    // Reset the driver to its default settings and clear latched fault
    // conditions.
    // TODO: set better defaults in driver library
    sd.resetSettings();
    set_current(0b0000);  // min
    sd.clearFaults();
    sd.enableDriver();
    // enable_driver();
}

bool Motor::set_current(uint8_t current) {
    // Rather than write directly to the register, this uses the
    // max milliamps as a work around to let us pass in the native
    // register bits without any scaling.  This does incur some unnecessary
    // float math, but otherwise results in the same value as setting
    // the register directly.  By doing this we can make use of the
    // library's register cashing and verifying.  One difference is
    // this does not invert the values, so current = 0 means lowest.
    // Internal math looks like this:
    // https://github.com/pololu/drv8434s-arduino/blob/master/DRV8434S.h
    //      uint8_t td = (current * 16 / fullCurrent); // convert 0-fullCurrent to 0-16
    //      if (td == 0) { td = 1; }                   // restrict to 1-16
    //      td = 16 - td;                              // convert 1-16 to 15-0 (15 = 6.25%, 0 = 100%)
    //      ctrl1 = (ctrl1 & 0b00001111) | (td << 4);
    sd.setCurrentMilliamps(current + 1, SettingLimits::MAX_CURRENT + 1);
    return sd.verifySettings();
}

bool Motor::update_settings() {
    // update planner
    stepper.setMaxSpeed(settings->data.top_speed);
    stepper.setAcceleration(settings->data.acceleration);
    // driver update
    sd.setStepMode(settings->data.microstep_res);
    return true; //sd.verifySettings();
}

void Motor::enable_driver() {
    // sd.enableDriver();
    digitalWrite(PIN_ENABLE, HIGH);
}

void Motor::disable_driver() {
    // sd.disableDriver();
    digitalWrite(PIN_ENABLE, LOW);
}

void Motor::goto_pos(long steps) {
    stepper.move(steps);
}

long Motor::position() {
    return stepper.currentPosition();
}

long Motor::steps_remaining() {
    return stepper.distanceToGo();
}

void Motor::run() {
    stepper.run();
}

void Motor::run_continuous() {
    stepper.runSpeed();
}

void Motor::stop() {
    stepper.stop();
}
void Motor::reset_position() {
    stepper.setCurrentPosition(stepper.targetPosition());
}
