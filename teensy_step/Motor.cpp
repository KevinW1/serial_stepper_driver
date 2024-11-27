#include "Motor.h"
#include "Settings.h"

// constructor
Motor::Motor(byte _pin_cs, byte _pin_step, byte _pin_dir, Settings_union &_settings)
    // init children
    : stepper{ AccelStepper::DRIVER, _pin_step, _pin_dir }, sd{} {
    // init
    pin_cs = _pin_cs;
    pin_step = _pin_step;
    pin_dir = _pin_dir;
    settings = &_settings;
}

void Motor::init() {
    SPI.begin();
    sd.setChipSelectPin(pin_cs);

    // Drive the STEP and DIR pins low initially.
    pinMode(pin_step, OUTPUT);
    pinMode(pin_dir, OUTPUT);
    digitalWrite(pin_step, LOW);
    digitalWrite(pin_dir, LOW);

    delay(1);
    // Reset the driver to its default settings and clear latched fault
    // conditions.
    // TODO: set better defaults in driver library
    sd.resetSettings();
    sd.clearFaults();
    update_settings();
    set_current(0b0000); // min
    enable_driver();
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
    return sd.verifySettings();
}

void Motor::set_home_speed(long home_speed) {
    stepper.setSpeed(home_speed);
}

void Motor::enable_driver() {
    if (!driver_enabled) {
        sd.enableDriver();
        driver_enabled = true;
    }
}

void Motor::disable_driver() {
    if (driver_enabled) {
        sd.disableDriver();
        driver_enabled = false;
    }
}

void Motor::goto_pos(long steps) {
    stepper.moveTo(steps);
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
