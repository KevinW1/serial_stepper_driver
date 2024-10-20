#include "Motor.h"
#include "Settings.h"

// constructor
Motor::Motor(byte _pin_cs, byte _pin_step, byte _pin_dir, Settings_union &_settings)
    // init children
    : stepper{AccelStepper::DRIVER, _pin_step, _pin_dir}, sd{}
 {
    // init
    pin_cs = _pin_cs;
    pin_step = _pin_step;
    pin_dir = _pin_dir;
    settings = &_settings;
}

void Motor::init() {
    SPI.begin();
    sd.setChipSelectPin(pin_cs);

    // // Drive the STEP and DIR pins low initially.
    pinMode(pin_step, OUTPUT);
    pinMode(pin_dir, OUTPUT);
    digitalWrite(pin_step, LOW);
    digitalWrite(pin_dir, LOW);

    delay(1);
    // Reset the driver to its default settings and clear latched fault
    // conditions.
    sd.resetSettings();
    sd.clearFaults();
    sd.setCurrentMilliamps(250);
    sd.setStepMode(DRV8434SStepMode::MicroStep32);
    enable_driver();
}

void Motor::update_settings() {
    stepper.setMaxSpeed(settings->data.top_speed);
    stepper.setAcceleration(settings->data.acceleration);
    // update driver
}

void Motor::enable_driver() {
    sd.enableDriver();
    // set state flag
}

void Motor::disable_driver() {
    sd.disableDriver();
    // set state flag
}

void Motor::goto_pos(long steps) {
    // move motor
    stepper.moveTo(steps);
}

void Motor::run() {
    stepper.run();
}

void Motor::stop() {
    stepper.stop();
}
