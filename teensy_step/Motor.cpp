#include "Motor.h"
#include "Settings.h"

// constructor
Motor::Motor(byte pin_cs, byte pin_step, byte pin_dir, Settings_union &settings)
    : stepper{AccelStepper::DRIVER, pin_step, pin_dir, &settings}, sd{}
 {
    // init
    this->pin_cs = pin_cs;
    this->pin_step = pin_step;
    this->pin_dir = pin_dir;
    this->settings = &settings;
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
    stepper.setMaxSpeed(100 * settings->data.top_speed);
    stepper.setAcceleration(20000);
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
