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
    // Reset the driver to its default settings and clear latched fault conditions.
    // TODO: set better defaults in driver library
    sd.resetSettings();
    set_current(0b0000);  // min
    sd.clearFaults();
    sd.enableDriver();  // digital enable.
}

bool Motor::try_verify_settings() {
    // accounts for processing times inside the driver chip.
    for(int i = 0; i < 10; i++) {
        if (sd.verifySettings()) {
            return true;
        }
        delay(1);
    }
    return false;
}

bool Motor::set_current(uint8_t current) {
    sd.setCurrent(current);
    return try_verify_settings();
}

bool Motor::update_settings() {
    // update planner
    stepper.setMaxSpeed(settings->data.top_speed);
    stepper.setAcceleration(settings->data.acceleration);
    // driver update
    sd.setStepMode(settings->data.microstep_res);
    return try_verify_settings();
}

void Motor::enable_driver() {
    digitalWrite(PIN_ENABLE, HIGH);
    driver_enabled = true;
}

void Motor::disable_driver() {
    digitalWrite(PIN_ENABLE, LOW);
    driver_enabled = false;
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

void Motor::hard_stop() {
    stepper.setCurrentPosition(stepper.targetPosition());
}

void Motor::reset_position() {
    stepper.setCurrentPosition(0);
}

byte* Motor::get_fault_registers() {
    fault_registers[0] = sd.readFault();
    fault_registers[1] = sd.readDiag1();
    fault_registers[2] = sd.readDiag2();
    return fault_registers;
}
