#include "Motor.h"
#include "Settings.h"

Motor::Motor(byte pin_cs, byte pin_step, byte pin_dir, byte pin_enable, byte pin_sleep, Settings_union &_settings)
    : PIN_CS(pin_cs),
      PIN_STEP(pin_step),
      PIN_DIR(pin_dir),
      PIN_ENABLE(pin_enable),
      PIN_SLEEP(pin_sleep),
      settings(&_settings),
      // init children.  AccelStepper for step rate, and DRV8434S for driver control.
      stepper{ AccelStepper::DRIVER, pin_step, pin_dir },
      driver{} {
}

void Motor::init() {
    SPI.begin();
    driver.setChipSelectPin(PIN_CS);

    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);
    pinMode(PIN_SLEEP, OUTPUT);

    digitalWrite(PIN_STEP, LOW);
    digitalWrite(PIN_DIR, LOW);
    digitalWrite(PIN_ENABLE, LOW); // high is enabled, low is disabled.
    digitalWrite(PIN_SLEEP, HIGH); // high is awake, low is sleep.

    delay(1);
    // Reset the driver to its default settings and clear latched fault conditions.
    // TODO: set better defaults in driver library
    driver.resetSettings();
    set_current(0b0000);  // min
    driver.clearFaults();
    driver.enableDriver();  // digital enable.
    disable_driver();   // physical disable.
}

void Motor::enable_driver() {
    digitalWrite(PIN_ENABLE, HIGH);
    driver_enabled = true;
}

void Motor::disable_driver() {
    digitalWrite(PIN_ENABLE, LOW);
    driver_enabled = false;
}

bool Motor::try_verify_settings() {
    // Accounts for processing times inside the driver chip.
    for(int i = 0; i < 10; i++) {
        if (driver.verifySettings()) {
            return true;
        }
        delay(1);
    }
    return false;
}

bool Motor::update_settings() {
    // update planner
    stepper.setMaxSpeed(settings->data.top_speed);
    stepper.setAcceleration(settings->data.acceleration);
    // driver update
    driver.setStepMode(settings->data.microstep_res);
    return try_verify_settings();
}

bool Motor::set_current(uint8_t current) {
    // this is dynamically switched based on controller state.
    driver.setCurrent(current);
    return try_verify_settings();
}

void Motor::goto_pos(long steps) {
    stepper.move(steps);
}

void Motor::reset_position() {
    stepper.setCurrentPosition(0);
}

void Motor::run() {
    stepper.run();
}

void Motor::run_continuous() {
    stepper.runSpeed();
}

void Motor::stop() {
    // includes deceleration
    stepper.stop();
}

void Motor::hard_stop() {
    stepper.setCurrentPosition(stepper.targetPosition());
}

long Motor::position() {
    return stepper.currentPosition();
}

long Motor::steps_remaining() {
    return stepper.distanceToGo();
}

byte* Motor::get_fault_registers() {
    fault_registers[0] = driver.readFault();
    fault_registers[1] = driver.readDiag1();
    fault_registers[2] = driver.readDiag2();
    return fault_registers;
}
