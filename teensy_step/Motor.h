#ifndef Motor_h
#define Motor_h

#include <Arduino.h>
#include <SPI.h>
#include "DRV8434S.h"
#include <AccelStepper.h>
#include "Settings.h"

class Motor {
  public:
    const uint8_t PIN_CS;
    const uint8_t PIN_STEP;
    const uint8_t PIN_DIR;
    const uint8_t PIN_ENABLE;
    const uint8_t PIN_SLEEP;
    bool driver_enabled = false;
    Settings_union* settings;
    Motor(uint8_t pin_cs, uint8_t pin_step, uint8_t pin_dir, uint8_t pin_enable, uint8_t pin_sleep, Settings_union& settings);
    void goto_pos(long steps);
    long position();
    bool set_current(uint8_t current);
    void init();
    void enable_driver();
    void disable_driver();
    bool update_settings();
    void set_home_speed(long home_speed);
    void run();
    void run_continuous();
    void stop();
    long steps_remaining();
    void reset_position();

  private:
    AccelStepper stepper;
    DRV8434S sd;
};

#endif