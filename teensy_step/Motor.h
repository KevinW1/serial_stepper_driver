#ifndef Motor_h
#define Motor_h

#include <arduino.h>
#include <SPI.h>
#include <DRV8434S.h>
#include <AccelStepper.h>
#include "Settings.h"

class Motor {
  public:
    uint8_t pin_cs;
    uint8_t pin_step;
    uint8_t pin_dir;
    Settings_union* settings;
    Motor(byte pin_cs, byte pin_step, byte pin_dir, Settings_union &settings);
    void goto_pos(long steps);
    void init();
    void enable_driver();
    void disable_driver();
    void update_settings();
    void run();
    void stop();

  private:
    AccelStepper stepper;
    DRV8434S sd;
};

#endif