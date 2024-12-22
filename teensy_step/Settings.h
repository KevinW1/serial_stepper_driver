#ifndef Settings_h
#define Settings_h

#include <Arduino.h>

struct SettingLimits {
    static constexpr uint8_t MAX_CURRENT = 0b1111;        // 4 bits
    static constexpr uint8_t MAX_MICROSTEP_RES = 0b1111;  // 4 bits
};

struct __attribute__((packed)) Settings_struct {
    //motor
    uint8_t step_current;   // 4 bits
    uint8_t sleep_current;  // 4 bits
    uint8_t microstep_res;  // 4 bits
    uint8_t sleep_timeout;  //8 bits, 10s of ms
    //trajectory
    uint32_t top_speed;     // 16 bits
    uint32_t acceleration;  // 16 bits
    //lims
    bool enable_lim1;        // 1 bit
    bool enable_lim2;        // 1 bit
    bool enable_home;        // 1 bit
    bool lim1_sig_polarity;  // 1 bit
    bool lim2_sig_polarity;  // 1 bit
    bool home_sig_polarity;  // 1 bit
};

union Settings_union {
    Settings_struct data;
    byte bytes[sizeof(Settings_struct)];
};

bool validate_settings(const Settings_union& settings);

#endif