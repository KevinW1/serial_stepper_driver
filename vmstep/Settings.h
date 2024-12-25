#ifndef Settings_h
#define Settings_h
#include <Arduino.h>

struct __attribute__((packed)) Settings_struct {
    //motor control (4 bits each)
    uint8_t step_current : 4;   // max 0b1111
    uint8_t sleep_current : 4;  // max 0b1111
    uint8_t microstep_res : 4;  // max 0b1111
    uint8_t reserved : 4;       // padding for alignment
    uint8_t sleep_timeout;      // 10s of ms

    //trajectory (32 bits each)
    uint32_t top_speed;
    uint32_t acceleration;

    //limit switch flags packed into single byte
    struct {
        uint8_t enable_lim1 : 1;        // bit 0
        uint8_t enable_lim2 : 1;        // bit 1
        uint8_t enable_home : 1;        // bit 2
        uint8_t lim1_sig_polarity : 1;  // bit 3
        uint8_t lim2_sig_polarity : 1;  // bit 4
        uint8_t home_sig_polarity : 1;  // bit 5
        uint8_t reserved : 2;           // bits 6-7 reserved
    } flags;
};

union Settings_union {
    Settings_struct data;
    byte bytes[sizeof(Settings_struct)];
};

bool validate_settings(const Settings_union& settings);

#endif