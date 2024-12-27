#ifndef Settings_h
#define Settings_h
#include <Arduino.h>

static constexpr uint8_t MicroStep1_100 = 0b0000;  // Full step with 100% current
static constexpr uint8_t MicroStep1 = 0b0001;      // Full step with 71% current
static constexpr uint8_t MicroStep2_NC = 0b0010;   // Non-circular 1/2 step
static constexpr uint8_t MicroStep2 = 0b0011;      // Circular 1/2 step
static constexpr uint8_t MicroStep4 = 0b0100;
static constexpr uint8_t MicroStep8 = 0b0101;
static constexpr uint8_t MicroStep16 = 0b0110;
static constexpr uint8_t MicroStep32 = 0b0111;
static constexpr uint8_t MicroStep64 = 0b1000;
static constexpr uint8_t MicroStep128 = 0b1001;
static constexpr uint8_t MicroStep256 = 0b1010;

struct __attribute__((packed)) Settings_struct {
    //motor control
    uint8_t run_current : 4;
    uint8_t sleep_current : 4;
    uint8_t microstep_res : 4;
    uint8_t reserved : 4;   // padding for alignment
    uint8_t sleep_timeout;  // 10s of ms

    //trajectory
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
    Settings_struct data{
        .run_current = 0b0000,         // run current (min)
        .sleep_current = 0b0000,       // sleep current (min)
        .microstep_res = MicroStep16,  // microstep res
        .sleep_timeout = 100,         // sleep timeout 1s
        .top_speed = 2000,           // top speed
        .acceleration = 4000,        // acceleration
        .flags = {
            // limit switch flags
            .enable_lim1 = 0,
            .enable_lim2 = 0,
            .enable_home = 0,
            .lim1_sig_polarity = 0,
            .lim2_sig_polarity = 0,
            .home_sig_polarity = 0,
            .reserved = 0 }
    };
    byte bytes[sizeof(Settings_struct)];
};

inline bool validate_settings(const Settings_union& settings) {
    // Nothing to validate yet
    return true;
}

#endif