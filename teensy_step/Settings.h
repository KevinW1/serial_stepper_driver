#ifndef SETTINGS_h
#define SETTINGS_h

struct __attribute__((packed)) Settings_struct {
    //motor
    uint8_t step_current;          // 4 bits
    uint8_t sleep_current;         // 4 bits
    uint8_t microstep_resolution;  // 4 bits
    uint8_t sleep_timeout;         //8 bits, 10s of ms
    //trajectory
    uint16_t top_speed;     // 8 bits
    uint16_t acceleration;  // 8 bits
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

#endif