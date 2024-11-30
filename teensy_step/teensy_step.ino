#include "Comms.h"
#include "Settings.h"
#include "Motor.h"

// pin definitions
// Motor
static constexpr byte PIN_SLEEP = 6;
static constexpr byte PIN_STEP = 7;
static constexpr byte PIN_DIR = 8;
static constexpr byte PIN_ENABLE = 9;
static constexpr byte PIN_SCS = 10;
static constexpr byte PIN_FAULT = 17;
// system
static constexpr byte PIN_LED_R = 4;
static constexpr byte PIN_LED_G = 5;
// Sensors
static constexpr byte PIN_LIM1 = 16;
static constexpr byte PIN_LIM2 = 15;
static constexpr byte PIN_HOME = 14;
// system control
static constexpr uint16_t LED_PULSE_PERIOD = 2000;  // 2 second cycle

Settings_union settings = {
    0b0011,  // run current (mid)
    0b0000,  // sleep current (min)
    0b0110,  // microstep res (1/16th)
    0x64,    // sleep timeout 1s
    0x07D0,  // top speed 2000
    0x4E20,  // accel 20,000
    0b0,     // enable lim1
    0b0,     // enable lim2
    0b1,     // enable home
    0b0,     // lim1 pol
    0b0,     // lim2 pol
    0b0,     // home pol
};

enum class Mode {
    idle,
    sleep,
    moving,
    homing,
    fault
};

union {
    long value;
    uint8_t bytes[4];
} position;

Mode device_mode = Mode::idle;
unsigned long idle_time = 0;
SerialTransciever Comms;
Motor motor(PIN_SCS, PIN_STEP, PIN_DIR, PIN_ENABLE, PIN_SLEEP, settings);

void setup() {
    reset_controller();
}

void reset_controller() {
    if (!validate_settings(settings)) {
        Comms.send(REPLY_FAULT, FAULT_INVALID_PARAMETERS);
        enter_fault_state();
        return;
    }
    // sensors
    pinMode(PIN_LIM1, INPUT_PULLUP);
    pinMode(PIN_LIM2, INPUT_PULLUP);
    pinMode(PIN_HOME, INPUT_PULLDOWN);
    // driver
    pinMode(PIN_FAULT, INPUT_PULLUP);  // logic low = fault
    // led
    pinMode(PIN_LED_R, OUTPUT);
    pinMode(PIN_LED_G, OUTPUT);

    init_serial(19200);
    motor.init();

    if (!motor.update_settings()) {
        enter_fault_state();
        Comms.send(REPLY_FAULT, FAULT_DRIVER_SYNC);
        return;
    }

    motor.enable_driver();
    enter_idle_state();
    Comms.send(REPLY_ACK, "Controller Reset");
}

void enter_idle_state() {
    device_mode = Mode::idle;
    idle_time = millis();
    analogWrite(PIN_LED_G, 128);
    analogWrite(PIN_LED_R, 0);
}

void enter_sleep_state() {
    device_mode = Mode::sleep;
    // apply sleep current to motor
    motor.set_current(settings.data.sleep_current);
    analogWrite(PIN_LED_R, 0);
    analogWrite(PIN_LED_G, 128);
}

void enter_moving_state() {
    device_mode = Mode::moving;
    // apply run current to motor
    motor.set_current(settings.data.step_current);
    analogWrite(PIN_LED_G, 255);
    analogWrite(PIN_LED_R, 0);
}

void enter_homing_state() {
    device_mode = Mode::homing;
    motor.set_current(settings.data.step_current);
    analogWrite(PIN_LED_G, 128);
    analogWrite(PIN_LED_R, 128);
    // apply run current to motor
}

void enter_fault_state() {
    device_mode = Mode::fault;
    motor_hard_stop();
    motor.disable_driver();
    analogWrite(PIN_LED_G, 0);
    analogWrite(PIN_LED_R, 255);
}

void motor_goto(byte data[]) {
    memcpy(position.bytes, data + 1, sizeof(position.bytes));
    motor.goto_pos(position.value);
    enter_moving_state();
    Comms.send(REPLY_ACK, position.bytes, sizeof(position.bytes));
}

void motor_home(byte data[]) {
    if (!settings.data.enable_home) {
        Comms.send(REPLY_FAULT, FAULT_HOME);
        enter_fault_state();
        return;
    }
    // Set continuous motion direction based on homing direction
    byte homing_direction = data[1];
    long homing_speed = settings.data.top_speed;
    String msg = "Homing direction: Forward";
    if (!homing_direction) {
        homing_speed *= -1;
        msg = "Homing direction: Backward";
    }
    motor.set_home_speed(homing_speed);
    enter_homing_state();
    Comms.send(REPLY_ACK, msg);
    // limits are checks in the main loop
}

void motor_stop() {
    motor.stop();
    enter_moving_state();
    Comms.send(REPLY_ACK);
}

void motor_hard_stop() {
    motor.reset_position();
}

void controller_update(byte data[], size_t length) {
    // First byte is always the command byte
    if (length - 1 == sizeof(settings.bytes)) {
        // Validate before applying
        Settings_union temp_settings;
        memcpy(temp_settings.bytes, data + 1, sizeof(settings.bytes));
        if (!validate_settings(temp_settings)) {
            Comms.send(REPLY_FAULT, FAULT_INVALID_PARAMETERS);
            return;
        }
        // Apply the settings
        memcpy(settings.bytes, data + 1, sizeof(settings.bytes));
        if (!motor.update_settings()) {
            Comms.send(REPLY_FAULT, FAULT_DRIVER_SYNC);
            enter_fault_state();
            return;
        }
        Comms.send(REPLY_ACK, "Settings updated");
    } else {
        Comms.send(REPLY_FAULT, FAULT_INVALID_PARAMETERS);
    }
}

void controller_echo() {
    Comms.send(REPLY_ACK, REPLY_ECHO);
}

void controller_query(byte data[]) {
    byte query_type = data[1];
    switch (query_type) {
        // TODO: pull from names.c
        case QUERY_MODEL_NO: Comms.send(REPLY_ACK, "Model no: 123"); break;
        case QUERY_SERIAL_NO: Comms.send(REPLY_ACK, "Serial no: 456"); break;
        case QUERY_FIRMWARE: Comms.send(REPLY_ACK, "Firmware: 0.0.1"); break;
        case QUERY_PARAMETERS:
            Comms.send(REPLY_ACK, settings.bytes, sizeof(settings.bytes));
            break;
        default:
            byte fault_data[] = { FAULT_NACK };
            Comms.send(REPLY_FAULT, fault_data, 1);
    }
}

void process_message(byte data[], size_t length) {
    byte cmd = data[0];

    // Check if command allowed in fault state
    if (device_mode == Mode::fault) {
        bool allowed = false;
        for (byte allowed_cmd : FAULT_ALLOWED_CMDS) {
            if (cmd == allowed_cmd) {
                allowed = true;
                break;
            }
        }
        if (!allowed) {
            Comms.send(REPLY_FAULT, FAULT_NACK);
            return;
        }
    }

    switch (cmd) {
        case CMD_GOTO: motor_goto(data); break;
        case CMD_STOP: motor_stop(); break;
        case CMD_HOME: motor_home(data); break;
        case CMD_RESET: reset_controller(); break;
        case CMD_QUERY: controller_query(data); break;
        case CMD_UPDATE_PARAMETERS: controller_update(data, length); break;
        case CMD_ECHO: controller_echo(); break;
        default:
            Comms.send(REPLY_FAULT, FAULT_NACK);
    }
}

void check_sensors() {
    // Check driver fault (active low)
    if (digitalRead(PIN_FAULT) == LOW) {
        enter_fault_state();
        Comms.send(REPLY_FAULT, FAULT_DRIVER);
        return;
    }

    // check home
    if (settings.data.enable_home && device_mode == Mode::homing) {
        if (digitalRead(PIN_HOME) == settings.data.home_sig_polarity) {
            enter_idle_state();
            Comms.send(REPLY_DONE);
        }
    }
    // check lim 1
    if (settings.data.enable_lim1) {
        if (digitalRead(PIN_LIM1) == settings.data.lim1_sig_polarity) {
            enter_fault_state();
            Comms.send(REPLY_FAULT, FAULT_LIMT1);
        }
    }
    // check lim 2
    if (settings.data.enable_lim2) {
        if (digitalRead(PIN_LIM2) == settings.data.lim2_sig_polarity) {
            enter_fault_state();
            Comms.send(REPLY_FAULT, FAULT_LIMT2);
        }
    }
}

void moving() {
    if (motor.steps_remaining() == 0) {
        position.value = motor.position();
        enter_idle_state();
        Comms.send(REPLY_DONE, position.bytes, sizeof(position.bytes));
    } else {
        motor.run();
    }
}

void homing() {
    motor.run_continuous();
}

void idle() {
    if ((millis() - idle_time) > (settings.data.sleep_timeout * 10)) {
        enter_sleep_state();
    }
}

void sleeping() {
    // pulse led geen
    float phase = (float)((millis() % LED_PULSE_PERIOD) * 2 * PI) / LED_PULSE_PERIOD;
    byte brightness = (byte)(128 + 127 * sin(phase));
    analogWrite(PIN_LED_G, brightness);
}

void loop() {
    Comms.run();
    if (Comms.new_data) {
        process_message(Comms.recv_data, Comms.data_length);
        Comms.new_data = false;
    }

    if (device_mode != Mode::fault) {
        check_sensors();
    }

    switch (device_mode) {
        case Mode::moving: moving(); break;
        case Mode::homing: homing(); break;
        case Mode::idle: idle(); break;
        case Mode::sleep: sleeping(); break;
        default: break;
    }
}
