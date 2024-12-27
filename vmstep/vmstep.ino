#include "Comms.h"
#include "Settings.h"
#include "Motor.h"
#include "name.h"
#include <limits.h>

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


enum class Mode {
    idle,    // waiting for commands, full current
    sleep,   // waiting for commands, low current
    moving,  // moving to a position
    homing,  // moving, waiting for home sensor
    fault    // Stop all operations
};
Mode device_mode = Mode::idle;

unsigned long idle_time = 0;  // time controler entered idle state

union {
    long value;
    uint8_t bytes[4];
} position;

Settings_union settings;
SerialTransciever Comms;
Motor motor(PIN_SCS, PIN_STEP, PIN_DIR, PIN_ENABLE, PIN_SLEEP, settings);

void setup() {
    reset_controller();
}

void reset_controller() {
    init_serial();
    // Check if settings are valid
    if (!validate_settings(settings)) {
        Comms.send(REPLY_FAULT, FAULT_INVALID_PARAMETERS);
        enter_fault_state();
        return;
    }

    // sensors
    pinMode(PIN_LIM1, INPUT_PULLUP);
    pinMode(PIN_LIM2, INPUT_PULLUP);
    pinMode(PIN_HOME, INPUT_PULLUP);
    // driver
    pinMode(PIN_FAULT, INPUT_PULLUP);  // logic low = fault
    // led
    pinMode(PIN_LED_R, OUTPUT);
    pinMode(PIN_LED_G, OUTPUT);

    motor.init();

    // Check communication with driver
    if (!motor.update_settings()) {
        enter_fault_state();
        Comms.send(REPLY_FAULT, FAULT_DRIVER_SYNC);
        return;
    }

    enter_idle_state();
    Comms.send(REPLY_ACK);
}

// State control functions
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
    motor.set_current(settings.data.run_current);
    analogWrite(PIN_LED_G, 255);
    analogWrite(PIN_LED_R, 0);
}

void enter_homing_state() {
    device_mode = Mode::homing;
    // apply run current to motor
    motor.set_current(settings.data.run_current);
    analogWrite(PIN_LED_G, 128);
    analogWrite(PIN_LED_R, 128);
}

void enter_fault_state() {
    device_mode = Mode::fault;
    motor_hard_stop();
    motor.disable_driver();
    analogWrite(PIN_LED_G, 0);
    analogWrite(PIN_LED_R, 255);
}

// Motor command functions
void motor_goto(byte data[]) {
    memcpy(position.bytes, data + 1, sizeof(position.bytes));
    motor.goto_pos(position.value);
    enter_moving_state();
    Comms.send(REPLY_ACK, position.bytes, sizeof(position.bytes));
}

void motor_home(byte data[]) {
    if (!settings.data.flags.enable_home) {
        Comms.send(REPLY_FAULT, FAULT_HOME);
        enter_fault_state();
        return;
    }
    // Set homing direction from byte 1
    long homing_pos = LONG_MAX;
    if (!data[1]) {
        homing_pos = -LONG_MAX;
    }
    motor.goto_pos(homing_pos);
    enter_homing_state();
    Comms.send(REPLY_ACK);
    // limits are checks in the main loop
}

void motor_stop() {
    motor.stop();
    enter_moving_state();  //decel
    Comms.send(REPLY_ACK);
}

void motor_hard_stop() {
    motor.hard_stop();
    enter_idle_state();
    Comms.send(REPLY_ACK);
}

void motor_enable() {
    motor.enable_driver();
    enter_idle_state();
    Comms.send(REPLY_ACK);
}

void motor_disable() {
    motor.disable_driver();
    enter_idle_state();
    Comms.send(REPLY_ACK);
}

void motor_reset_position() {
    if (device_mode != Mode::idle && device_mode != Mode::sleep) {
        Comms.send(REPLY_FAULT, FAULT_NACK);
        return;
    }
    motor.reset_position();
    position.value = 0;  // Reset our position tracking
    enter_idle_state();
    Comms.send(REPLY_ACK);
}

// Controller functions

void controller_echo() {
    Comms.send(REPLY_ACK, REPLY_ECHO);
}

void update_parameters(byte data[], size_t length) {
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

void controller_query(byte data[]) {
    byte query_type = data[1];
    switch (query_type) {
        case QUERY_MODEL_NO:
            {
                // Create static array from macro
                static const char product[] = PRODUCT_NAME;
                char model[PRODUCT_NAME_LEN + 1];  // +1 for null terminator
                memcpy(model, product, PRODUCT_NAME_LEN);
                model[PRODUCT_NAME_LEN] = '\0';
                Comms.send(REPLY_ACK, model);
                break;
            }
        case QUERY_SERIAL_NO:
            {
                // Create static array from macro
                static const char serial_num[] = SERIAL_NUMBER;
                char serial[SERIAL_NUMBER_LEN + 1];  // +1 for null terminator
                memcpy(serial, serial_num, SERIAL_NUMBER_LEN);
                serial[SERIAL_NUMBER_LEN] = '\0';
                Comms.send(REPLY_ACK, serial);
                break;
            }
        case QUERY_FIRMWARE: Comms.send(REPLY_ACK, "Firmware: 0.0.1"); break;
        case QUERY_PARAMETERS:
            Comms.send(REPLY_ACK, settings.bytes, sizeof(settings.bytes));
            break;
        case QUERY_POSITION:
            {
                position.value = motor.position();  // Update position from motor
                Comms.send(REPLY_ACK, position.bytes, sizeof(position.bytes));
                break;
            }
        case QUERY_MODE:
            {
                byte mode = static_cast<byte>(device_mode);
                Comms.send(REPLY_ACK, &mode, 1);
                break;
            }
        case QUERY_FAULT_REGS:
            {
                const std::array<byte, 3>& fault_registers = motor.get_fault_registers();
                Comms.send(REPLY_ACK, fault_registers.data(), 3);  // Send all 3 registers
                break;
            }

        default:
            Comms.send(REPLY_FAULT, FAULT_NACK);
    }
}

bool fault_allowed_command(byte cmd) {
    switch (cmd) {
        case CMD_RESET: [[fallthrough]];
        case CMD_QUERY: [[fallthrough]];
        case CMD_ECHO:
            return true;
        default:
            return false;
    }
}

void process_message(byte data[], size_t length) {
    byte cmd = data[0];

    // Check if command allowed in fault state
    if (device_mode == Mode::fault && !fault_allowed_command(cmd)) {
        Comms.send(REPLY_FAULT, FAULT_NACK);
        return;
    }

    switch (cmd) {
        case CMD_GOTO: motor_goto(data); break;
        case CMD_STOP: motor_stop(); break;
        case CMD_HOME: motor_home(data); break;
        case CMD_RESET: reset_controller(); break;
        case CMD_QUERY: controller_query(data); break;
        case CMD_UPDATE_PARAMETERS: update_parameters(data, length); break;
        case CMD_ECHO: controller_echo(); break;
        case CMD_ENABLE: motor_enable(); break;
        case CMD_DISABLE: motor_disable(); break;
        case CMD_RESET_POSITION: motor_reset_position(); break;
        default:
            Comms.send(REPLY_FAULT, FAULT_NACK);
    }
}

void check_sensors() {
    // Check driver fault (active low)
    if (digitalRead(PIN_FAULT) == LOW && device_mode != Mode::fault) {
        // Create fault report array: [FAULT_DRIVER, fault_reg, diag1_reg, diag2_reg]
        byte fault_data[4] = { FAULT_DRIVER };

        const std::array<byte, 3>& fault_registers = motor.get_fault_registers();
        memcpy(fault_data + 1, fault_registers.data(), 3);

        enter_fault_state();
        Comms.send(REPLY_FAULT, fault_data, sizeof(fault_data));
        return;
    }

    // check home
    if (settings.data.flags.enable_home && device_mode == Mode::homing) {
        if (digitalRead(PIN_HOME) == settings.data.flags.home_sig_polarity) {
            enter_idle_state();
            motor.reset_position();
            Comms.send(REPLY_DONE);
        }
    }
    // check lim 1
    if (settings.data.flags.enable_lim1) {
        if (digitalRead(PIN_LIM1) == settings.data.flags.lim1_sig_polarity) {
            enter_fault_state();
            Comms.send(REPLY_FAULT, FAULT_LIMT1);
        }
    }
    // check lim 2
    if (settings.data.flags.enable_lim2) {
        if (digitalRead(PIN_LIM2) == settings.data.flags.lim2_sig_polarity) {
            enter_fault_state();
            Comms.send(REPLY_FAULT, FAULT_LIMT2);
        }
    }
}

// Loop activities

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
    if (motor.steps_remaining() == 0) {
        // home distance exceedd max position value.
        enter_fault_state();
        Comms.send(REPLY_FAULT, FAULT_HOME);
    } else {
        motor.run();
    }
}

void loop() {
    Comms.run(process_message);

    check_sensors();

    switch (device_mode) {
        case Mode::idle: idle(); break;
        case Mode::sleep: sleeping(); break;
        case Mode::moving: moving(); break;
        case Mode::homing: homing(); break;
        default: break;
    }
}
