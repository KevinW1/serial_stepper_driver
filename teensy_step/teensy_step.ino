#include "Comms.h"
#include "Settings.h"
#include "Motor.h"
#include <EEPROM.h>

// pin definitions
// Motor
const byte PIN_SLEEP = 6;
const byte PIN_STEP = 7;
const byte PIN_DIR = 8;
const byte PIN_ENABLE = 9;
const byte PIN_SCS = 10;
const byte PIN_FAULT = 17;
// system
const byte PIN_LED_R = 4;
const byte PIN_LED_G = 5;
// Sensors
const byte PIN_HOME = 14;
const byte PIN_LIM2 = 15;
const byte PIN_LIM1 = 16;

// TODO: Store this in eprom memory, maybe?
Settings_union settings = {
    0b1111,  // run current (inverse, 1111 is min)
    0b1111,  // sleep current
    0b0111,  // microstep res
    0x64,    // sleep timeout
    0x07D0,  // top speed 2000
    0x4E20,  // accel 20,000
    0b0,     // enable lim1
    0b0,     // enable lim2
    0b0,     // enable home
    0b1,     // lim1 pol
    0b0,     // lim2 pol
    0b1,     // home pol
};


union { long value; uint8_t bytes[4]; } long_union;

SerialTransciever Comms;
Motor motor(PIN_SCS, PIN_STEP, PIN_DIR, settings);

void setup() {
    init_serial(9600);
    reset_controller();
}

void reset_controller() {
    motor.init();
    motor.update_settings();
    // Limits:init();
}

void motor_goto(byte data[]) {
    memcpy(long_union.bytes, data + 1, sizeof(long_union.bytes));
    motor.goto_pos(long_union.value);
    Comms.send(REPLY_ACK, long_union.bytes, sizeof(long_union.bytes));
}

void motor_home() {
    Comms.send(REPLY_ACK);
    // run homing procedure
    // reply done
    Comms.send(REPLY_DONE);
}

void motor_home(byte data[]) {
    byte direction = data[1];
    Comms.send(REPLY_ACK, direction);
}

void motor_stop() {
    motor.stop();
    Comms.send(REPLY_ACK);
}

void controller_update(byte data[], size_t length) {
    // TODO check limits on each datafield
    // First byte is always the command byte
    if (length - 1 == sizeof(settings.bytes)) {
        memcpy(settings.bytes, data + 1, sizeof(settings.bytes));
        motor.update_settings();
        Comms.send(REPLY_ACK, "Settings updated");
    } else {
        byte fault_data[] = { FAULT_INVALID_PARAMETERS };
        Comms.send(REPLY_FAULT, fault_data, 1);
    }
}

void controller_echo() {
    Comms.send(REPLY_ACK);
}

void controller_query(byte data[]) {
    byte query_type = data[1];
    switch (query_type) {
        case QUERY_MODEL_NO: Comms.send(REPLY_ACK, "Model no: 123"); break;
        case QUERY_SERIAL_NO: Comms.send(REPLY_ACK, "Serial no: 456"); break;
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
    switch (cmd) {
        case CMD_GOTO: motor_goto(data); break;
        case CMD_STOP: motor_stop(); break;
        case CMD_HOME: motor_home(data); break;
        case CMD_RESET: reset_controller(); break;
        case CMD_QUERY: controller_query(data); break;
        case CMD_UPDATE_PARAMETERS: controller_update(data, length); break;
        case CMD_ECHO: controller_echo(); break;
        default:
            byte fault_data[] = { FAULT_NACK };
            Comms.send(REPLY_FAULT, fault_data, 1);
    }
}

void loop() {
    Comms.run();
    if (Comms.new_data == true) {
        process_message(Comms.recv_data, Comms.data_length);
        Comms.new_data = false;
    }
    motor.run();
    // limits.check();
}
