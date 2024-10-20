#include <arduino.h>
#include "Comms.h"

void init_serial(int baud) {
    Serial.begin(baud);
    // Serial.clear();  //Teensy only
}


SerialTransciever::SerialTransciever() {
    reset();
}

void SerialTransciever::reset() {
    recv_in_progress = false;
    cursor = 0;
}

void SerialTransciever::send(byte reply_code, byte msg[], size_t length) {
    Serial.write(start_marker);
    Serial.write(reply_code);
    Serial.write(msg, length);
    Serial.write(end_marker);
}

void SerialTransciever::send(byte reply_code, byte data) {
    Serial.write(start_marker);
    Serial.write(reply_code);
    Serial.write(data);
    Serial.write(end_marker);
}

void SerialTransciever::send(byte reply_code) {
    Serial.write(start_marker);
    Serial.write(reply_code);
    Serial.write(end_marker);
}

void SerialTransciever::send(byte reply_code, String msg) {
    byte buffer[msg.length()];  // not null terminated
    msg.getBytes(buffer, msg.length() + 1);
    send(reply_code, buffer, sizeof(buffer));
}


void SerialTransciever::recieve() {
    while (Serial.available() > 0 && new_data == false) {
        incoming_byte = Serial.read();

        if (recv_in_progress == true) {

            // data done, roll it up!
            if (incoming_byte == end_marker) {
                // recv_data[cursor] = '\0';  // null terminate the string
                new_data = true;
                data_length = cursor;
                reset();

                // previous data fucked, start reading the new message
            } else if (incoming_byte == start_marker) {
                new_data = false;
                cursor = 0;
                Serial.println("You sent double start markers! bad!");

                // normal data recieve
            } else {
                if (cursor + 1 >= buffer_size) {
                    // too many chars
                    reset();
                    new_data = false;
                    Serial.println("Too many characters bro!");
                    break;
                }
                recv_data[cursor] = incoming_byte;
                cursor++;
            }
        } else if (incoming_byte == start_marker) {
            recv_in_progress = true;
        }
        // no start marker and not in progress
    }
}

void SerialTransciever::run() {
    recieve();
}
