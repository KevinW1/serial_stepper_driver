#include "Comms.h"

void init_serial() {
    Serial.begin(BAUD_RATE);
    Serial.clear();  //Teensy only
}

void SerialTransciever::send(byte reply_code, byte msg[], size_t length) {
    Serial.write(START_MARKER);
    Serial.write(reply_code);
    if (length > 0 && msg != nullptr) {
        // TODO: what happens if msg contains END_MARKER?
        Serial.write(msg, length);
    }
    Serial.write(END_MARKER);
}

void SerialTransciever::send(byte reply_code, byte data) {
    send(reply_code, &data, 1u);
}

void SerialTransciever::send(byte reply_code) {
    send(reply_code, nullptr, 0u);
}

void SerialTransciever::send(byte reply_code, String msg) {
    // ignore null terminator
    send(reply_code, msg.c_str(), msg.length());
}

void SerialTransciever::recieve() {
    int incoming_byte = -1;
    while ((receiver_state != ReceiverState::MSG_PENDING) &&
           (incoming_byte = Serial.read()) != -1) {
        // State maching handling start/end markers
        switch (receiver_state) {
            case ReceiverState::MSG_WAITING: {
                if (incoming_byte == START_MARKER) {
                    receiver_state = ReceiverState::MSG_READING;
                } else {
                    // Anything else is a fault, or eating characters to recover from a fault
                    // (to reach the next message in the stream).
                    reset();
                }
                break;
            }
            case ReceiverState::MSG_READING: {
                if (incoming_byte == START_MARKER) {
                    Serial.println("You sent double start markers! bad!");
                    reset();
                } else if (incoming_byte == END_MARKER) {
                    // data done, allow it to be consumed by the user
                    if (data_size == 0) {
                        // No payload received. Go back to the WAITING state
                        Serial.println("Received empty message payload!");
                        reset();
                    } else {
                        // Have a payload. Mark it as pending user consumption
                        receiver_state = ReceiverState::MSG_PENDING;
                    }
                } else if (data_size + 1 >= BUFFER_SIZE) {
                    // too many chars
                    Serial.println("Too many characters bro!");
                    reset();
                } else {
                    // copy bytes into the data buffer
                    data[data_size] = incoming_byte;
                    data_size++;
                }
                break;
            }
            case ReceiverState::MSG_PENDING: {
                break;  // This function does nothing when in the pending state (handled in loop above)
            }
        }
    }
}

void SerialTransciever::reset() {
    data_size = 0;
    receiver_state = ReceiverState::MSG_WAITING;
}

void SerialTransciever::run(SerialHandlerCallback callback) {
    recieve();
    if (receiver_state == ReceiverState::MSG_PENDING) {
        callback(data, data_size);
        reset();
    }
}
