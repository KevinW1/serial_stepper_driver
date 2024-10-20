#ifndef Comms_h
#define Comms_h

#include <arduino.h>

// serial commands
const uint8_t CMD_GOTO = 'G';
const uint8_t CMD_STOP = 'S';
const uint8_t CMD_HOME = 'H';
const uint8_t CMD_RESET = 'R';
const uint8_t CMD_QUERY = 'Q';
const uint8_t CMD_UPDATE_PARAMETERS = 'P';
const uint8_t CMD_ECHO = 'E';
// Query
const uint8_t QUERY_MODEL_NO = 'M';
const uint8_t QUERY_SERIAL_NO = 'S';
const uint8_t QUERY_PARAMETERS = 'P';
const uint8_t QUERY_FAULTS = 'F';
// Replies
const uint8_t REPLY_ACK = 'A';
const uint8_t REPLY_DONE = 'D';
const uint8_t REPLY_FAULT = 'F';
const uint8_t REPLY_ECHO = 'E';
// Faults
const uint8_t FAULT_NACK = 'N';
const uint8_t FAULT_DRIVER = 'D';
const uint8_t FAULT_INVALID_PARAMETERS = 'P';
const uint8_t FAULT_LIMT1 = 'L';
const uint8_t FAULT_LIMT2 = 'K';
const uint8_t FAULT_HOME = 'H';

void init_serial(int);

class SerialTransciever {
  public:
    static const byte buffer_size = 32;
    byte recv_data[buffer_size];
    boolean recv_in_progress;
    boolean new_data = false;
    size_t data_length;
    SerialTransciever();
    void send(byte reply_code, byte msg[], size_t length);
    void send(byte reply_code, byte data);
    void send(byte reply_code, String msg);
    void send(byte reply_code);
    void recieve();
    void run();

  private:
    const byte start_marker = '[';
    const byte end_marker = ']';
    byte cursor;
    byte incoming_byte;
    void reset();
};

#endif