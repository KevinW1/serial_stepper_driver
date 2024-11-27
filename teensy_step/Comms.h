#ifndef Comms_h
#define Comms_h

#include <Arduino.h>

// serial commands
static constexpr char CMD_GOTO = 'G';
static constexpr char CMD_STOP = 'S';
static constexpr char CMD_HOME = 'H';
static constexpr char CMD_RESET = 'R';
static constexpr char CMD_QUERY = 'Q';
static constexpr char CMD_UPDATE_PARAMETERS = 'P';
static constexpr char CMD_ECHO = 'E';
// Query
static constexpr char QUERY_MODEL_NO = 'M';
static constexpr char QUERY_SERIAL_NO = 'S';
static constexpr char QUERY_PARAMETERS = 'P';
static constexpr char QUERY_FAULTS = 'F';
// Replies
static constexpr char REPLY_ACK = 'A';
static constexpr char REPLY_DONE = 'D';
static constexpr char REPLY_FAULT = 'F';
static constexpr char REPLY_ECHO = 'E';
// Faults
static constexpr char FAULT_NACK = 'N';
static constexpr char FAULT_DRIVER = 'D';
static constexpr char FAULT_INVALID_PARAMETERS = 'P';
static constexpr char FAULT_LIMT1 = 'L';
static constexpr char FAULT_LIMT2 = 'K';
static constexpr char FAULT_HOME = 'H';

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