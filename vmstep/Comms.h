#ifndef Comms_h
#define Comms_h

#include <Arduino.h>

// flow control
static constexpr uint32_t BAUD_RATE = 19200;
static constexpr char START_MARKER = '[';
static constexpr char END_MARKER = ']';

// serial commands
static constexpr char CMD_GOTO = 'G';
static constexpr char CMD_STOP = 'S';
static constexpr char CMD_HOME = 'H';
static constexpr char CMD_RESET = 'R';
static constexpr char CMD_QUERY = 'Q';
static constexpr char CMD_UPDATE_PARAMETERS = 'P';
static constexpr char CMD_ECHO = 'E';
static constexpr char CMD_ENABLE = 'Y';
static constexpr char CMD_DISABLE = 'X';
static constexpr char CMD_RESET_POSITION = 'Z';

// commads allowed in fault state
static constexpr byte FAULT_ALLOWED_CMDS[] = {
    CMD_RESET,
    CMD_QUERY,
    CMD_ECHO
};

// Query
static constexpr char QUERY_MODEL_NO = 'M';
static constexpr char QUERY_SERIAL_NO = 'S';
static constexpr char QUERY_FIRMWARE = 'W';
static constexpr char QUERY_PARAMETERS = 'P';
static constexpr char QUERY_FAULTS = 'F';
static constexpr char QUERY_POSITION = 'X';
static constexpr char QUERY_MODE = 'T';
static constexpr char QUERY_FAULT_REGS = 'R';

// Replies
static constexpr char REPLY_ACK = 'A';
static constexpr char REPLY_DONE = 'D';
static constexpr char REPLY_FAULT = 'F';
static constexpr char REPLY_ECHO = 'E';

// Faults
static constexpr char FAULT_NACK = 'N';
static constexpr char FAULT_DRIVER = 'D';
static constexpr char FAULT_INVALID_PARAMETERS = 'P';
static constexpr char FAULT_DRIVER_SYNC = 'Y';
static constexpr char FAULT_LIMT1 = 'L';
static constexpr char FAULT_LIMT2 = 'K';
static constexpr char FAULT_HOME = 'H';

void init_serial();

typedef void (*SerialHandlerCallback)(byte msg[], size_t size);

class SerialTransciever {
  public:
    static constexpr size_t BUFFER_SIZE = 32u;
    void send(byte reply_code, const byte msg[], size_t length);
    void send(byte reply_code, byte data);
    void send(byte reply_code, String msg);
    void send(byte reply_code);

    // Call every main loop
    //
    // The given callback is executed when a whole message has been received. Accepts callbacks of the form:
    //     (byte[] msg, size_t size)
    // The contents of "msg" is ONLY VALID DURING THE CALLBACK. Consider it invalid after returning
    // The callback will not be run if there is no message pending.
    void run(SerialHandlerCallback callback);

  private:
    // Only valid when;
    //   receiver_state == MSG_PENDING
    //   data_length > 0
    byte data[BUFFER_SIZE] = {};
    size_t data_size = 0;

    // Simple state machine for handling the start/end tokens in the serial stream.
    enum class ReceiverState : uint8_t {
        // The "data" buffer should be considered invalid/incomplete in these states.
        MSG_WAITING = 0,  // data is empty and we're waiting for a start marker
        //  v
        // run() transitions from MSG_WAITING to MSG_READING upon receiving a start marker
        //  v
        MSG_READING,
        //  v
        // run() transitions from MSG_READING to MSG_PENDING upon receiving an end marker
        // run() transitions from MSG_READING to MSG_WAITING if a fault occurs
        //  v
        MSG_PENDING  // Indicates that a new, complete message is pending for consumption in the "data" buffer.
        //  v
        // run() transitions from MSG_PENDING back to MSG_WAITING
    };
    ReceiverState receiver_state = ReceiverState::MSG_WAITING;

    // Attempts to receive a single message from serial, and noops until the read message is no longer pending.
    void recieve();
    void reset();
};

#endif