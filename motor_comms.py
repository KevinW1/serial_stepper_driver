import time
from collections.abc import Mapping
from enum import Enum
from time import sleep
from types import MappingProxyType

import dataclasses_struct as dcs
import numpy as np
import serial
from serial.tools import list_ports

LINE_BEGIN = b"["
LINE_END = b"]"


class Cmd(Enum):
    GOTO = b"G"
    STOP = b"S"
    HOME = b"H"
    RESET = b"R"
    QUERY = b"Q"
    UPDATE_PARAMETERS = b"P"
    ECHO = b"E"


class Query(Enum):
    MODEL_NO = b"M"
    SERIAL_NO = b"S"
    PARAMETERS = b"P"
    FAULTS = b"F"


class Reply(Enum):
    ACK = b"A"
    DONE = b"D"
    FAULT = b"F"
    ECHO = b"E"


class Fault(Enum):
    NACK = b"N"
    DRIVER = b"D"
    INVALID_PARAMETERS = b"P"
    LIMT1 = b"L"
    LIMT2 = b"K"
    HOME = b"H"


@dcs.dataclass(dcs.LITTLE_ENDIAN)
class Settings:
    # motor
    step_current: dcs.U8
    sleep_current: dcs.U8
    microstep_resolution: dcs.U8
    sleep_timeout: dcs.U8
    # trajectory
    top_speed: dcs.U16
    acceleration: dcs.U16
    # lims
    enable_lim1: dcs.Bool
    enable_lim2: dcs.Bool
    enable_home: dcs.Bool
    lim1_sig_polarity: dcs.Bool
    lim2_sig_polarity: dcs.Bool
    home_sig_polarity: dcs.Bool


class VMSTEP:
    VMSTEP_SERIAL_KWARGS: Mapping = MappingProxyType(
        {
            "baudrate": 9600,
            "parity": serial.PARITY_NONE,
            "stopbits": serial.STOPBITS_ONE,
            "bytesize": serial.EIGHTBITS,
            "timeout": 0.5,
        }
    )

    def __init__(self, port=None):
        if isinstance(port, str):
            self._port = serial.Serial(port, **self.VMSTEP_SERIAL_KWARGS)
        else:
            self._port = port
        sleep(2)  # wait for the arduino to reset, boooo
        self._clear_buffer()

    def close(self):
        self._port.close()

    def _clear_buffer(self):
        self._port.read_all()

    def recieve(self):
        data = self._port.read_until(LINE_END)
        if not (data.startswith(LINE_BEGIN) and data.endswith(LINE_END)):
            raise IOError(f"Invalid response: {data}")
        data = data[1:-1]  # strip line begin/end bytes
        reply_code = Reply(data[0:1])  # extract reply code

        if reply_code == Reply.ACK:
            return data[1:]  # strip reply byte

        elif reply_code == Reply.DONE:
            # TODO handle done.
            return data[1:]

        elif reply_code == Reply.FAULT:
            fault = Fault(data[1:2]).name  # second byte is fault-type
            print(f"Fault: {fault}")

    def send_command(self, command: int, arg_bytes: bytes = None):
        message = LINE_BEGIN
        message += command.value
        if arg_bytes:
            message += arg_bytes
        message += LINE_END
        self._port.write(message)
        return self.recieve()

    def reset(self):
        self.send_command(Cmd.RESET)

    def echo(self):
        results = self.send_command(Cmd.ECHO)
        return results

    def query(self, query: Query):
        results = self.send_command(Cmd.QUERY, query.value)
        return results

    def get_parameters(self):
        reply = self.query(Query.PARAMETERS)
        return Settings.from_packed(reply)

    def set_parameters(self, settings: Settings):
        results = self.send_command(Cmd.UPDATE_PARAMETERS, settings.pack())
        return results

    def goto(self, distance: int):
        distance = distance.to_bytes(length=4, byteorder="little", signed=True)
        print("distance", distance)
        reply = self.send_command(Cmd.GOTO, distance)
        return reply

    def home(self, direction: bool):
        direction = direction.to_bytes(length=1, byteorder="little")
        print("direction", direction)
        reply = self.send_command(Cmd.HOME, direction)
        return reply


# Example Usage:
if __name__ == "__main__":
    print("start")
    mc = VMSTEP(port="COM3")
    print("object made")
    # Example commands
    # status = mc.echo()
    # status = mc.query(Query.SERIAL_NO)
    # status = mc.home(True)
    # status = mc.query(Query.FAULTS)

    status = mc.set_parameters(
        Settings(11, 15, 7, 100, 1, 255, False, False, False, False, True, True)
    )
    print("Reply: ", status)

    status = mc.get_parameters()
    print("Reply: ", status)

    status = mc.goto(1700)
    print("Reply: ", status)
    mc.close()
