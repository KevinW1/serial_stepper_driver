import struct
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
    FIRMWARE = b"W"
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
    DRIVER_SYNC = b"Y"
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
            "baudrate": 19200,
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

    def receive(self):
        data = self._port.read_until(LINE_END)
        print(f"Received: {data}")
        if not (data.startswith(LINE_BEGIN) and data.endswith(LINE_END)):
            raise IOError(f"Invalid response: {data}")
        return data[1:-1]  # strip line begin/end bytes

    def parse_reply(self, data: bytes):
        reply_code = Reply(data[0:1])  # extract reply code

        if reply_code == Reply.ACK:
            return data[1:]  # strip reply byte

        elif reply_code == Reply.DONE:
            # TODO handle done.
            return data[1:]

        elif reply_code == Reply.FAULT:
            fault = Fault(data[1:2]).name  # second byte is fault-type
            print(f"Fault: {fault}")

    def send_command(self, command: Cmd, arg_bytes: bytes = None):
        print(f"Sending: {command.name}")
        message = LINE_BEGIN
        message += command.value
        if arg_bytes:
            message += arg_bytes
        message += LINE_END
        self._port.write(message)
        return self.parse_reply(self.receive())

    def reset(self):
        result = self.send_command(Cmd.RESET)
        return result

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
        reply = self.send_command(Cmd.GOTO, distance)
        return reply

    def stop(self):
        self.send_command(Cmd.STOP)
        # TODO: needs to wait for a longer timeout
        # because of the decel time
        data = self.receive()
        reply_code = Reply(data[0:1])
        if reply_code == Reply.DONE:
            return struct.unpack("<L", data[1:])[0]

    def home(self, direction: bool):
        direction = direction.to_bytes(length=1, byteorder="little")
        reply = self.send_command(Cmd.HOME, direction)
        return reply


# Example Usage:
if __name__ == "__main__":
    print("start")
    mc = VMSTEP(port="/dev/ttyACM0")
    print("object made")
    # Example commands
    # status = mc.echo()
    # status = mc.query(Query.SERIAL_NO)
    # status = mc.home(True)
    # status = mc.query(Query.FAULTS)

    settings = Settings(
        step_current=1,
        sleep_current=0,
        microstep_resolution=7,
        sleep_timeout=100,
        top_speed=8000,
        acceleration=4000,
        enable_lim1=False,
        enable_lim2=False,
        enable_home=True,
        lim1_sig_polarity=False,
        lim2_sig_polarity=False,
        home_sig_polarity=False,
    )

    # status = mc.reset()
    # print("Reply: ", status)

    status = mc.set_parameters(settings)
    print("Reply: ", status)

    status = mc.get_parameters()
    print("Reply: ", status)

    status = mc.goto(1600)
    print("Reply: ", status)

    # sleep(1)
    # status = mc.stop()
    # print(status)

    # status = mc.home(True)
    # print("Reply: ", status)

    mc.close()
