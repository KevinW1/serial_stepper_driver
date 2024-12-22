import struct
import time
from collections.abc import Mapping
from enum import Enum
from time import sleep
from types import MappingProxyType

import dataclasses_struct as dcs
import numpy as np
import serial
from serial import SerialException
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
    ENABLE = b"Y"
    DISABLE = b"X"
    RESET_POSITION = b"Z"  # New command


class Query(Enum):
    MODEL_NO = b"M"
    SERIAL_NO = b"S"
    FIRMWARE = b"W"
    PARAMETERS = b"P"
    FAULTS = b"F"
    POSITION = b"X"
    MODE = b"T"


class DeviceMode(Enum):
    """Maps to the Mode enum in the Arduino code"""

    IDLE = 0
    SLEEP = 1
    MOVING = 2
    HOMING = 3
    FAULT = 4


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
    top_speed: dcs.U32
    acceleration: dcs.U32
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
        self._clear_buffer()

    def close(self):
        self._port.close()

    def _clear_buffer(self):
        self._port.read_all()

    def receive(self):
        data = self._port.read_until(LINE_END)
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

    def enable(self):
        results = self.send_command(Cmd.ENABLE)
        return results

    def disable(self):
        results = self.send_command(Cmd.DISABLE)
        return results

    def get_parameters(self):
        reply = self.query(Query.PARAMETERS)
        return Settings.from_packed(reply)

    def set_parameters(self, settings: Settings):
        results = self.send_command(Cmd.UPDATE_PARAMETERS, settings.pack())
        return results

    def goto(self, distance: int, timeout_s: float = 10.0):
        distance = distance.to_bytes(length=4, byteorder="little", signed=True)
        self.send_command(Cmd.GOTO, distance)
        reply = self.wait_for_done(timeout_s)
        return struct.unpack("<l", reply[1:])[0]

    def wait_for_done(self, timeout_s: float = 10.0):
        start_time = time.time()
        while True:
            if time.time() - start_time > timeout_s:
                raise TimeoutError("Motor operation timed out")
            try:
                data = self.receive()
                reply_code = Reply(data[0:1])
                if reply_code == Reply.DONE:
                    return data
            except (SerialException, ValueError, OSError):
                # Brief pause before retry
                time.sleep(0.1)
                continue

    def stop(self, timeout_s=10):
        self.send_command(Cmd.STOP)
        self.wait_for_done(timeout_s)

    def home(self, direction: bool, timeout_s=10):
        direction = direction.to_bytes(length=1, byteorder="little")
        self.send_command(Cmd.HOME, direction)
        self.wait_for_done(timeout_s)
        # r
        return "Homeing Done"

    def get_position(self) -> int:
        """Query current motor position. Returns position in steps."""
        reply = self.query(Query.POSITION)
        return struct.unpack("<l", reply)[0]  # Unpack as 32-bit signed integer

    def get_mode(self) -> DeviceMode:
        """
        Query current device mode.
        Returns:
            DeviceMode enum representing the current state
        """
        reply = self.query(Query.MODE)
        return DeviceMode(reply[0])

    def reset_position(self):
        """Reset the current position to zero"""
        return self.send_command(Cmd.RESET_POSITION)


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
        step_current=0,
        sleep_current=0,
        microstep_resolution=7,
        sleep_timeout=100,
        top_speed=16000,
        acceleration=120000,
        enable_lim1=False,
        enable_lim2=False,
        enable_home=True,
        lim1_sig_polarity=False,
        lim2_sig_polarity=False,
        home_sig_polarity=False,
    )

    # status = mc.reset()
    # print("Reply: ", status)

    # status = mc.get_position()
    # print("Reply: ", status)

    status = mc.set_parameters(settings)
    print("Reply: ", status)

    # status = mc.get_parameters()
    # print("Reply: ", status)

    status = mc.enable()

    # status = mc.goto(-250)
    # print("Reply: ", status)

    # status = mc.disable()
    # print("Reply: ", status)

    # sleep(1)
    # status = mc.stop()
    # print(status)

    status = mc.home(False)
    status = mc.goto(-750)
    status = mc.disable()

    status = mc.get_position()
    print("Position: ", status)

    mc.reset_position()

    status = mc.get_position()
    print("Position: ", status)

    mc.close()
