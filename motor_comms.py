from __future__ import annotations

import logging
import struct
import time
from collections.abc import Mapping
from enum import Enum
from types import MappingProxyType, TracebackType
from typing import Optional, Type

import dataclasses_struct as dcs
import serial
from serial import SerialException
from serial.tools.list_ports import comports

logger = logging.getLogger(__name__)

MANUFACTURER = "VIRTUALMATTER"
PRODUCT = "VMSTEP"

LINE_BEGIN = b"["
LINE_END = b"]"

FAULT_MESSAGES = {
    7: "FAULT pin active",
    6: "SPI protocol error",
    5: "Supply undervoltage lockout",
    4: "Charge pump undervoltage",
    3: "Overcurrent",
    2: "Motor stall",
    1: "Thermal flag",
    0: "Open load",
}

DIAG1_MESSAGES = {
    7: "Overcurrent on BOUT low-side FET 2",
    6: "Overcurrent on BOUT high-side FET 2",
    5: "Overcurrent on BOUT low-side FET 1",
    4: "Overcurrent on BOUT high-side FET 1",
    3: "Overcurrent on AOUT low-side FET 2",
    2: "Overcurrent on AOUT high-side FET 2",
    1: "Overcurrent on AOUT low-side FET 1",
    0: "Overcurrent on AOUT high-side FET 1",
}

DIAG2_MESSAGES = {
    6: "Overtemperature warning",
    5: "Overtemperature shutdown",
    4: "Stall detection learning successful",
    3: "Motor stall detected",
    1: "Open load on BOUT",
    0: "Open load on AOUT",
}


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
    RESET_POSITION = b"Z"


class Query(Enum):
    MODEL_NO = b"M"
    SERIAL_NO = b"S"
    FIRMWARE = b"W"
    PARAMETERS = b"P"
    FAULTS = b"F"
    POSITION = b"X"
    MODE = b"T"
    FAULT_REGS = b"R"


class DeviceMode(Enum):
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

    @classmethod
    def discover(self, serial_number=None) -> "VMSTEP":
        """Attempt automatic discovery of the VMSTEP serial port
        and return the VMSTEP object.

        Returns
        -------
        VMSTEP
            A successfully located VMSTEP object.

        Raises
        ------
        serial.SerialException
            If no serial port can be automatically linked.
        """

        port_list = comports()

        if len(port_list) == 0:
            raise serial.SerialException("No serial ports found on this machine")

        for p in port_list:
            if p.manufacturer != MANUFACTURER and p.product != PRODUCT:
                continue

            if not serial_number or p.serial_number == serial_number:
                serial_port = serial.Serial(p.device, **self.VMSTEP_SERIAL_KWARGS)
                serial_port.read_all()
                return VMSTEP(serial_port)
            else:
                continue

        raise serial.SerialException(
            """Could not connect to any devices.
                Check connection and power."""
        )

    def __init__(self, serial_port=None):
        self.logger = logging.getLogger(__name__)
        if isinstance(serial_port, str):
            self._port = serial.Serial(serial_port, **self.VMSTEP_SERIAL_KWARGS)
        else:
            self._port = serial_port

    def __enter__(self) -> Self:
        self.open()
        return self

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_val: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ) -> bool:
        self.close()
        return False

    def _clear_buffer(self):
        self._port.read_all()

    def open(self):
        if isinstance(self._port, str):
            self._port = serial.Serial(self._port, **self.VMSTEP_SERIAL_KWARGS)
        self._clear_buffer()

    def close(self):
        self._port.close()

    def receive(self):
        data = self._port.read_until(LINE_END)
        if not (data.startswith(LINE_BEGIN) and data.endswith(LINE_END)):
            self.logger.error(f"Invalid response: {data}")
        return data[1:-1]  # strip line begin/end bytes

    def parse_reply(self, data: bytes):
        reply_code = Reply(data[0:1])  # extract reply code

        if reply_code == Reply.ACK:
            return data[1:]  # strip reply byte

        elif reply_code == Reply.FAULT:
            fault = Fault(data[1:2])  # first byte is fault-type

            if fault == Fault.DRIVER:
                fault_reg = data[2]  # DRV8434S fault register
                diag1_reg = data[3]  # DRV8434S diagnostic 1 register
                diag2_reg = data[4]  # DRV8434S diagnostic 2 register
                error_msg = self.parse_driver_fault(fault_reg, diag1_reg, diag2_reg)
                raise RuntimeError(f"Driver Fault:\n{error_msg}")
            else:
                raise RuntimeError(f"Device fault: {fault}")

    def send_command(self, command: Cmd, arg_bytes: bytes = None):
        self.logger.debug(f"Sending: {command.name}")
        message = LINE_BEGIN
        message += command.value
        if arg_bytes:
            message += arg_bytes
        message += LINE_END
        self._port.write(message)
        return self.parse_reply(self.receive())

    def reset(self):
        self.send_command(Cmd.RESET)

    def echo(self):
        results = self.send_command(Cmd.ECHO)
        return results

    def query(self, query: Query):
        results = self.send_command(Cmd.QUERY, query.value)
        return results

    def enable(self):
        self.send_command(Cmd.ENABLE)

    def disable(self):
        self.send_command(Cmd.DISABLE)

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
        self.send_command(Cmd.RESET_POSITION)

    def parse_driver_fault(self, fault_reg: int, diag1_reg: int, diag2_reg: int) -> str:
        """Parse DRV8434S fault registers into human readable message"""
        messages = []

        # Parse each register
        for bit, msg in FAULT_MESSAGES.items():
            if fault_reg & (1 << bit):
                messages.append(msg)

        for bit, msg in DIAG1_MESSAGES.items():
            if diag1_reg & (1 << bit):
                messages.append(msg)

        for bit, msg in DIAG2_MESSAGES.items():
            if diag2_reg & (1 << bit):
                messages.append(msg)

        if not messages:
            return "No faults detected"

        return "\n".join(messages)

    def get_fault_registers(self) -> tuple[int, int, int]:
        """
        Query the driver fault registers directly.
        Returns:
            tuple(fault_reg, diag1_reg, diag2_reg)
        """
        reply = self.query(Query.FAULT_REGS)
        if len(reply) != 3:
            raise ValueError("Invalid fault register response length")

        fault_reg = reply[0]
        diag1_reg = reply[1]
        diag2_reg = reply[2]

        return (fault_reg, diag1_reg, diag2_reg)


# Example Usage:
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    logger.info("Start")

    settings = Settings(
        step_current=1,
        sleep_current=0,
        microstep_resolution=7,
        sleep_timeout=100,
        top_speed=40000,
        acceleration=200000,
        enable_lim1=False,
        enable_lim2=False,
        enable_home=True,
        lim1_sig_polarity=False,
        lim2_sig_polarity=False,
        home_sig_polarity=False,
    )

    with VMSTEP.discover() as mc:
        status = mc.set_parameters(settings)
        print("Reply: ", status)
        status = mc.enable()
        status = mc.goto(-7500)
        status = mc.disable()

        status = mc.get_position()
        print("Position: ", status)

        # mc = VMSTEP(port="/dev/ttyACM0")
        # print("object made")
        # Example commands
        # status = mc.echo()
        status = mc.query(Query.SERIAL_NO)
        print("Serial: ", status)
        status = mc.query(Query.MODEL_NO)
        print("Model: ", status)

        # status = mc.home(True)
        # status = mc.query(Query.FAULTS)

        # status = mc.reset()
        # print("Reply: ", status)

        # status = mc.get_position()
        # print("Reply: ", status)

        # status = mc.get_parameters()
        # print("Reply: ", status)

        # status = mc.goto(-250)
        # print("Reply: ", status)

        # status = mc.disable()
        # print("Reply: ", status)

        # time.sleep(1)
        # status = mc.stop()
        # print(status)

        # # status = mc.home(False)

        # mc.reset_position()

        # status = mc.get_position()
        # print("Position: ", status)

        # Get fault registers
        # fault_reg, diag1_reg, diag2_reg = mc.get_fault_registers()
        # print(f"Fault register: 0x{fault_reg:02x}")
        # print(f"DIAG1 register: 0x{diag1_reg:02x}")
        # print(f"DIAG2 register: 0x{diag2_reg:02x}")

        # # Parse the registers if desired
        # error_msg = mc.parse_driver_fault(fault_reg, diag1_reg, diag2_reg)
        # print("\nFault details:")
        # print(error_msg)
