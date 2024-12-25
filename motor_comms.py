from __future__ import annotations

import logging
import struct
import time
from collections.abc import Mapping
from ctypes import Structure, Union, c_uint8, c_uint32, sizeof
from enum import Enum
from types import MappingProxyType, TracebackType
from typing import Optional, Type

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
    # 7: "Reserved",
    6: "Overtemperature warning",
    5: "Overtemperature shutdown",
    4: "Stall detection learning successful",
    3: "Motor stall detected",
    # 2: "Reserved",
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


class SettingsFlags(Structure):
    """Maps to the flags bitfield struct in Settings_struct"""

    _fields_ = [
        ("enable_lim1", c_uint8, 1),  # bit 0
        ("enable_lim2", c_uint8, 1),  # bit 1
        ("enable_home", c_uint8, 1),  # bit 2
        ("lim1_sig_polarity", c_uint8, 1),  # bit 3
        ("lim2_sig_polarity", c_uint8, 1),  # bit 4
        ("home_sig_polarity", c_uint8, 1),  # bit 5
        ("reserved", c_uint8, 2),  # bits 6-7
    ]


class SettingsStruct(Structure):
    """Maps to Settings_struct in C++"""

    _pack_ = 1  # Match __attribute__((packed))
    _fields_ = [
        ("step_current", c_uint8, 4),  # max 0b1111
        ("sleep_current", c_uint8, 4),  # max 0b1111
        ("microstep_res", c_uint8, 4),  # max 0b1111
        ("reserved", c_uint8, 4),  # padding for algiment
        ("sleep_timeout", c_uint8),  # 10s of ms
        ("top_speed", c_uint32),  # 32 bits
        ("acceleration", c_uint32),  # 32 bits
        ("flags", SettingsFlags),  # bitfield struct
    ]


class Settings(Union):
    """Maps to Settings_union in C++"""

    _pack_ = 1
    _fields_ = [("data", SettingsStruct), ("bytes", c_uint8 * sizeof(SettingsStruct))]


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
            raise SerialException("No serial ports found on this machine")

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
            raise SerialException(f"Invalid response: {data}")
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
        """Get current device settings"""
        reply = self.query(Query.PARAMETERS)
        settings = Settings()
        for i, b in enumerate(reply):
            settings.bytes[i] = b
        return settings

    def set_parameters(self, settings: Settings):
        """Send new settings to device"""
        results = self.send_command(Cmd.UPDATE_PARAMETERS, bytes(settings.bytes))
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

    settings = Settings()
    settings.data.step_current = 1
    settings.data.sleep_current = 0
    settings.data.microstep_res = 7
    settings.data.sleep_timeout = 100
    settings.data.top_speed = 40000
    settings.data.acceleration = 200000
    settings.data.flags.enable_home = False
    settings.data.flags.home_sig_polarity = False

    with VMSTEP.discover() as mc:
        status = mc.set_parameters(settings)
        print("Reply: ", status)

        # Query ###########################
        # echo = mc.echo()
        # serial = mc.query(Query.SERIAL_NO)
        # model = mc.query(Query.MODEL_NO)
        # firm = mc.query(Query.FIRMWARE)
        # position = mc.get_position()
        # parms = mc.get_parameters()
        # print(f"Echo: {echo}")
        # print(f"Serial: {serial}")
        # print(f"Model: {model}")
        # print(f"Firmware: {firm}")
        # print(f"Position: {position}")
        # print(f"Parameters: {parms}")

        # # Get fault registers
        # fault_reg, diag1_reg, diag2_reg = mc.get_fault_registers()
        # print(f"Fault register: 0x{fault_reg:02x}")
        # print(f"DIAG1 register: 0x{diag1_reg:02x}")
        # print(f"DIAG2 register: 0x{diag2_reg:02x}")

        # # Parse the registers if desired
        # error_msg = mc.parse_driver_fault(fault_reg, diag1_reg, diag2_reg)
        # print(f"\nFault details: \n{error_msg}")

        # Homing ##########################
        settings.data.flags.enable_home = True
        settings.data.top_speed = 10000
        mc.set_parameters(settings)
        mc.enable()
        mc.home(False)
        status = mc.goto(-830)
        print("Reply: ", status)
        mc.reset_position()
        mc.disable()
