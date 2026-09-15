"""Minimal Modbus RTU master over pyserial (FC03 / FC06 / FC10).

Shared by scripts/acm9767_ctl.py; framing and CRC-16 are implemented here so
no pymodbus dependency is needed. UART/Modbus defaults come from the repo's
platform.json through platform_config.py (the same file the gateware is built
from), with a hard-coded fallback for a standalone copy of this file.
"""

import os
import struct
import sys

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover - import guard
    serial = None

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    import platform_config as _platform

    DEFAULT_BAUD, DEFAULT_SLAVE = _platform.UART_BAUD, _platform.MODBUS_DEVICE_ID
    DEFAULT_BYTESIZE = _platform.UART_DATA_BITS
    DEFAULT_PARITY = _platform.UART_PARITY
    DEFAULT_STOP = _platform.UART_STOP_BITS
except Exception:  # pragma: no cover - standalone fallback
    DEFAULT_BAUD, DEFAULT_SLAVE = 1000000, 7
    DEFAULT_BYTESIZE, DEFAULT_PARITY, DEFAULT_STOP = 8, "E", 1

DEFAULT_PORT = "/dev/ttyGowin"


def crc16(data: bytes) -> int:
    """CRC-16/Modbus (poly 0xA001, init 0xFFFF). Appended little-endian."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc


def _u16(name, v):
    if not isinstance(v, int) or not (0 <= v <= 0xFFFF):
        raise ValueError(f"{name} {v!r} out of range 0..65535")
    return v


class ModbusError(Exception):
    def __init__(self, code):
        self.code = code
        names = {1: "illegal function", 2: "illegal data address",
                 3: "illegal data value", 4: "slave device failure"}
        super().__init__(f"Modbus exception 0x{code:02X} ({names.get(code, '?')})")


class ModbusRTU:
    """RTU master bound to one slave id on one serial port.

    `transport` may be any object with write()/read(n)/reset_input_buffer()/
    close() -- the tests inject a fake; normal use opens pyserial on `port`.
    """

    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD, slave=DEFAULT_SLAVE,
                 timeout=1.0, transport=None):
        if not (0 <= slave <= 247):
            raise ValueError(f"slave id {slave} out of range 0..247")
        self.slave = slave
        if transport is not None:
            self.ser = transport
        else:
            if serial is None:
                raise RuntimeError("pyserial not installed -- run: pip install pyserial")
            self.ser = serial.Serial(
                port=port, baudrate=baud, bytesize=DEFAULT_BYTESIZE,
                parity=DEFAULT_PARITY, stopbits=DEFAULT_STOP, timeout=timeout)

    def close(self):
        self.ser.close()

    def _read_exact(self, n):
        buf = self.ser.read(n)
        if len(buf) != n:
            raise TimeoutError(f"timeout: wanted {n} bytes, got {len(buf)}")
        return buf

    @staticmethod
    def _check_crc(frame):
        if crc16(frame) != 0:
            raise ValueError("response CRC mismatch")

    def _txn(self, func, payload):
        req = bytes([self.slave, func]) + payload
        req += struct.pack("<H", crc16(req))
        self.ser.reset_input_buffer()
        self.ser.write(req)

        head = self._read_exact(2)
        rfunc = head[1]
        if rfunc & 0x80:
            rest = self._read_exact(3)
            self._check_crc(head + rest)
            raise ModbusError(rest[0])
        if rfunc == 0x03:
            bc = self._read_exact(1)
            rest = bc + self._read_exact(bc[0] + 2)
        elif rfunc in (0x06, 0x10):
            rest = self._read_exact(4 + 2)
        else:
            raise ValueError(f"unexpected function 0x{rfunc:02X} in response")
        self._check_crc(head + rest)
        if head[0] != self.slave:
            raise ValueError(f"response from slave {head[0]}, expected {self.slave}")
        return rest

    def read_holding(self, addr, count):
        _u16("address", addr)
        if not (1 <= count <= 125):
            raise ValueError(f"read count {count} out of range 1..125")
        rest = self._txn(0x03, struct.pack(">HH", addr, count))
        if rest[0] != 2 * count:
            raise ValueError(f"byte count {rest[0]} != expected {2 * count}")
        return list(struct.unpack(">" + "H" * count, rest[1:1 + rest[0]]))

    def write_single(self, addr, value):
        _u16("address", addr)
        _u16("value", value)
        self._txn(0x06, struct.pack(">HH", addr, value))

    def write_multiple(self, addr, values):
        _u16("address", addr)
        if not (1 <= len(values) <= 123):
            raise ValueError(f"register count {len(values)} out of range 1..123")
        for i, v in enumerate(values):
            _u16(f"value[{i}]", v)
        payload = struct.pack(">HHB", addr, len(values), 2 * len(values))
        payload += b"".join(struct.pack(">H", v) for v in values)
        self._txn(0x10, payload)
