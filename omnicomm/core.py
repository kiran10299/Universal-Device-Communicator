"""
Core protocol framing, checksums (CRC-16/MODBUS, XOR LRC), hex/ASCII codecs,
Modbus RTU/TCP packet builders, CAN frame codecs, and Virtual Instrument responders.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Union
import struct
import time
import math
import random


def crc16_modbus(data: bytes) -> int:
    """Compute standard CRC-16/MODBUS (polynomial 0xA001, init 0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_crc16_modbus(data: bytes) -> bytes:
    """Append little-endian CRC-16/MODBUS to a frame."""
    crc = crc16_modbus(data)
    return data + struct.pack("<H", crc)


def verify_crc16_modbus(frame: bytes) -> bool:
    """Verify that a Modbus RTU frame ends with a valid CRC-16."""
    if len(frame) < 4:
        return False
    expected = crc16_modbus(frame[:-2])
    actual = struct.unpack("<H", frame[-2:])[0]
    return expected == actual


def lrc_xor(data: bytes) -> int:
    """Compute 8-bit longitudinal redundancy check (XOR of all bytes)."""
    acc = 0
    for b in data:
        acc ^= b
    return acc & 0xFF


def hex_to_bytes(hex_str: str) -> bytes:
    """Convert flexible hex string ('01 03 00 00 00 02' or '010300000002') to bytes."""
    cleaned = (
        hex_str.replace("0x", "")
        .replace("0X", "")
        .replace(",", " ")
        .replace("-", " ")
        .strip()
    )
    if not cleaned:
        return b""
    parts = cleaned.split()
    if len(parts) == 1 and len(parts[0]) % 2 == 0:
        return bytes.fromhex(parts[0])
    return bytes(int(p, 16) for p in parts)


def bytes_to_hex(data: bytes, sep: str = " ") -> str:
    """Format bytes as uppercase hex string ('01 03 04 00 C8')."""
    return sep.join(f"{b:02X}" for b in data)


def bytes_to_printable_ascii(data: bytes) -> str:
    """Convert bytes to readable ASCII, replacing non-printable bytes with '.'."""
    out = []
    for b in data:
        if 32 <= b <= 126:
            out.append(chr(b))
        elif b == 10:
            out.append("\\n")
        elif b == 13:
            out.append("\\r")
        elif b == 9:
            out.append("\\t")
        else:
            out.append(".")
    return "".join(out)


@dataclass
class PacketLogEntry:
    """Unified TX/RX packet record across any communication transport."""
    timestamp: float
    direction: str  # "TX" or "RX"
    protocol: str
    endpoint: str
    raw_bytes: bytes
    note: str = ""

    @property
    def hex_str(self) -> str:
        return bytes_to_hex(self.raw_bytes)

    @property
    def ascii_str(self) -> str:
        return bytes_to_printable_ascii(self.raw_bytes)

    @property
    def length(self) -> int:
        return len(self.raw_bytes)

    def to_dict(self) -> Dict[str, Union[str, int, float]]:
        return {
            "timestamp": self.timestamp,
            "direction": self.direction,
            "protocol": self.protocol,
            "endpoint": self.endpoint,
            "length": self.length,
            "hex": self.hex_str,
            "ascii": self.ascii_str,
            "note": self.note,
        }


class ModbusCodec:
    """Builds and parses Modbus RTU and Modbus TCP frames."""

    @staticmethod
    def build_rtu_read_request(slave_id: int, function_code: int, start_addr: int, count: int) -> bytes:
        if function_code not in (1, 2, 3, 4):
            raise ValueError(f"Unsupported read function code: {function_code}")
        pdu = struct.pack(">BBHH", slave_id & 0xFF, function_code & 0xFF, start_addr & 0xFFFF, count & 0xFFFF)
        return append_crc16_modbus(pdu)

    @staticmethod
    def build_rtu_write_single(slave_id: int, function_code: int, addr: int, value: int) -> bytes:
        if function_code not in (5, 6):
            raise ValueError(f"Unsupported single-write function code: {function_code}")
        pdu = struct.pack(">BBHH", slave_id & 0xFF, function_code & 0xFF, addr & 0xFFFF, value & 0xFFFF)
        return append_crc16_modbus(pdu)

    @staticmethod
    def build_tcp_frame(transaction_id: int, unit_id: int, function_code: int, payload: bytes) -> bytes:
        length = 1 + 1 + len(payload)  # unit_id + fc + payload
        mbap = struct.pack(">HHHB", transaction_id & 0xFFFF, 0, length, unit_id & 0xFF)
        return mbap + bytes([function_code & 0xFF]) + payload

    @staticmethod
    def decode_registers(reg_bytes: bytes) -> List[int]:
        """Decode big-endian 16-bit register byte stream into list of uint16 values."""
        if len(reg_bytes) % 2 != 0:
            reg_bytes = reg_bytes[:-1]
        count = len(reg_bytes) // 2
        return list(struct.unpack(f">{count}H", reg_bytes))

    @staticmethod
    def registers_to_float32(high_word: int, low_word: int) -> float:
        """Convert two 16-bit Modbus registers (big-endian word order) to IEEE-754 float32."""
        raw = struct.pack(">HH", high_word & 0xFFFF, low_word & 0xFFFF)
        return struct.unpack(">f", raw)[0]


@dataclass
class CanFrame:
    """Represents a CAN 2.0A (11-bit) or 2.0B (29-bit) frame."""
    arbitration_id: int
    data: bytes
    is_extended: bool = False
    is_remote: bool = False
    timestamp: float = field(default_factory=time.time)

    @property
    def dlc(self) -> int:
        return len(self.data)

    def to_slcan(self) -> str:
        """Serialize frame to Lawicel SLCAN ASCII format (e.g., 't1234DEADBEEF\\r')."""
        if len(self.data) > 8:
            raise ValueError("Classic CAN data length cannot exceed 8 bytes")
        if self.is_extended:
            cmd = "R" if self.is_remote else "T"
            id_hex = f"{self.arbitration_id:08X}"
        else:
            cmd = "r" if self.is_remote else "t"
            id_hex = f"{self.arbitration_id:03X}"
        data_hex = "".join(f"{b:02X}" for b in self.data)
        return f"{cmd}{id_hex}{self.dlc:1X}{data_hex}\r"

    @classmethod
    def from_slcan(cls, line: str) -> "CanFrame":
        """Parse a Lawicel SLCAN ASCII string into a CanFrame."""
        s = line.strip()
        if not s or s[0] not in ("t", "T", "r", "R"):
            raise ValueError(f"Invalid SLCAN frame: {line!r}")
        cmd = s[0]
        is_ext = cmd in ("T", "R")
        is_rem = cmd in ("r", "R")
        id_len = 8 if is_ext else 3
        arb_id = int(s[1 : 1 + id_len], 16)
        dlc = int(s[1 + id_len], 16)
        data_hex = s[2 + id_len : 2 + id_len + dlc * 2]
        data = bytes.fromhex(data_hex) if not is_rem and data_hex else b""
        return cls(arbitration_id=arb_id, data=data, is_extended=is_ext, is_remote=is_rem)


class CameraFramePacket:
    """Binary length-prefixed camera frame packetizer (LabVIEW <-> Python TCP Vision format)."""
    MAGIC = b"OMNI"

    @classmethod
    def pack_frame(cls, jpeg_bytes: bytes, width: int, height: int, frame_seq: int = 0) -> bytes:
        """Pack header (MAGIC + width + height + seq + payload_len) + jpeg_bytes."""
        header = struct.pack(">4sHHII", cls.MAGIC, width, height, frame_seq, len(jpeg_bytes))
        return header + jpeg_bytes

    @classmethod
    def unpack_header(cls, header_bytes: bytes) -> Tuple[int, int, int, int]:
        """Unpack 16-byte header -> (width, height, frame_seq, payload_len)."""
        if len(header_bytes) < 16:
            raise ValueError("Header must be 16 bytes")
        magic, width, height, seq, length = struct.unpack(">4sHHII", header_bytes[:16])
        if magic != cls.MAGIC:
            raise ValueError(f"Invalid camera frame magic: {magic!r}")
        return width, height, seq, length


class VirtualDeviceSimulator:
    """
    Deterministic multi-protocol responder for testing SCPI, Modbus RTU/TCP,
    Serial AT/ASCII, TCP/UDP, Telnet, and CAN Bus without physical hardware.
    """

    def __init__(self) -> None:
        self.holding_registers: Dict[int, int] = {
            0: 2305,   # 230.5 V (scale x10)
            1: 512,    # 5.12 A (scale x100)
            2: 4998,   # 49.98 Hz (scale x100)
            3: 342,    # 34.2 C bearing temp (scale x10)
            4: 1180,   # 1.180 mm/s RMS vibration
            5: 1450,   # 1450 RPM spindle speed
            6: 1,      # System status: 1 = ARMED
            7: 0,      # Fault code: 0 = NO_FAULT
        }
        self.coils: Dict[int, int] = {i: (1 if i % 2 == 0 else 0) for i in range(16)}
        self.scpi_voltage: float = 5.0024
        self.scpi_current: float = 0.2418
        self.scpi_output: bool = True

    def handle_scpi(self, command: str) -> str:
        cmd = command.strip().upper()
        if not cmd:
            return ""
        if cmd == "*IDN?":
            return "KEYSIGHT TECHNOLOGIES,34465A,MY59001284,A.03.01-03.15"
        if cmd == "*STB?":
            return "0"
        if cmd == "*OPC?":
            return "1"
        if cmd in ("*RST", "*CLS"):
            self.scpi_voltage = 5.0000
            self.scpi_current = 0.2000
            return "OK"
        if cmd.startswith(":SYST:ERR") or cmd == "SYST:ERR?":
            return '+0,"No error"'
        if "VOLT" in cmd and "?" in cmd:
            jitter = (math.sin(time.time() * 3.7) * 0.0015)
            return f"{self.scpi_voltage + jitter:+.6E}"
        if "CURR" in cmd and "?" in cmd:
            jitter = (math.cos(time.time() * 2.3) * 0.0008)
            return f"{self.scpi_current + jitter:+.6E}"
        if "FREQ" in cmd and "?" in cmd:
            return "+1.000024E+03"
        if "RES" in cmd and "?" in cmd:
            return "+1.000420E+02"
        if "OUTP?" in cmd:
            return "1" if self.scpi_output else "0"
        if cmd.startswith("VOLT ") or cmd.startswith(":SOUR:VOLT "):
            try:
                self.scpi_voltage = float(cmd.split()[-1])
                return f"ACK:VSET={self.scpi_voltage:.4f}V"
            except ValueError:
                return '-102,"Syntax error"'
        return f"ACK:{cmd}"

    def handle_modbus_rtu(self, frame: bytes) -> bytes:
        if len(frame) < 4 or not verify_crc16_modbus(frame):
            return b""
        slave = frame[0]
        fc = frame[1]
        if fc in (3, 4) and len(frame) >= 8:
            start, count = struct.unpack(">HH", frame[2:6])
            regs = [self.holding_registers.get(start + i, 0) for i in range(min(count, 64))]
            byte_count = len(regs) * 2
            payload = struct.pack(f">BBB{len(regs)}H", slave, fc, byte_count, *regs)
            return append_crc16_modbus(payload)
        if fc == 6 and len(frame) >= 8:
            addr, val = struct.unpack(">HH", frame[2:6])
            self.holding_registers[addr] = val & 0xFFFF
            return frame[:8]  # Echo back request on FC06 success
        if fc == 5 and len(frame) >= 8:
            addr, val = struct.unpack(">HH", frame[2:6])
            self.coils[addr] = 1 if val else 0
            return frame[:8]
        # Exception response: Illegal Function (0x01)
        return append_crc16_modbus(bytes([slave, fc | 0x80, 0x01]))
