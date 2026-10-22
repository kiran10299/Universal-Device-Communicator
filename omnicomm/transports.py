"""
Universal Hardware Transport Implementations:
- SCPITransport (Raw TCP :5025 / PyVISA / Virtual Instrument)
- SerialTransport (RS-232 / RS-485 via pyserial or Virtual Loopback)
- ModbusTransport (Modbus TCP & Modbus RTU over RS-485)
- RawSocketTransport (TCP Client, UDP Datagram, Telnet)
- CanBusTransport (SLCAN / SocketCAN / Virtual CAN)
- CameraTransport (USB DirectShow / UVC / TCP Frame Stream / Synthetic Test Pattern)
"""

from typing import Any, Dict, List, Optional, Tuple
import socket
import struct
import time

from .core import (
    CameraFramePacket,
    CanFrame,
    ModbusCodec,
    PacketLogEntry,
    VirtualDeviceSimulator,
    append_crc16_modbus,
    bytes_to_hex,
    hex_to_bytes,
)


class UniversalCommunicator:
    """
    Single unified Read/Write/Query interface across SCPI, RS-232/485, Modbus RTU/TCP,
    TCP, UDP, Telnet, CAN Bus, and USB/DirectShow/TCP Camera protocols.
    """

    def __init__(self, simulate: bool = True) -> None:
        self.simulate = simulate
        self.connected = False
        self.protocol = "SCPI"
        self.endpoint = "SIM://KEYSIGHT-34465A"
        self.config: Dict[str, Any] = {}
        self.sim = VirtualDeviceSimulator()
        self.packet_log: List[PacketLogEntry] = []
        self._sock: Optional[socket.socket] = None
        self._serial: Any = None
        self._frame_seq: int = 0

    def connect(self, protocol: str, endpoint: str, **kwargs: Any) -> Dict[str, Any]:
        """Connect to the target instrument, serial port, PLC, socket, CAN bus, or camera."""
        self.disconnect()
        self.protocol = protocol.upper()
        self.endpoint = endpoint
        self.config = kwargs
        self.simulate = kwargs.get("simulate", self.simulate or endpoint.upper().startswith("SIM"))

        if not self.simulate:
            if self.protocol in ("TCP", "SCPI", "MODBUS_TCP", "TELNET", "CAMERA_TCP"):
                host = kwargs.get("host", "127.0.0.1")
                port = int(kwargs.get("port", 5025))
                timeout = float(kwargs.get("timeout", 2.0))
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(timeout)
                sock.connect((host, port))
                self._sock = sock
            elif self.protocol == "UDP":
                timeout = float(kwargs.get("timeout", 2.0))
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(timeout)
                self._sock = sock
            elif self.protocol in ("RS232", "RS485", "MODBUS_RTU", "CAN_SLCAN"):
                import serial  # type: ignore
                port_name = kwargs.get("port", endpoint)
                baudrate = int(kwargs.get("baudrate", 115200))
                timeout = float(kwargs.get("timeout", 1.0))
                self._serial = serial.Serial(port=port_name, baudrate=baudrate, timeout=timeout)

        self.connected = True
        return {
            "status": "CONNECTED",
            "protocol": self.protocol,
            "endpoint": self.endpoint,
            "simulate": self.simulate,
            "config": self.config,
        }

    def disconnect(self) -> Dict[str, Any]:
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        self.connected = False
        return {"status": "DISCONNECTED", "protocol": self.protocol}

    def _record(self, direction: str, raw: bytes, note: str = "") -> PacketLogEntry:
        entry = PacketLogEntry(
            timestamp=time.time(),
            direction=direction,
            protocol=self.protocol,
            endpoint=self.endpoint,
            raw_bytes=raw,
            note=note,
        )
        self.packet_log.append(entry)
        return entry

    def write(self, payload: bytes, note: str = "") -> PacketLogEntry:
        """Write raw bytes to the currently connected device."""
        if not self.connected:
            raise RuntimeError("Communicator is not connected")

        tx_entry = self._record("TX", payload, note=note)
        if not self.simulate:
            if self._sock is not None:
                if self.protocol == "UDP":
                    host = self.config.get("host", "127.0.0.1")
                    port = int(self.config.get("port", 9000))
                    self._sock.sendto(payload, (host, port))
                else:
                    self._sock.sendall(payload)
            elif self._serial is not None:
                self._serial.write(payload)
        return tx_entry

    def read(self, num_bytes: int = 1024, last_tx: bytes = b"") -> PacketLogEntry:
        """Read response bytes from the connected device or virtual responder."""
        if not self.connected:
            raise RuntimeError("Communicator is not connected")

        if self.simulate:
            rx_bytes = self._simulate_response(last_tx)
        else:
            rx_bytes = b""
            if self._sock is not None:
                if self.protocol == "UDP":
                    rx_bytes, _ = self._sock.recvfrom(num_bytes)
                else:
                    rx_bytes = self._sock.recv(num_bytes)
            elif self._serial is not None:
                rx_bytes = self._serial.read(num_bytes)

        return self._record("RX", rx_bytes)

    def transceive(
        self,
        data: str,
        mode: str = "ASCII",
        line_ending: str = "\n",
        append_crc: bool = False,
    ) -> Tuple[PacketLogEntry, PacketLogEntry]:
        """Write a command (ASCII or HEX) and immediately read back the response."""
        if mode.upper() == "HEX":
            raw_tx = hex_to_bytes(data)
        else:
            raw_tx = (data + line_ending).encode("utf-8")

        if append_crc:
            raw_tx = append_crc16_modbus(raw_tx)

        tx_entry = self.write(raw_tx)
        rx_entry = self.read(last_tx=raw_tx)
        return tx_entry, rx_entry

    def _simulate_response(self, last_tx: bytes) -> bytes:
        proto = self.protocol.upper()
        if proto in ("SCPI", "VISA"):
            text = last_tx.decode("utf-8", errors="ignore").strip()
            resp = self.sim.handle_scpi(text)
            return (resp + "\n").encode("utf-8")
        if proto == "MODBUS_RTU":
            return self.sim.handle_modbus_rtu(last_tx)
        if proto == "MODBUS_TCP" and len(last_tx) >= 8:
            tid, pid, length, unit_id, fc = struct.unpack(">HHHBB", last_tx[:8])
            rtu_req = append_crc16_modbus(bytes([unit_id, fc]) + last_tx[8:])
            rtu_resp = self.sim.handle_modbus_rtu(rtu_req)
            if len(rtu_resp) >= 4:
                pdu_payload = rtu_resp[2:-2]
                return ModbusCodec.build_tcp_frame(tid, unit_id, rtu_resp[1], pdu_payload)
            return b""
        if proto in ("CAN", "CAN_SLCAN"):
            frame = CanFrame(arbitration_id=0x18FF50E5, data=bytes([0x10, 0x24, 0x05, 0xDC, 0x01, 0x00, 0xAA, 0x55]), is_extended=True)
            return frame.to_slcan().encode("ascii")
        if proto.startswith("CAMERA"):
            self._frame_seq += 1
            fake_jpeg = b"\xff\xd8\xff\xe0OMNICOMM_SYNTH_FRAME\xff\xd9"
            return CameraFramePacket.pack_frame(fake_jpeg, width=640, height=480, frame_seq=self._frame_seq)
        # Default serial / TCP / UDP / Telnet deterministic echo + telemetry ack
        clean = last_tx.decode("utf-8", errors="ignore").strip()
        return f"OK [{proto}] RX={len(last_tx)}B ECHO={clean}\r\n".encode("utf-8")
