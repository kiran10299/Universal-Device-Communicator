"""
Universal Hardware Transport Implementations:
- SCPITransport (Raw TCP :5025, VISA TCPIP string parser, PyVISA USB/GPIB/VISA fallback, or Virtual Instrument)
- SerialTransport (RS-232 / RS-485 via pyserial with 8N1/8E1/8O1/7E1 framing & fast in_waiting drain)
- ModbusTransport (Modbus TCP on Port 502 & Modbus RTU over RS-485 with automatic CRC-16)
- RawSocketTransport (TCP Client with TCP_NODELAY, UDP Datagram, Telnet with RFC 854 IAC negotiation)
- CanBusTransport (Lawicel SLCAN serial adapter with bitrate init, python-can Bus fallback, or Virtual CAN)
- CameraTransport (OpenCV DirectShow/UVC USB Camera, TCP Binary Frame Stream, or Synthetic Pattern)
"""

from typing import Any, Dict, List, Optional, Tuple
import re
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

SLCAN_BITRATE_MAP = {
    10000: "S0",
    20000: "S1",
    50000: "S2",
    100000: "S3",
    125000: "S4",
    250000: "S5",
    500000: "S6",
    800000: "S7",
    1000000: "S8",
}


def parse_endpoint_host_port(endpoint: str, default_host: str = "127.0.0.1", default_port: int = 5025) -> Tuple[str, int]:
    """
    Extract (host, port) from VISA resource strings like 'TCPIP0::192.168.1.50::5025::SOCKET',
    URLs like 'tcp://192.168.1.50:5025', or '192.168.1.50:502'.
    """
    ep = (endpoint or "").strip()
    if not ep:
        return default_host, default_port

    # Match TCPIP[0]::host::port::SOCKET or TCPIP[0]::host::INSTR
    m_visa = re.match(r"^TCPIP\d*::([^:]+)(?:::(\d+))?", ep, re.IGNORECASE)
    if m_visa:
        host = m_visa.group(1)
        port = int(m_visa.group(2)) if m_visa.group(2) else default_port
        return host, port

    # Strip scheme://
    ep_clean = re.sub(r"^[a-zA-Z0-9_+.-]+://", "", ep)
    if ":" in ep_clean:
        parts = ep_clean.rsplit(":", 1)
        host = parts[0].strip() or default_host
        try:
            port = int(parts[1].split("/")[0].strip())
        except ValueError:
            port = default_port
        return host, port

    return ep_clean, default_port


def strip_telnet_iac(data: bytes) -> Tuple[bytes, bytes]:
    """
    Parse RFC 854 Telnet IAC (0xFF) command sequences from incoming bytes.
    Returns (clean_ascii_bytes, iac_reply_bytes) where DO/WILL requests are politely refused (WONT/DONT).
    """
    clean = bytearray()
    reply = bytearray()
    i = 0
    n = len(data)
    while i < n:
        b = data[i]
        if b == 0xFF and i + 1 < n:
            cmd = data[i + 1]
            if cmd == 0xFF:
                clean.append(0xFF)
                i += 2
            elif cmd in (0xFD, 0xFE, 0xFB, 0xFC) and i + 2 < n:
                opt = data[i + 2]
                # Refuse DO (0xFD) with WONT (0xFC); refuse WILL (0xFB) with DONT (0xFE)
                if cmd == 0xFD:
                    reply.extend([0xFF, 0xFC, opt])
                elif cmd == 0xFB:
                    reply.extend([0xFF, 0xFE, opt])
                i += 3
            elif cmd == 0xFA:
                # Subnegotiation until IAC SE (0xFF 0xF0)
                i += 2
                while i < n - 1 and not (data[i] == 0xFF and data[i + 1] == 0xF0):
                    i += 1
                i += 2
            else:
                i += 2
        else:
            clean.append(b)
            i += 1
    return bytes(clean), bytes(reply)


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
        self._visa_inst: Any = None
        self._can_bus: Any = None
        self._cv_cap: Any = None
        self._frame_seq: int = 0

    def connect(self, protocol: str, endpoint: str, **kwargs: Any) -> Dict[str, Any]:
        """Connect to the target instrument, serial port, PLC, socket, CAN bus, or camera."""
        self.disconnect()
        self.protocol = protocol.upper()
        self.endpoint = endpoint
        self.config = dict(kwargs)
        self.simulate = bool(kwargs.get("simulate", self.simulate or str(endpoint).upper().startswith("SIM")))

        if not self.simulate:
            if self.protocol in ("TCP", "SCPI", "VISA", "MODBUS_TCP", "TELNET", "CAMERA_TCP"):
                # If VISA USB/GPIB resource string is given, try pyvisa first
                if self.protocol in ("SCPI", "VISA") and re.match(r"^(USB|GPIB|ASRL)\d*::", str(endpoint), re.I):
                    import pyvisa  # type: ignore
                    rm = pyvisa.ResourceManager()
                    self._visa_inst = rm.open_resource(endpoint)
                    self._visa_inst.timeout = int(float(kwargs.get("timeout", 2.0)) * 1000)
                else:
                    default_port = {
                        "SCPI": 5025,
                        "VISA": 5025,
                        "MODBUS_TCP": 502,
                        "TELNET": 23,
                        "CAMERA_TCP": 8090,
                        "TCP": 9000,
                    }.get(self.protocol, 5025)
                    ep_host, ep_port = parse_endpoint_host_port(str(endpoint), "127.0.0.1", default_port)
                    host = str(kwargs.get("host") or ep_host)
                    port = int(kwargs.get("port") or ep_port)
                    timeout = float(kwargs.get("timeout", 2.0))
                    self.config["host"] = host
                    self.config["port"] = port

                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    sock.settimeout(timeout)
                    sock.connect((host, port))
                    self._sock = sock

            elif self.protocol == "UDP":
                ep_host, ep_port = parse_endpoint_host_port(str(endpoint), "127.0.0.1", 9000)
                host = str(kwargs.get("host") or ep_host)
                port = int(kwargs.get("port") or ep_port)
                timeout = float(kwargs.get("timeout", 2.0))
                self.config["host"] = host
                self.config["port"] = port
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(timeout)
                if kwargs.get("bind_port"):
                    sock.bind(("0.0.0.0", int(kwargs["bind_port"])))
                self._sock = sock

            elif self.protocol in ("RS232", "RS485", "MODBUS_RTU", "CAN", "CAN_SLCAN"):
                # Check if python-can interface is requested for CAN
                if self.protocol in ("CAN", "CAN_SLCAN") and kwargs.get("bustype"):
                    import can  # type: ignore
                    self._can_bus = can.interface.Bus(
                        channel=kwargs.get("channel", endpoint),
                        bustype=kwargs.get("bustype", "socketcan"),
                        bitrate=int(kwargs.get("bitrate", 500000)),
                    )
                else:
                    import serial  # type: ignore
                    port_name = str(kwargs.get("port") or endpoint).split()[0]
                    baudrate = int(str(kwargs.get("baudrate", 115200)).split()[0])
                    timeout = float(kwargs.get("timeout", 1.0))
                    framing = str(kwargs.get("framing", "8N1")).upper()

                    bytesize = serial.SEVENBITS if framing.startswith("7") else serial.EIGHTBITS
                    parity = serial.PARITY_NONE
                    if "E" in framing:
                        parity = serial.PARITY_EVEN
                    elif "O" in framing:
                        parity = serial.PARITY_ODD
                    stopbits = serial.STOPBITS_TWO if framing.endswith("2") else serial.STOPBITS_ONE

                    self._serial = serial.Serial(
                        port=port_name,
                        baudrate=baudrate,
                        bytesize=bytesize,
                        parity=parity,
                        stopbits=stopbits,
                        timeout=timeout,
                    )
                    # If Lawicel SLCAN adapter, initialize CAN bitrate & open channel
                    if self.protocol in ("CAN", "CAN_SLCAN"):
                        bitrate_str = str(kwargs.get("bitrate", "500000")).lower().replace("kbps", "000").strip()
                        try:
                            bitrate_val = int(bitrate_str.split()[0])
                            if bitrate_val <= 1000:
                                bitrate_val *= 1000
                        except ValueError:
                            bitrate_val = 500000
                        slcan_cmd = SLCAN_BITRATE_MAP.get(bitrate_val, "S6")
                        self._serial.write(f"C\r{slcan_cmd}\rO\r".encode("ascii"))

            elif self.protocol in ("CAMERA", "CAMERA_USB", "CAMERA_DSHOW"):
                # Check if endpoint is a TCP stream or USB index
                if "://" in str(endpoint) and str(endpoint).lower().startswith("tcp://"):
                    ep_host, ep_port = parse_endpoint_host_port(str(endpoint), "127.0.0.1", 8090)
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    sock.settimeout(float(kwargs.get("timeout", 2.0)))
                    sock.connect((ep_host, ep_port))
                    self._sock = sock
                else:
                    import cv2  # type: ignore
                    cam_idx = int(kwargs.get("camera_index", 0) if str(endpoint).isdigit() is False else int(endpoint))
                    self._cv_cap = cv2.VideoCapture(cam_idx)

        self.connected = True
        return {
            "status": "CONNECTED",
            "protocol": self.protocol,
            "endpoint": self.endpoint,
            "simulate": self.simulate,
            "config": self.config,
        }

    def disconnect(self) -> Dict[str, Any]:
        for attr in ("_sock", "_serial", "_visa_inst", "_can_bus"):
            obj = getattr(self, attr, None)
            if obj is not None:
                try:
                    if attr == "_can_bus":
                        obj.shutdown()
                    else:
                        obj.close()
                except Exception:
                    pass
                setattr(self, attr, None)
        if self._cv_cap is not None:
            try:
                self._cv_cap.release()
            except Exception:
                pass
            self._cv_cap = None
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
            if self._visa_inst is not None:
                self._visa_inst.write_raw(payload)
            elif self._sock is not None:
                if self.protocol == "UDP":
                    host = str(self.config.get("host", "127.0.0.1"))
                    port = int(self.config.get("port", 9000))
                    self._sock.sendto(payload, (host, port))
                else:
                    self._sock.sendall(payload)
            elif self._serial is not None:
                self._serial.write(payload)
                self._serial.flush()
            elif self._can_bus is not None:
                import can  # type: ignore
                txt = payload.decode("ascii", errors="ignore").strip()
                if txt and txt[0] in ("t", "T"):
                    cf = CanFrame.from_slcan(txt)
                    msg = can.Message(arbitration_id=cf.arbitration_id, data=cf.data, is_extended_id=cf.is_extended)
                else:
                    arb_id = int(str(self.config.get("arbitration_id", "0x123")), 0)
                    msg = can.Message(arbitration_id=arb_id, data=payload[:8], is_extended_id=(arb_id > 0x7FF))
                self._can_bus.send(msg)
        return tx_entry

    def read(self, num_bytes: int = 4096, last_tx: bytes = b"") -> PacketLogEntry:
        """Read response bytes from the connected device or virtual responder."""
        if not self.connected:
            raise RuntimeError("Communicator is not connected")

        if self.simulate:
            rx_bytes = self._simulate_response(last_tx)
        else:
            rx_bytes = b""
            if self._visa_inst is not None:
                rx_bytes = self._visa_inst.read_raw()
            elif self._sock is not None:
                if self.protocol == "UDP":
                    rx_bytes, _ = self._sock.recvfrom(num_bytes)
                elif self.protocol == "TELNET":
                    raw_in = self._sock.recv(num_bytes)
                    clean, iac_reply = strip_telnet_iac(raw_in)
                    if iac_reply:
                        self._sock.sendall(iac_reply)
                    # If first recv was only IAC negotiation, read once more for actual banner/response
                    if not clean:
                        try:
                            more_in = self._sock.recv(num_bytes)
                            clean2, iac_reply2 = strip_telnet_iac(more_in)
                            if iac_reply2:
                                self._sock.sendall(iac_reply2)
                            clean += clean2
                        except socket.timeout:
                            pass
                    rx_bytes = clean
                else:
                    rx_bytes = self._sock.recv(num_bytes)
            elif self._serial is not None:
                # Read first byte(s) up to timeout, then drain any remaining bytes in OS buffer immediately
                first = self._serial.read(1)
                if first:
                    time.sleep(0.005)
                    waiting = getattr(self._serial, "in_waiting", 0)
                    rest = self._serial.read(waiting if waiting > 0 else num_bytes - 1) if waiting else b""
                    rx_bytes = first + rest
            elif self._can_bus is not None:
                msg = self._can_bus.recv(timeout=float(self.config.get("timeout", 1.0)))
                if msg is not None:
                    cf = CanFrame(arbitration_id=msg.arbitration_id, data=bytes(msg.data), is_extended=msg.is_extended_id)
                    rx_bytes = cf.to_slcan().encode("ascii")
            elif self._cv_cap is not None:
                import cv2  # type: ignore
                ok, frame = self._cv_cap.read()
                if ok and frame is not None:
                    self._frame_seq += 1
                    h, w = frame.shape[:2]
                    ok_enc, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                    jpeg_bytes = buf.tobytes() if ok_enc else b""
                    rx_bytes = CameraFramePacket.pack_frame(jpeg_bytes, width=w, height=h, frame_seq=self._frame_seq)

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
