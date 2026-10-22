from http.server import HTTPServer
import json
import socket
import sys
import threading
import urllib.request
from unittest.mock import MagicMock, patch

from omnicomm.core import (
    CameraFramePacket,
    CanFrame,
    ModbusCodec,
    append_crc16_modbus,
    crc16_modbus,
    hex_to_bytes,
    verify_crc16_modbus,
)
from omnicomm.transports import UniversalCommunicator, parse_endpoint_host_port, strip_telnet_iac
from omnicomm_cli import BridgeHandler


def test_crc16_modbus():
    # Standard Modbus RTU Read Holding Registers: 01 03 00 00 00 04 -> CRC 0x0944 (LE: 44 09)
    pdu = hex_to_bytes("01 03 00 00 00 04")
    assert crc16_modbus(pdu) == 0x0944
    framed = append_crc16_modbus(pdu)
    assert framed[-2:] == bytes([0x44, 0x09])
    assert verify_crc16_modbus(framed) is True


def test_scpi_virtual_transceive():
    comm = UniversalCommunicator(simulate=True)
    res = comm.connect("SCPI", "TCPIP0::192.168.1.50::5025::SOCKET")
    assert res["status"] == "CONNECTED"
    tx, rx = comm.transceive("*IDN?")
    assert "KEYSIGHT" in rx.ascii_str
    tx2, rx2 = comm.transceive(":SYST:ERR?")
    assert "No error" in rx2.ascii_str


def test_modbus_rtu_read_write_registers():
    comm = UniversalCommunicator(simulate=True)
    comm.connect("MODBUS_RTU", "COM4")
    # Write Single Register 0x0002 = 7777 (0x1E61)
    write_frame = append_crc16_modbus(bytes([0x01, 0x06, 0x00, 0x02, 0x1E, 0x61]))
    comm.write(write_frame)
    ack = comm.read(last_tx=write_frame)
    assert ack.raw_bytes == write_frame

    # Read back first 4 holding registers
    read_frame = ModbusCodec.build_rtu_read_request(1, 3, 0, 4)
    comm.write(read_frame)
    resp = comm.read(last_tx=read_frame)
    assert verify_crc16_modbus(resp.raw_bytes)
    regs = ModbusCodec.decode_registers(resp.raw_bytes[3:-2])
    assert regs[2] == 7777


def test_can_slcan_codec():
    frame = CanFrame(arbitration_id=0x123, data=bytes([0xDE, 0xAD, 0xBE, 0xEF]))
    encoded = frame.to_slcan()
    assert encoded == "t1234DEADBEEF\r"
    decoded = CanFrame.from_slcan(encoded)
    assert decoded.arbitration_id == 0x123
    assert decoded.data == bytes([0xDE, 0xAD, 0xBE, 0xEF])
    assert decoded.is_extended is False


def test_camera_frame_packet_header():
    payload = b"\xff\xd8\xff\xe0TEST_JPEG\xff\xd9"
    pkt = CameraFramePacket.pack_frame(payload, width=1280, height=720, frame_seq=42)
    width, height, seq, length = CameraFramePacket.unpack_header(pkt)
    assert width == 1280
    assert height == 720
    assert seq == 42
    assert length == len(payload)
    assert pkt[16:] == payload


def test_real_scpi_tcp_socket_server():
    """Verify real TCP socket connection (simulate=False) with VISA TCPIP endpoint parsing."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]

    def server_worker():
        conn, _ = srv.accept()
        with conn:
            data = conn.recv(1024)
            if b"*IDN?" in data:
                conn.sendall(b"YOKOGAWA,WT500,91N82041,F1.04\n")
        srv.close()

    t = threading.Thread(target=server_worker, daemon=True)
    t.start()

    comm = UniversalCommunicator(simulate=False)
    comm.connect("SCPI", f"TCPIP0::127.0.0.1::{port}::SOCKET", simulate=False, timeout=2.0)
    tx, rx = comm.transceive("*IDN?", mode="ASCII", line_ending="\n")
    comm.disconnect()
    t.join(timeout=2.0)

    assert "YOKOGAWA,WT500" in rx.ascii_str


def test_real_modbus_tcp_server():
    """Verify real Modbus TCP socket communication (simulate=False) over OS loopback."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]

    def plc_worker():
        conn, _ = srv.accept()
        with conn:
            req = conn.recv(1024)
            # Echo MBAP transaction ID and return 2 holding registers: [2400, 5000]
            tid = (req[0] << 8) | req[1]
            resp = ModbusCodec.build_tcp_frame(tid, 1, 3, bytes([4, 0x09, 0x60, 0x13, 0x88]))
            conn.sendall(resp)
        srv.close()

    t = threading.Thread(target=plc_worker, daemon=True)
    t.start()

    comm = UniversalCommunicator(simulate=False)
    comm.connect("MODBUS_TCP", f"127.0.0.1:{port}", simulate=False, timeout=2.0)
    tx, rx = comm.transceive("00 07 00 00 00 06 01 03 00 00 00 02", mode="HEX")
    comm.disconnect()
    t.join(timeout=2.0)

    regs = ModbusCodec.decode_registers(rx.raw_bytes[9:])
    assert regs == [2400, 5000]


def test_real_udp_datagram_server():
    """Verify real UDP datagram read/write communication (simulate=False)."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    srv.bind(("127.0.0.1", 0))
    port = srv.getsockname()[1]

    def udp_worker():
        data, addr = srv.recvfrom(1024)
        srv.sendto(b"UDP_ACK:" + data, addr)
        srv.close()

    t = threading.Thread(target=udp_worker, daemon=True)
    t.start()

    comm = UniversalCommunicator(simulate=False)
    comm.connect("UDP", f"127.0.0.1:{port}", simulate=False, timeout=2.0)
    tx, rx = comm.transceive("TELEMETRY_PING", mode="ASCII", line_ending="")
    comm.disconnect()
    t.join(timeout=2.0)

    assert rx.ascii_str == "UDP_ACK:TELEMETRY_PING"


def test_real_telnet_iac_negotiation_server():
    """Verify real Telnet socket with RFC 854 IAC negotiation stripping and refusal."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]
    received_iac_refusal = []

    def telnet_worker():
        conn, _ = srv.accept()
        with conn:
            # Wait for client command, then send IAC DO ECHO (FF FD 01) + IAC WILL SGA (FF FB 03) + ASCII reply
            _ = conn.recv(1024)
            conn.sendall(bytes([0xFF, 0xFD, 0x01, 0xFF, 0xFB, 0x03]) + b"GM10_READY\r\n")
            refusal = conn.recv(1024)
            received_iac_refusal.append(refusal)
        srv.close()

    t = threading.Thread(target=telnet_worker, daemon=True)
    t.start()

    comm = UniversalCommunicator(simulate=False)
    comm.connect("TELNET", f"127.0.0.1:{port}", simulate=False, timeout=2.0)
    tx, rx = comm.transceive("STATUS", mode="ASCII", line_ending="\r\n")
    comm.disconnect()
    t.join(timeout=2.0)

    assert "GM10_READY" in rx.ascii_str
    # Verify client sent IAC WONT ECHO (FF FC 01) + IAC DONT SGA (FF FE 03)
    assert received_iac_refusal[0] == bytes([0xFF, 0xFC, 0x01, 0xFF, 0xFE, 0x03])


def test_real_serial_rs232_modbus_rtu_and_can_slcan():
    """Verify pyserial hardware path (simulate=False) for RS-232, Modbus RTU, and CAN SLCAN."""
    fake_serial_mod = MagicMock()
    serial_instance = MagicMock()
    fake_serial_mod.Serial.return_value = serial_instance
    fake_serial_mod.EIGHTBITS = 8
    fake_serial_mod.SEVENBITS = 7
    fake_serial_mod.PARITY_NONE = "N"
    fake_serial_mod.PARITY_EVEN = "E"
    fake_serial_mod.PARITY_ODD = "O"
    fake_serial_mod.STOPBITS_ONE = 1
    fake_serial_mod.STOPBITS_TWO = 2

    # Simulate incoming serial buffer: first read(1) gets first byte, then in_waiting has the rest
    resp_frame = append_crc16_modbus(bytes([0x01, 0x03, 0x02, 0x09, 0x60]))
    serial_instance.read.side_effect = [resp_frame[:1], resp_frame[1:]]
    serial_instance.in_waiting = len(resp_frame) - 1

    with patch.dict(sys.modules, {"serial": fake_serial_mod}):
        comm = UniversalCommunicator(simulate=False)
        comm.connect("MODBUS_RTU", "COM4", simulate=False, baudrate=115200, framing="8E1")
        tx, rx = comm.transceive("01 03 00 00 00 01", mode="HEX", append_crc=True)
        assert verify_crc16_modbus(rx.raw_bytes)
        assert ModbusCodec.decode_registers(rx.raw_bytes[3:-2]) == [2400]

        # Also test CAN SLCAN initialization sequence (C\rS6\rO\r)
        serial_instance.write.reset_mock()
        comm.connect("CAN", "COM5", simulate=False, bitrate="500 kbps")
        serial_instance.write.assert_called_with(b"C\rS6\rO\r")
        comm.disconnect()


def test_real_camera_tcp_stream_and_http_bridge():
    """Verify end-to-end HTTP Hardware Bridge -> Real TCP Camera Frame Server (simulate=False)."""
    cam_srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cam_srv.bind(("127.0.0.1", 0))
    cam_srv.listen(1)
    cam_port = cam_srv.getsockname()[1]

    def cam_worker():
        conn, _ = cam_srv.accept()
        with conn:
            cmd = conn.recv(1024)
            if b"SNAP" in cmd:
                pkt = CameraFramePacket.pack_frame(b"\xff\xd8REAL_CAM_JPEG\xff\xd9", width=640, height=360, frame_seq=7)
                conn.sendall(pkt)
        cam_srv.close()

    t_cam = threading.Thread(target=cam_worker, daemon=True)
    t_cam.start()

    bridge_srv = HTTPServer(("127.0.0.1", 0), BridgeHandler)
    bridge_port = bridge_srv.server_port
    t_bridge = threading.Thread(target=bridge_srv.serve_forever, daemon=True)
    t_bridge.start()

    try:
        # 1. Connect bridge to the real TCP Camera server
        req_conn = urllib.request.Request(
            f"http://127.0.0.1:{bridge_port}/",
            data=json.dumps({
                "action": "connect",
                "protocol": "CAMERA_TCP",
                "endpoint": f"127.0.0.1:{cam_port}",
                "simulate": False,
            }).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req_conn, timeout=2.0) as resp:
            conn_data = json.loads(resp.read().decode("utf-8"))
            assert conn_data["status"] == "CONNECTED"
            assert conn_data["simulate"] is False

        # 2. Transceive SNAP command through bridge to real TCP camera server
        req_tx = urllib.request.Request(
            f"http://127.0.0.1:{bridge_port}/",
            data=json.dumps({
                "action": "transceive",
                "data": "SNAP",
                "mode": "ASCII",
                "line_ending": "\n",
            }).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req_tx, timeout=2.0) as resp:
            tx_data = json.loads(resp.read().decode("utf-8"))
            rx_raw = hex_to_bytes(tx_data["rx"]["hex"])
            width, height, seq, length = CameraFramePacket.unpack_header(rx_raw)
            assert width == 640
            assert height == 360
            assert seq == 7
            assert rx_raw[16:] == b"\xff\xd8REAL_CAM_JPEG\xff\xd9"
    finally:
        bridge_srv.shutdown()
        t_cam.join(timeout=2.0)
