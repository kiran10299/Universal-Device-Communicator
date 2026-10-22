import pytest
from omnicomm.core import (
    CameraFramePacket,
    CanFrame,
    ModbusCodec,
    append_crc16_modbus,
    bytes_to_hex,
    crc16_modbus,
    hex_to_bytes,
    verify_crc16_modbus,
)
from omnicomm.transports import UniversalCommunicator


def test_crc16_modbus_standard_vector():
    # Standard Modbus RTU Read Holding Registers (Slave 1, FC 03, Addr 0x0000, Count 10)
    # 01 03 00 00 00 0A -> CRC-16 little-endian is C5 CD
    pdu = hex_to_bytes("01 03 00 00 00 0A")
    framed = append_crc16_modbus(pdu)
    assert bytes_to_hex(framed) == "01 03 00 00 00 0A C5 CD"
    assert verify_crc16_modbus(framed) is True


def test_scpi_transceive_idn_and_voltage():
    comm = UniversalCommunicator(simulate=True)
    comm.connect("SCPI", "TCPIP0::192.168.1.50::5025::SOCKET")
    tx, rx = comm.transceive("*IDN?")
    assert "KEYSIGHT" in rx.ascii_str
    tx2, rx2 = comm.transceive(":MEAS:VOLT:DC?")
    assert "E+" in rx2.ascii_str or "E-" in rx2.ascii_str
    comm.disconnect()


def test_modbus_rtu_read_and_write_registers():
    comm = UniversalCommunicator(simulate=True)
    comm.connect("MODBUS_RTU", "COM4", baudrate=115200)

    # Write single holding register 0x0000 = 3300 (330.0V)
    write_frame = ModbusCodec.build_rtu_write_single(1, 6, 0, 3300)
    comm.write(write_frame)
    rx_write = comm.read(last_tx=write_frame)
    assert rx_write.raw_bytes == write_frame

    # Read back registers 0..3
    read_frame = ModbusCodec.build_rtu_read_request(1, 3, 0, 4)
    comm.write(read_frame)
    rx_read = comm.read(last_tx=read_frame)
    assert verify_crc16_modbus(rx_read.raw_bytes)
    regs = ModbusCodec.decode_registers(rx_read.raw_bytes[3:-2])
    assert regs[0] == 3300
    assert len(regs) == 4


def test_can_slcan_codec():
    frame = CanFrame(arbitration_id=0x123, data=bytes.fromhex("DEADBEEF"), is_extended=False)
    slcan = frame.to_slcan()
    assert slcan == "t1234DEADBEEF\r"
    parsed = CanFrame.from_slcan(slcan)
    assert parsed.arbitration_id == 0x123
    assert parsed.dlc == 4
    assert parsed.data == bytes.fromhex("DEADBEEF")


def test_camera_tcp_binary_packetizer():
    comm = UniversalCommunicator(simulate=True)
    comm.connect("CAMERA_TCP", "127.0.0.1:8090")
    tx = comm.write(b"SNAP\n")
    rx = comm.read(last_tx=tx.raw_bytes)
    w, h, seq, length = CameraFramePacket.unpack_header(rx.raw_bytes[:16])
    assert (w, h, seq) == (640, 480, 1)
    assert len(rx.raw_bytes[16:]) == length
