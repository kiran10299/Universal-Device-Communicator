# 🛰️ OmniComm Studio — Universal Hardware, Protocol & Vision Communicator

<p align="left">
  <a href="https://github.com/kiran10299/Universal-Device-Communicator/actions/workflows/ci.yml"><img src="https://img.shields.io/github/actions/workflow/status/kiran10299/Universal-Device-Communicator/ci.yml?branch=master&style=for-the-badge&logo=github-actions&logoColor=white&label=CI%20Pipeline" alt="CI Pipeline" /></a>
  <a href="https://kiran10299.github.io/omnicomm.html"><img src="https://img.shields.io/badge/Live%20Web%20App-OmniComm%20Studio-22d3ee?style=for-the-badge&logo=googlechrome&logoColor=white" alt="Live Web Workbench" /></a>
  <a href="https://python.org"><img src="https://img.shields.io/badge/Python-3.9+-3776AB?style=for-the-badge&logo=python&logoColor=white" alt="Python 3.9+" /></a>
  <a href="https://pytest.org"><img src="https://img.shields.io/badge/Tests-5%2F5%20Passing-10b981?style=for-the-badge&logo=pytest&logoColor=white" alt="Pytest Passing" /></a>
  <a href="https://opensource.org/licenses/MIT"><img src="https://img.shields.io/badge/License-MIT-f59e0b?style=for-the-badge" alt="MIT License" /></a>
</p>

**OmniComm Studio** (`Universal-Device-Communicator`) is an open-source **"Postman for Hardware, Industrial Fieldbuses & Machine Vision Cameras"**. Instead of switching between 7 different utilities (NI MAX, PuTTY, Modbus Poll, PacketSender, PCAN-View, AMCap), engineers can connect, **Read**, **Write**, **Query**, and inspect raw hex/ASCII frames across every major instrument and industrial protocol in one unified interface.

👉 **Try the Live Browser Workbench (Supports WebSerial, WebRTC USB Cameras & Virtual Simulator):**  
**[https://kiran10299.github.io/omnicomm.html](https://kiran10299.github.io/omnicomm.html)**

---

## 🌐 Supported Protocols & Hardware Modes

| Category | Protocol / Interface | Key Capabilities |
| :--- | :--- | :--- |
| **Test Instruments** | **SCPI / VISA** (`TCP :5025`, `GPIB`, `USB-TMC`) | `*IDN?`, `*RST`, `:MEAS:VOLT:DC?`, `:SYST:ERR?`, IEEE 488.2 status polling |
| **Serial Bus** | **RS-232 / UART / RS-485** | Native browser **WebSerial API** + `pyserial`, `9600`–`921600` baud, `8N1`/`8E1`/`7E1` |
| **Industrial PLC** | **Modbus RTU** (RS-485) | `FC01`–`FC16`, automatic little-endian `CRC-16/MODBUS` generation & verification |
| **Industrial Ethernet** | **Modbus TCP** (`:502`) | 7-byte MBAP header builder, Unit ID routing, live `40001–40008` register map |
| **Automotive / Robotics** | **CAN Bus 2.0A / 2.0B** | Lawicel `SLCAN`, SocketCAN, 11-bit standard & 29-bit extended frames, 8-byte DLC |
| **Network Sockets** | **TCP Client / Stream** | Persistent socket, ASCII / HEX / binary struct payloads |
| **Datagram** | **UDP Unicast & Broadcast** | Low-latency telemetry datagrams, node discovery broadcasts |
| **Legacy Console** | **Telnet (`:23`)** | Interactive CLI command execution with `CRLF`/`LF` termination |
| **Machine Vision** | **Cameras (USB / DirectShow / TCP)** | Browser **WebRTC USB Camera**, OpenCV `CAP_DSHOW`, and binary `OMNI` length-prefixed TCP frame streams |

---

## 📺 CLI Multi-Protocol Demo Output

```text
====================================================================================
  OmniComm Studio | Universal Hardware & Protocol Communicator v1.0.0
====================================================================================
  [SCPI]       TX: *IDN?\n                    | RX: KEYSIGHT TECHNOLOGIES,34465A,MY59001284,A.03.01-03.15\n
  [SCPI]       TX: :MEAS:VOLT:DC?\n           | RX: +5.000908E+00\n
  [MODBUS_RTU] TX: 01 03 00 00 00 04 44 09    | RX: 01 03 08 09 01 02 00 13 86 01 56 20 5C -> Regs=[2305, 512, 4998, 342]
  [CAN_BUS]    TX: t1234DEADBEEF\r            | RX: T18FF50E58102405DC0100AA55\r
  [CAMERA_TCP] TX: SNAP\n                     | RX: 42 bytes (Header=OMNI 640x480)
====================================================================================
  Total Packets Logged: 10 | Status: ALL PROTOCOLS VERIFIED
====================================================================================
```

---

## 🚀 Quickstart

### 1. Use Immediately in Browser (Zero Install)
Open **[https://kiran10299.github.io/omnicomm.html](https://kiran10299.github.io/omnicomm.html)** in Chrome or Edge:
- Click **Virtual Hardware Simulator** to test SCPI, Modbus RTU/TCP, CAN, TCP/UDP, and synthetic camera streams immediately.
- Click **Direct Browser Hardware** to connect to physical **USB-Serial (RS-232/RS-485)** adapters via WebSerial or physical **USB Webcams** via WebRTC.

### 2. Run Python SDK / Local OS Hardware Bridge
```bash
git clone https://github.com/kiran10299/Universal-Device-Communicator.git
cd Universal-Device-Communicator
pip install -r requirements.txt

# Run automated multi-protocol demo
python omnicomm_cli.py --demo

# Start local hardware bridge on http://127.0.0.1:8765 for raw TCP/UDP/SCPI/DirectShow
python omnicomm_cli.py --serve

# Run pytest suite
pytest -v tests/
```

---

## 📄 License
MIT License — Created by **[Kiran Shivakumar](https://kiran10299.github.io)**.
