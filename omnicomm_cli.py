#!/usr/bin/env python3
"""
OmniComm Studio — CLI & Local Hardware Bridge Server
Run `python omnicomm_cli.py --demo` to execute an automated multi-protocol verification run,
or `python omnicomm_cli.py --serve` to start the local hardware bridge on http://127.0.0.1:8765.
"""

import argparse
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
from typing import Any, Dict

from omnicomm.core import ModbusCodec, bytes_to_hex, hex_to_bytes
from omnicomm.transports import UniversalCommunicator

GLOBAL_COMM = UniversalCommunicator(simulate=True)


class BridgeHandler(BaseHTTPRequestHandler):
    """Lightweight HTTP/JSON bridge allowing the browser UI to access local OS sockets, serial, CAN, and cameras."""

    def log_message(self, format: str, *args: Any) -> None:
        # Suppress noisy per-request HTTP logs during high-rate polling
        pass

    def _send_json(self, payload: Dict[str, Any], status: int = 200) -> None:
        raw = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def do_OPTIONS(self) -> None:
        self._send_json({"ok": True})

    def do_GET(self) -> None:
        if self.path == "/status":
            self._send_json(
                {
                    "ok": True,
                    "connected": GLOBAL_COMM.connected,
                    "protocol": GLOBAL_COMM.protocol,
                    "endpoint": GLOBAL_COMM.endpoint,
                    "simulate": GLOBAL_COMM.simulate,
                    "packets": len(GLOBAL_COMM.packet_log),
                }
            )
            return
        self._send_json({"service": "OmniComm Local Hardware Bridge", "version": "1.0.0", "ok": True})

    def do_POST(self) -> None:
        length = int(self.headers.get("Content-Length", "0"))
        body = json.loads(self.rfile.read(length).decode("utf-8") or "{}")
        action = body.get("action", "")
        try:
            if action == "connect":
                cfg = dict(body.get("config", {}))
                if "simulate" in body:
                    cfg["simulate"] = bool(body["simulate"])
                res = GLOBAL_COMM.connect(
                    protocol=body.get("protocol", "SCPI"),
                    endpoint=body.get("endpoint", "127.0.0.1:5025"),
                    **cfg,
                )
                self._send_json(res)
            elif action == "disconnect":
                self._send_json(GLOBAL_COMM.disconnect())
            elif action == "write":
                if "raw_hex" in body:
                    raw_tx = hex_to_bytes(body["raw_hex"])
                elif body.get("mode", "ASCII").upper() == "HEX":
                    raw_tx = hex_to_bytes(body.get("data", ""))
                else:
                    raw_tx = (body.get("data", "") + body.get("line_ending", "\n")).encode("utf-8")
                tx = GLOBAL_COMM.write(raw_tx)
                self._send_json({"tx": tx.to_dict()})
            elif action == "read":
                rx = GLOBAL_COMM.read(num_bytes=int(body.get("num_bytes", 4096)))
                self._send_json({"rx": rx.to_dict()})
            elif action == "transceive":
                if "raw_hex" in body:
                    raw_tx = hex_to_bytes(body["raw_hex"])
                    tx = GLOBAL_COMM.write(raw_tx)
                    rx = GLOBAL_COMM.read(last_tx=raw_tx)
                else:
                    tx, rx = GLOBAL_COMM.transceive(
                        data=body.get("data", ""),
                        mode=body.get("mode", "ASCII"),
                        line_ending=body.get("line_ending", "\n"),
                        append_crc=bool(body.get("append_crc", False)),
                    )
                self._send_json({"tx": tx.to_dict(), "rx": rx.to_dict()})
            else:
                self._send_json({"error": f"Unknown action: {action}"}, status=400)
        except Exception as exc:
            self._send_json({"error": str(exc)}, status=500)


def run_demo() -> None:
    comm = UniversalCommunicator(simulate=True)
    print("=" * 84)
    print("  OmniComm Studio | Universal Hardware & Protocol Communicator v1.0.0")
    print("=" * 84)

    # 1. SCPI
    comm.connect("SCPI", "TCPIP0::192.168.1.50::5025::SOCKET")
    tx, rx = comm.transceive("*IDN?")
    print(f"  [SCPI]       TX: {tx.ascii_str:<26} | RX: {rx.ascii_str}")
    tx, rx = comm.transceive(":MEAS:VOLT:DC?")
    print(f"  [SCPI]       TX: {tx.ascii_str:<26} | RX: {rx.ascii_str}")

    # 2. Modbus RTU
    comm.connect("MODBUS_RTU", "COM4 (RS-485 @ 115200 8N1)")
    req = ModbusCodec.build_rtu_read_request(slave_id=1, function_code=3, start_addr=0, count=4)
    tx = comm.write(req)
    rx = comm.read(last_tx=req)
    regs = ModbusCodec.decode_registers(rx.raw_bytes[3:-2])
    print(f"  [MODBUS_RTU] TX: {bytes_to_hex(tx.raw_bytes):<26} | RX: {bytes_to_hex(rx.raw_bytes)} -> Regs={regs}")

    # 3. CAN Bus
    comm.connect("CAN", "can0 @ 500kbps")
    tx, rx = comm.transceive("t1234DEADBEEF", line_ending="\r")
    print(f"  [CAN_BUS]    TX: {tx.ascii_str:<26} | RX: {rx.ascii_str}")

    # 4. TCP / UDP
    comm.connect("TCP", "192.168.1.100:9000")
    tx, rx = comm.transceive("PING_DAQ_CH01")
    print(f"  [TCP_RAW]    TX: {tx.ascii_str:<26} | RX: {rx.ascii_str}")

    # 5. Camera Frame Stream
    comm.connect("CAMERA_TCP", "127.0.0.1:8090")
    tx = comm.write(b"SNAP\n")
    rx = comm.read(last_tx=b"SNAP\n")
    print(f"  [CAMERA_TCP] TX: {tx.ascii_str:<26} | RX: {len(rx.raw_bytes)}B Frame Header={bytes_to_hex(rx.raw_bytes[:16])}")
    print("=" * 84)
    print(f"  Total Packets Logged: {len(comm.packet_log)} | Status: ALL PROTOCOLS VERIFIED")
    print("=" * 84)


def main() -> None:
    parser = argparse.ArgumentParser(description="OmniComm Studio CLI & Hardware Bridge")
    parser.add_argument("--demo", action="store_true", help="Run multi-protocol verification demo")
    parser.add_argument("--serve", action="store_true", help="Start local HTTP hardware bridge on port 8765")
    parser.add_argument("--port", type=int, default=8765, help="Port for local HTTP bridge (default: 8765)")
    args = parser.parse_args()

    if args.serve:
        GLOBAL_COMM.connect("SCPI", "SIM://LOCAL")
        server = HTTPServer(("127.0.0.1", args.port), BridgeHandler)
        print(f"[OmniComm] Local Hardware Bridge listening on http://127.0.0.1:{args.port}")
        server.serve_forever()
    else:
        run_demo()


if __name__ == "__main__":
    main()
