#!/usr/bin/env python3
"""Serve the bundled esp-web-sdr page on localhost and bridge its /ws
WebSocket to the board's USB serial port: the same page and transport the
firmware serves over WiFi, without joining the TTGO-SDR access point.

    python tools/build_web.py --dist build/web
    python tools/ws_bridge.py COM7 build/web          # http://localhost:8765

The serial port is opened without toggling DTR/RTS, so the board keeps
running. Requires: pip install pyserial websockets (>= 13).
"""
import asyncio
import mimetypes
import pathlib
import sys

import serial
from websockets.asyncio.server import serve
from websockets.datastructures import Headers
from websockets.http11 import Response

PORT, ROOT = sys.argv[1], pathlib.Path(sys.argv[2] if len(sys.argv) > 2 else "build/web")
HTTP_PORT = int(sys.argv[3]) if len(sys.argv) > 3 else 8765


def process_request(connection, request):
    if request.path == "/ws":
        return None  # WebSocket handshake
    name = request.path.split("?")[0].lstrip("/") or "index.html"
    path = (ROOT / name).resolve()
    if ROOT.resolve() not in path.parents or not path.is_file():
        return connection.respond(404, "not found\n")
    body = path.read_bytes()
    ctype = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
    return Response(200, "OK", Headers([("Content-Type", ctype), ("Content-Length", str(len(body))),
                                        ("Cache-Control", "no-cache")]), body)


async def bridge(ws):
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = PORT, 2000000, 0.05
    ser.dtr = ser.rts = False
    ser.open()
    loop = asyncio.get_running_loop()
    running = True

    async def device_to_browser():
        while running:
            data = await loop.run_in_executor(None, lambda: ser.read(max(1, ser.in_waiting)))
            if data:
                await ws.send(data)

    pump = asyncio.create_task(device_to_browser())
    try:
        async for message in ws:
            ser.write(message if isinstance(message, bytes) else message.encode())
    finally:
        running = False
        pump.cancel()
        await asyncio.sleep(0.1)
        ser.close()


async def main():
    async with serve(bridge, "127.0.0.1", HTTP_PORT, process_request=process_request, max_size=None):
        print(f"http://localhost:{HTTP_PORT}  ->  {PORT}", flush=True)
        await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())
