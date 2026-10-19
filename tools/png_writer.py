"""Minimal PNG writer (RGB8, no dependencies) for tools/sdr_probe.py."""
import struct
import zlib


def write(path, width, height, rows, scale=3):
    """rows: list of `height` bytes objects with width*3 RGB bytes."""
    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data))
    raw = bytearray()
    for row in rows:
        wide = bytearray()
        for x in range(width):
            wide += row[3 * x:3 * x + 3] * scale
        for _ in range(scale):
            raw += b"\x00" + wide
    data = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width * scale, height * scale, 8, 2, 0, 0, 0))
    data += chunk(b"IDAT", zlib.compress(bytes(raw), 9)) + chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(data)
