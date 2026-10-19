#!/usr/bin/env python3
"""Exercise the firmware's esp-sdr protocol over USB serial and measure
spectrum stability.

    python tools/sdr_probe.py COM7 protocol     # negotiation, CAP16/CAP20, SPEC
    python tools/sdr_probe.py COM7 jitter       # A/B: old vs new scan pipeline
    python tools/sdr_probe.py COM7 stat         # one TTGO STAT? line
    python tools/sdr_probe.py COM7 shot x.png   # screenshot of the TFT

The port is opened without toggling DTR/RTS, so the board is not reset.
"""
import json
import struct
import sys
import time
import zlib

import serial


class Link:
    def __init__(self, port, baud=2000000):
        self.s = serial.Serial()
        self.s.port, self.s.baudrate, self.s.timeout = port, baud, 0.05
        self.s.dtr = self.s.rts = False
        self.s.open()
        self.buf = b""

    def read(self, n, timeout=5.0):
        end = time.time() + timeout
        while len(self.buf) < n:
            if time.time() > end:
                raise TimeoutError(f"wanted {n} bytes, have {len(self.buf)}")
            self.buf += self.s.read(max(1, min(65536, self.s.in_waiting or 1)))
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def line(self, timeout=5.0):
        end = time.time() + timeout
        while b"\n" not in self.buf:
            if time.time() > end:
                raise TimeoutError("no line: %r" % self.buf[-80:])
            self.buf += self.s.read(max(1, self.s.in_waiting or 1))
        i = self.buf.index(b"\n")
        out, self.buf = self.buf[:i], self.buf[i + 1:]
        return out.decode("latin-1").strip()

    def cmd(self, text):
        self.s.write(text.encode() + b"\n")

    def ask(self, text, timeout=5.0):
        self.cmd(text)
        while True:
            reply = self.line(timeout)
            if not reply.startswith("["):  # console log lines between sessions
                return reply

    def sync(self):
        marker = "SYNC %d" % int(time.time() * 1000)
        self.s.write(b"\n" + marker.encode() + b"\n")
        end = time.time() + 4
        while time.time() < end:
            try:
                if self.line(1) == marker:
                    return
            except TimeoutError:
                pass
        raise RuntimeError("SYNC failed")


def check(cond, what):
    print(("  ok   " if cond else "  FAIL ") + what)
    if not cond:
        check.failed += 1


check.failed = 0


def protocol(link):
    link.sync()
    print("negotiation (the order esp-web-sdr radio.js uses)")
    info = link.ask("INFO")
    check(info == "ESP32SDR 6 burst 16380", f"INFO -> {info}")
    caps = link.ask("CAPS").split()
    check(all(c in caps for c in "SPEC SPECN SPECCAPS UARTBAUD RXLIMITS TUNEEXT GAIN HWAGC VERSION".split()),
          "CAPS " + " ".join(caps[1:]))
    t = link.ask("TRANSPORT?")
    check(t == "TRANSPORT UART 2000000", f"TRANSPORT? -> {t}")
    r = link.ask("RANGE?")
    check(r.startswith("RANGE ") and r.endswith(" 1"), f"RANGE? -> {r}")
    lim = link.ask("LIMITS?")
    j = json.loads(lim[7:])
    check(j["rates"] == [80000000, 40000000, 16000000] and j["gain"][1] > 0, f"LIMITS? -> {lim}")
    v = link.ask("VERSION?")
    vj = json.loads(v[8:])
    check(vj["profile"] == "esp32" and len(vj["build_timestamp"]) == 20, f"VERSION? -> {v}")
    si = link.ask("SPECINFO?")
    sj = json.loads(si[9:])
    check(len(sj["profiles"]) == 12 and sj["continuous"] is False, f"SPECINFO? -> {len(sj['profiles'])} profiles")
    check(link.ask("FREQ 2437") == "OK", "FREQ 2437")
    check(link.ask("BANDWIDTH 20") == "OK", "BANDWIDTH 20")
    check(link.ask("GAIN MANUAL 40") == "OK", "GAIN MANUAL 40")
    g = link.ask("GAIN?")
    check(g.startswith("GAIN MANUAL 40 0 "), f"GAIN? -> {g}")
    check(link.ask("GAIN HARDWARE") == "OK", "GAIN HARDWARE")
    check(link.ask("BOGUS") == "ERR command", "unknown command -> ERR command")

    print("I/Q captures")
    for bits, n in ((10, 16380), (8, 16380), (10, 4096)):
        t0 = time.time()
        link.cmd(f"CAP{bits * 2} {n} 0")
        h = link.line().split()
        size = (n * 20 + 7) // 8 if bits == 10 else n * 2
        data = link.read(size)
        ok = h[0] == "DATA" and int(h[1]) == n and zlib.crc32(data) == int(h[2], 16)
        check(ok, f"CAP{bits * 2} {n}: {size} bytes, CRC {'match' if ok else 'MISMATCH'}, "
                  f"capture {h[3]} us, {1000 * (time.time() - t0):.0f} ms total")

    print("on-chip spectra (SPEC)")
    for rate, n in ((0, 2048), (1, 256), (6, 1024)):
        link.cmd(f"SPEC 0 1 1 0 {rate} {n} 1")
        head = link.line().split()
        check(len(head) == 5 and int(head[1]) == n and head[4] == "2437", "SPEC header " + " ".join(head))
        frames = stats = bad = ffts = 0
        t0 = time.time()
        buf, link.buf = link.buf, b""
        while time.time() - t0 < 2.0:
            buf += link.s.read(max(1, link.s.in_waiting))
            while len(buf) >= 4:
                tag = buf[:4]
                if tag == b"SPC1":
                    if len(buf) < 32 + n:
                        break
                    body = buf[:28 + n]
                    crc = struct.unpack_from("<I", buf, 28 + n)[0]
                    if zlib.crc32(body) == crc and buf[26] == n.bit_length() - 1 and buf[27] == 2:
                        frames += 1
                        ffts = struct.unpack_from("<H", buf, 20)[0]
                        buf = buf[32 + n:]
                    else:
                        bad += 1
                        buf = buf[1:]
                elif tag == b"SPS1":
                    if len(buf) < 40:
                        break
                    if zlib.crc32(buf[:36]) == struct.unpack_from("<I", buf, 36)[0]:
                        stats += 1
                        buf = buf[40:]
                    else:
                        bad += 1
                        buf = buf[1:]
                else:
                    bad += 1
                    buf = buf[1:]
        if "-v" in sys.argv:
            print(f"    debug: frames={frames} bad={bad} pending={len(buf)} tail={buf[-60:]!r}")
        link.cmd("")
        link.buf = buf  # the tail of the stream ends with the SPECEND line
        end = link.line(3)
        while "SPECEND" not in end:
            end = link.line(3)
        end = end[end.index("SPECEND"):]
        check(frames > 5 and bad == 0 and end.split()[1] == "0",
              f"{n} bins @ code {rate}: {frames / 2:.0f} frames/s, {ffts} FFT/frame, {stats} SPS1, "
              f"{bad} bad bytes; {end}")
    check(link.ask("RELEASE") == "OK", "RELEASE")


def screenshot(link, path):
    """Save the TFT contents (exact pixels the display shows) as a PNG."""
    sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parent))
    import png_writer
    link.cmd("TTGO SHOT?")
    while True:
        head = link.line(5)
        if head.startswith("TTGO SHOT"):
            break
    w, h = map(int, head.split()[2:4])
    raw = link.read(w * h * 2, 10)
    rows = []
    for y in range(h):
        row = bytearray()
        for x in range(w):
            v = raw[2 * (y * w + x)] << 8 | raw[2 * (y * w + x) + 1]
            r, g, b = (v >> 11) & 31, (v >> 5) & 63, v & 31
            row += bytes((r * 255 // 31, g * 255 // 63, b * 255 // 31))
        rows.append(bytes(row))
    png_writer.write(path, w, h, rows)


def stat(link):
    return json.loads(link.ask("TTGO STAT?")[10:])


def jitter(link, seconds=12):
    link.sync()
    link.ask("RELEASE")
    print(f"frame-to-frame jitter of the local 2.4 GHz scan, {seconds} s per case")
    print("  mean |dB(t)-dB(t-1)| per column; 'screen' = the trace as drawn on the TFT")
    rows = []
    for name, agc, gain, legacy, avg in (("previous firmware: old scan, AGC, no avg", 1, 40, 1, 0),
                                         ("old scan, manual gain 40, no avg", 0, 40, 1, 0),
                                         ("new scan, AGC, no avg", 1, 40, 0, 0),
                                         ("new scan, manual gain 40, no avg", 0, 40, 0, 0),
                                         ("new scan, AGC, avg 1.5 s", 1, 40, 0, 2),
                                         ("new firmware default: manual 40, avg 1.5 s", 0, 40, 0, 2)):
        assert link.ask(f"TTGO LOCAL 0 {agc} {gain} {legacy}") == "OK"
        assert link.ask(f"TTGO AVG {avg}") == "OK"
        time.sleep(1.5)
        stat(link)  # reset the window
        time.sleep(seconds)
        st = stat(link)
        rows.append((name, st))
        print(f"  {name:44s} raw {st['jitter_raw']:5.2f} dB  screen {st['jitter_screen']:5.2f} dB  "
              f"floor {st['floor']:6.1f} dBFS  {st['frames'] / seconds:4.1f} fps  "
              f"gain idx {st['gain'][0]}..{st['gain'][1]}  AGC moved mid-capture {st['agc_moves']}x")
    assert link.ask("TTGO LOCAL 0 0 40 0") == "OK"
    assert link.ask("TTGO AVG 2") == "OK"
    return rows


if __name__ == "__main__":
    port, what = sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "protocol"
    link = Link(port)
    if what == "protocol":
        protocol(link)
        sys.exit(1 if check.failed else 0)
    elif what == "jitter":
        jitter(link)
    elif what == "shot":
        link.sync()
        link.ask("RELEASE")
        screenshot(link, sys.argv[3] if len(sys.argv) > 3 else "screen.png")
    else:
        link.sync()
        print(stat(link))
