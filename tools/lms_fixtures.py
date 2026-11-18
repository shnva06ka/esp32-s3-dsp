#!/usr/bin/env python3
"""Independent fixture generator for the LMS (SlimProto) client tests.

Builds the expected wire bytes straight from docs/lms_protocol_spec.md using struct,
so the C++ tests in test/test_lms compare against bytes that did NOT come from the C++ code.
Run:  python tools/lms_fixtures.py          (prints C arrays)
      python tools/lms_fixtures.py --check  (asserts the arrays embedded in test_main.cpp match)
"""
import re
import struct
import sys
from pathlib import Path

MAC = bytes([0x02, 0x11, 0x22, 0x33, 0x44, 0x55])
CAPS = b"flc,pcm,MaxSampleRate=96000"


def helo(mac=MAC, caps=CAPS):
    payload = struct.pack(">BB6s16sHQ2s", 12, 0, mac, bytes(16), 0x07FF, 0, b"en") + caps
    return b"HELO" + struct.pack(">I", len(payload)) + payload


def stat(event=b"STMt"):
    payload = struct.pack(
        ">4sBBBIIQHIIIIHIIH",
        event, 0, ord("m"), 0,
        2097152, 1000, 123456789012, 70, 5000,
        786432, 4096, 12, 0, 1500, 0xCAFEBABE, 0,
    )
    assert len(payload) == 53
    return b"STAT" + struct.pack(">I", len(payload)) + payload


def server_frame(op, data):
    return struct.pack(">H", 4 + len(data)) + op + data


def strm_start():
    http = b"GET /stream.flac?player=02:11:22:33:44:55 HTTP/1.0\r\n\r\n"
    data = struct.pack(
        ">cccccccBBBcBBBIHI",
        b"s", b"1", b"f", b"?", b"?", b"?", b"?",
        255, 0, 0, b"0", 0x41, 20, 0,
        0x00018000, 9000, 0,
    ) + http
    assert len(data) - len(http) == 24
    return server_frame(b"strm", data)


def audg():
    data = struct.pack(">IIBBII", 100, 100, 1, 255, 0x00008000, 0x00010000)
    return server_frame(b"audg", data)


def c_array(name, b):
    body = ", ".join("0x%02X" % x for x in b)
    return "static const uint8_t %s[%d] = {%s};" % (name, len(b), body)


FIXTURES = {
    "kFixHelo": helo(),
    "kFixStat": stat(),
    "kFixStrm": strm_start(),
    "kFixAudg": audg(),
}

if __name__ == "__main__":
    if "--check" in sys.argv:
        src = (Path(__file__).resolve().parent.parent / "test" / "test_lms" / "test_main.cpp").read_text()
        ok = True
        for name, b in FIXTURES.items():
            if c_array(name, b) not in src:
                print("MISMATCH:", name)
                ok = False
        print("fixtures match" if ok else "fixtures differ")
        sys.exit(0 if ok else 1)
    for name, b in FIXTURES.items():
        print(c_array(name, b))
