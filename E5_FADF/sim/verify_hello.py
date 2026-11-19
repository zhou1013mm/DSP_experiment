#!/usr/bin/env python3
"""verify_hello.py -- decode + validate a CCES simulator memory dump.

CCES memory export format (IDE "Export Memory" / CCES_runner -d):
  * plain text, ONE value per line, LITTLE-ENDIAN byte order,
  * each line is one byte: 2 hex digits ("3f", "0x3f") or decimal (0..255),
  * no address column, no header.

Because a 2-char line such as "80" is ambiguous (hex 0x80 vs decimal 80), the
parser auto-detects the format for the WHOLE file: if any data line contains a
hex letter or a 0x prefix the file is read as hex, otherwise as decimal.
A change of representation inside one file is rejected as an error.

Layout used by sim_hello.c (arrays are adjacent in DM):
  offset   0..255 : float32 gOut[64]      (gOut  @ 0x241330)
  offset 256..271 : uint32  gCycles[4]    (gCycles @ 0x241430)
     gCycles[1] == 49995000, [2] == 0xFFFFFFFF, [3] == 10000
     gCycles[0] == measured cycles of the 10000-iteration loop

Usage:  python verify_hello.py [hello_out.txt]
"""
import re
import struct
import sys

OUT_LEN = 64
HEX_LINE = re.compile(r"^(?:0x)?[0-9a-fA-F]{1,2}$")


def read_lines(path):
    vals = []
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for lineno, line in enumerate(fh, 1):
            s = line.strip()
            if not s or s[0] in "#/":
                continue
            s = s.split()[0].rstrip(",")
            stop = s.find(";")
            if stop > 0:
                s = s[:stop]
            if re.fullmatch(r"0[xX][0-9a-fA-F]+", s):
                ok, base = True, 16
            elif re.fullmatch(r"[0-9a-fA-F]{1,2}", s):
                ok, base = True, (16 if re.search(r"[a-fA-F]", s) else None)
            elif re.fullmatch(r"[0-9]{1,3}", s):
                ok, base = True, 10
            else:
                ok, base = False, None
            if not ok:
                raise SystemExit("unparseable value on line %d: %r" % (lineno, line))
            vals.append((lineno, s, base))
    # whole-file format decision
    has_hex_hint = any(s.lower().startswith("0x") or re.search(r"[a-fA-F]", s)
                       for _, s, _ in vals)
    out = bytearray()
    for lineno, s, base in vals:
        if s.lower().startswith("0x") or (has_hex_hint and re.fullmatch(r"[0-9a-fA-F]{1,2}", s)):
            v = int(s, 16)
        else:
            v = int(s, 10)
        if not 0 <= v <= 0xFF:
            raise SystemExit("out of range on line %d: %r" % (lineno, s))
        out.append(v)
    return bytes(out)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "hello_out.txt"
    raw = read_lines(path)
    print("%s: %d bytes" % (path, len(raw)))
    if len(raw) < OUT_LEN * 4:
        raise SystemExit("too short: need at least gOut[64] (256 B)")

    got = struct.unpack("<%df" % OUT_LEN, raw[: OUT_LEN * 4])
    exp = [i * 0.5 for i in range(OUT_LEN)]
    bad = [(i, got[i], exp[i]) for i in range(OUT_LEN)
           if abs(got[i] - exp[i]) > 1e-5]
    print("gOut[0..3]   = %s" % [round(v, 4) for v in got[:4]])
    print("gOut[60..63] = %s" % [round(v, 4) for v in got[60:]])
    print("gOut pattern : %s" % ("PASS" if not bad else "FAIL %s" % bad[:5]))

    if len(raw) >= (OUT_LEN * 4) + 16:
        cyc = struct.unpack("<4I", raw[OUT_LEN * 4: OUT_LEN * 4 + 16])
        print("gCycles      = [%d, %d, 0x%08X, %d]" % cyc)
        ok_meta = (cyc[1] == 49995000 and cyc[2] == 0xFFFFFFFF and cyc[3] == 10000)
        print("gCycles meta : %s" % ("PASS" if ok_meta else "FAIL"))
        n = cyc[0]
        if n == 0:
            print("cycles       : 0 -- functional simulator does not model the"
                  " SHARC core cycle counter (EMUCLK); use Cycle-Accurate-Sim")
        elif 20000 <= n <= 5000000:
            print("cycles       : %d for the 10000-iteration loop -- plausible"
                  " for -O0 Debug SHARC code" % n)
        else:
            print("cycles       : %d -- OUTSIDE the expected range 20000..5000000" % n)
    else:
        print("gCycles      : not present (export length was only gOut)")


if __name__ == "__main__":
    main()
