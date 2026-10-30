"""decode_fast must equal decode on every 8-bit colour type, with and without tRNS, and the
banded premultiply must equal the whole-image formula (including rows across band edges).
usage: python -I test_pngdec_fast.py     exit 0 = all pass"""
import os
import random
import struct
import sys
import zlib

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pngdec  # noqa: E402

fails = 0


def check(name, ok):
    global fails
    print(("PASS " if ok else "FAIL ") + name)
    fails += not ok


def chunk(kind, body):
    return (struct.pack(">I", len(body)) + kind + body
            + struct.pack(">I", zlib.crc32(kind + body) & 0xFFFFFFFF))


def png(w, h, ctype, rows, plte=None, trns=None):
    raw = b"".join(bytes([random.randrange(5)]) + r for r in rows)   # random filter bytes
    # filter bytes are applied by the decoder; the encoder here writes data that is already
    # "filtered" by those types, which is fine: both decoders must agree on the result.
    out = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, ctype, 0, 0, 0))
    if plte is not None:
        out += chunk(b"PLTE", plte)
    if trns is not None:
        out += chunk(b"tRNS", trns)
    return out + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b"")


random.seed(3)
CH = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}
for ctype in (0, 2, 3, 4, 6):
    for with_trns in (False, True):
        if with_trns and ctype in (4, 6):
            continue
        w, h = 37, 23
        n = CH[ctype]
        plte = bytes(random.randrange(256) for _ in range(16 * 3)) if ctype == 3 else None
        rows = []
        for _ in range(h):
            if ctype == 3:
                rows.append(bytes(random.randrange(16) for _ in range(w)))
            else:
                rows.append(bytes(random.choice((0, 7, 200, random.randrange(256)))
                                  for _ in range(w * n)))
        trns = None
        if with_trns:
            trns = {0: struct.pack(">H", 7), 2: struct.pack(">HHH", 7, 7, 7),
                    3: bytes(random.randrange(256) for _ in range(9))}[ctype]
        data = png(w, h, ctype, rows, plte, trns)
        try:
            a = pngdec.decode(data)[2]
        except pngdec.PngError:
            a = None
        try:
            b = pngdec.decode_fast(data)[2]
        except pngdec.PngError:
            b = None
        ok = (a is None and b is None) or (a is not None and b is not None and np.array_equal(a, b))
        check(f"ctype {ctype} trns={with_trns}", ok)

img = np.random.RandomState(5).randint(0, 256, (1300, 61, 4)).astype(np.uint8)
img[::3, :, 3] = 255
a = img[:, :, 3].astype(np.int32)
ref = img.copy()
for c in range(3):
    ch = img[:, :, c].astype(np.int32)
    ref[:, :, c] = np.where(a < 255, ((a + 1) * ch) >> 8, ch).astype(np.uint8)
check("banded premultiply == whole-image formula (1300 rows, 3 bands)",
      np.array_equal(pngdec.premultiply(img), ref))

big = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", 40000, 30000, 8, 6, 0, 0, 0))
try:
    pngdec.decode_fast(big + chunk(b"IEND", b""))
    check("over-cap image raises TooLarge", False)
except pngdec.TooLarge:
    check("over-cap image raises TooLarge", True)

print("ALL PASS" if fails == 0 else f"{fails} FAILED")
sys.exit(1 if fails else 0)
