"""Minimal dependency-free PNG reader/writer (8-bit, non-interlaced).

Reads colour types 0 (grey), 2 (RGB), 3 (palette, with tRNS), 4 (grey+alpha),
6 (RGBA). Returns (width, height, rows) where rows[y] is a list of (r,g,b,a).
Anything else (16-bit, interlaced, low bit depth) raises ValueError naming why,
rather than producing a wrong image.
"""
import struct
import zlib

_SIG = b"\x89PNG\r\n\x1a\n"


def _paeth(a, b, c):
    p = a + b - c
    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    return b if pb <= pc else c


def read_png(path):
    with open(path, "rb") as f:
        data = f.read()
    if data[:8] != _SIG:
        raise ValueError(f"{path}: not a PNG")
    pos = 8
    idat = b""
    plte = None
    trns = None
    width = height = depth = ctype = interlace = None
    while pos < len(data):
        (length,) = struct.unpack(">I", data[pos:pos + 4])
        kind = data[pos + 4:pos + 8]
        body = data[pos + 8:pos + 8 + length]
        pos += 12 + length
        if kind == b"IHDR":
            width, height, depth, ctype, _, _, interlace = struct.unpack(">IIBBBBB", body)
        elif kind == b"PLTE":
            plte = [tuple(body[i:i + 3]) for i in range(0, len(body), 3)]
        elif kind == b"tRNS":
            trns = body
        elif kind == b"IDAT":
            idat += body
        elif kind == b"IEND":
            break
    if depth != 8:
        raise ValueError(f"{path}: bit depth {depth} not supported (8 only)")
    if interlace:
        raise ValueError(f"{path}: interlaced PNG not supported")
    channels = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}.get(ctype)
    if channels is None:
        raise ValueError(f"{path}: colour type {ctype} not supported")
    raw = zlib.decompress(idat)
    stride = width * channels
    rows = []
    prev = bytearray(stride)
    p = 0
    for _ in range(height):
        ftype = raw[p]
        line = bytearray(raw[p + 1:p + 1 + stride])
        p += 1 + stride
        bpp = channels
        for i in range(stride):
            a = line[i - bpp] if i >= bpp else 0
            b = prev[i]
            c = prev[i - bpp] if i >= bpp else 0
            if ftype == 1:
                line[i] = (line[i] + a) & 0xFF
            elif ftype == 2:
                line[i] = (line[i] + b) & 0xFF
            elif ftype == 3:
                line[i] = (line[i] + ((a + b) >> 1)) & 0xFF
            elif ftype == 4:
                line[i] = (line[i] + _paeth(a, b, c)) & 0xFF
            elif ftype != 0:
                raise ValueError(f"{path}: bad filter type {ftype}")
        prev = line
        out = []
        for x in range(width):
            px = line[x * channels:(x + 1) * channels]
            if ctype == 6:
                out.append(tuple(px))
            elif ctype == 2:
                out.append((px[0], px[1], px[2], 255))
            elif ctype == 0:
                out.append((px[0], px[0], px[0], 255))
            elif ctype == 4:
                out.append((px[0], px[0], px[0], px[1]))
            else:  # palette
                idx = px[0]
                r, g, b = plte[idx]
                a = trns[idx] if trns is not None and idx < len(trns) else 255
                out.append((r, g, b, a))
        rows.append(out)
    return width, height, rows


def write_png(path, width, height, rows):
    """rows[y][x] = (r,g,b,a)."""
    raw = bytearray()
    for y in range(height):
        raw.append(0)
        for r, g, b, a in rows[y]:
            raw += bytes((r, g, b, a))

    def chunk(kind, body):
        return (struct.pack(">I", len(body)) + kind + body
                + struct.pack(">I", zlib.crc32(kind + body) & 0xFFFFFFFF))

    with open(path, "wb") as f:
        f.write(_SIG)
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(bytes(raw), 9)))
        f.write(chunk(b"IEND", b""))
