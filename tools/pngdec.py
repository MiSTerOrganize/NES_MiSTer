"""PNG decoder that reproduces Mesen2's PNGHelper::DecodePNG.

Mesen decodes with libspng: SPNG_FMT_RGBA8, SPNG_DECODE_TRNS, CRC errors ignored
(SPNG_CRC_USE), no gamma. So:
  - every colour type and bit depth (1/2/4/8/16), Adam7 interlace;
  - 16-bit samples become 8-bit by taking the high byte;
  - grey of depth < 8 is scaled to 0..255 (1 -> x255, 2 -> x85, 4 -> x17);
  - tRNS: grey/RGB pixels whose RAW samples equal the tRNS value get alpha 0; palette entries
    take their alpha from tRNS (255 past its end);
  - gAMA/sRGB/iCCP are ignored.
Returns (width, height, numpy uint8 array [h, w, 4] RGBA). Raises PngError on anything spng
would refuse (bad signature, missing IHDR/IDAT, palette index out of range, ...).
"""
import struct
import zlib

import numpy as np

_SIG = b"\x89PNG\r\n\x1a\n"
_ADAM7 = ((0, 0, 8, 8), (4, 0, 8, 8), (0, 4, 4, 8), (2, 0, 4, 4),
          (0, 2, 2, 4), (1, 0, 2, 2), (0, 1, 1, 2))
_CHANNELS = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}
_DEPTHS = {0: (1, 2, 4, 8, 16), 2: (8, 16), 3: (1, 2, 4, 8), 4: (8, 16), 6: (8, 16)}


class PngError(ValueError):
    pass


# Pixel cap for decode_fast's Pillow path. Pillow's own decompression-bomb guard (179 MP)
# rejects real packs: Metroid HD 2.0 ships three 16384x15360 RGBA maps (251.7 MP each), which
# Mesen's spng loads (it keeps the 2^31-1 dimension limit; its 64 MB chunk limits apply only
# to compressed ancillary chunks, not IDAT). This cap is ours: 1 G pixels = 4 GB of RGBA8, far
# above any pack in the corpus and still a bound on an untrusted file.
MAX_PIXELS = 1 << 30


class TooLarge(Exception):
    """Over our cap but NOT invalid: Mesen would still draw it, so callers must refuse the
    pack rather than treat the image as undecodable (which means 'draws nothing')."""


def _unfilter(raw, pos, width, height, channels, depth):
    """Undo the per-row filters of one (sub-)image. Returns (rows uint8 [h, stride], new pos)."""
    bpp = max(1, channels * depth // 8)
    stride = (width * channels * depth + 7) // 8
    out = np.zeros((height, stride), dtype=np.uint8)
    prev = np.zeros(stride, dtype=np.int32)
    for y in range(height):
        if pos + 1 + stride > len(raw):
            raise PngError("image data too short")
        ftype = raw[pos]
        line = np.frombuffer(raw, dtype=np.uint8, count=stride, offset=pos + 1).astype(np.int32)
        pos += 1 + stride
        if ftype == 0:
            cur = line
        elif ftype == 2:
            cur = (line + prev) & 0xFF
        elif ftype in (1, 3, 4):
            cur = line.copy()
            a = 0
            for i in range(stride):
                a = cur[i - bpp] if i >= bpp else 0
                if ftype == 1:
                    cur[i] = (cur[i] + a) & 0xFF
                elif ftype == 3:
                    cur[i] = (cur[i] + ((a + prev[i]) >> 1)) & 0xFF
                else:
                    b = prev[i]
                    c = prev[i - bpp] if i >= bpp else 0
                    p = a + b - c
                    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                    pred = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
                    cur[i] = (cur[i] + pred) & 0xFF
        else:
            raise PngError(f"bad filter type {ftype}")
        out[y] = cur
        prev = cur
    return out, pos


def _samples(rows, width, channels, depth):
    """Packed rows -> int array [h, w, channels] of raw sample values."""
    h = rows.shape[0]
    if depth == 8:
        return rows[:, :width * channels].reshape(h, width, channels).astype(np.int32)
    if depth == 16:
        s = rows[:, :width * channels * 2].reshape(h, width * channels, 2).astype(np.int32)
        return (s[:, :, 0] * 256 + s[:, :, 1]).reshape(h, width, channels)
    bits = np.unpackbits(rows, axis=1)                      # MSB first
    n = width * channels
    bits = bits[:, :n * depth].reshape(h, n, depth)
    weights = 1 << np.arange(depth - 1, -1, -1)
    return (bits * weights).sum(axis=2).reshape(h, width, channels).astype(np.int32)


def _chunks(data):
    if data[:8] != _SIG:
        raise PngError("not a PNG")
    pos, ihdr, plte, trns = 8, None, None, None
    while pos + 8 <= len(data):
        (length,) = struct.unpack(">I", data[pos:pos + 4])
        kind = data[pos + 4:pos + 8]
        body = data[pos + 8:pos + 8 + length]
        pos += 12 + length
        if kind == b"IHDR":
            ihdr = struct.unpack(">IIBBBBB", body[:13])
        elif kind == b"PLTE":
            plte = np.frombuffer(body, dtype=np.uint8)[:len(body) // 3 * 3].reshape(-1, 3)
        elif kind == b"tRNS":
            trns = body
        elif kind == b"IEND":
            break
    return ihdr, plte, trns


def decode_fast(data):
    """Same result as decode(), using Pillow only to inflate and unfilter 8-bit images (the
    pure-Python unfilter is too slow for large backgrounds). The sample-to-RGBA conversion,
    palette and tRNS handling are ours, as in decode(). Other depths go to decode()."""
    ihdr, plte, trns = _chunks(data)
    if ihdr is None or ihdr[2] != 8:
        return decode(data)
    import io
    from PIL import Image
    width, height, depth, ctype = ihdr[:4]
    if width * height > MAX_PIXELS:
        raise TooLarge(f"{width}x{height} exceeds the {MAX_PIXELS:,}-pixel cap")
    saved = Image.MAX_IMAGE_PIXELS
    Image.MAX_IMAGE_PIXELS = MAX_PIXELS
    try:
        im = Image.open(io.BytesIO(data))
        im.load()
    except Exception:
        return decode(data)
    finally:
        Image.MAX_IMAGE_PIXELS = saved
    want = {0: "L", 2: "RGB", 3: "P", 4: "LA", 6: "RGBA"}.get(ctype)
    if im.mode != want:
        return decode(data)
    s = np.asarray(im)                    # uint8: an int32 copy of a 251 MP map is 4 GB
    del im
    if s.ndim == 2:
        s = s[:, :, None]
    out = np.empty((height, width, 4), dtype=np.uint8)
    if ctype == 3:
        if plte is None or s.max() >= len(plte):
            raise PngError("palette index out of range")
        alpha = np.full(len(plte), 255, dtype=np.uint8)
        if trns is not None:
            t = np.frombuffer(trns[:len(plte)], dtype=np.uint8)
            alpha[:len(t)] = t
        out[:, :, :3] = plte[s[:, :, 0]]
        out[:, :, 3] = alpha[s[:, :, 0]]
    elif ctype in (0, 4):
        out[:, :, 0] = out[:, :, 1] = out[:, :, 2] = s[:, :, 0]
        out[:, :, 3] = s[:, :, 1] if ctype == 4 else 255
        if ctype == 0 and trns is not None and len(trns) >= 2:
            out[:, :, 3][s[:, :, 0] == struct.unpack(">H", trns[:2])[0]] = 0
    else:
        out[:, :, :3] = s[:, :, :3]
        out[:, :, 3] = s[:, :, 3] if ctype == 6 else 255
        if ctype == 2 and trns is not None and len(trns) >= 6:
            tv = struct.unpack(">HHH", trns[:6])
            m = (s[:, :, 0] == tv[0]) & (s[:, :, 1] == tv[1]) & (s[:, :, 2] == tv[2])
            out[:, :, 3][m] = 0
    return width, height, out


def decode(data):
    if data[:8] != _SIG:
        raise PngError("not a PNG")
    pos = 8
    ihdr = None
    plte = None
    trns = None
    idat = []
    while pos + 8 <= len(data):
        (length,) = struct.unpack(">I", data[pos:pos + 4])
        kind = data[pos + 4:pos + 8]
        body = data[pos + 8:pos + 8 + length]
        if len(body) < length:
            raise PngError("truncated chunk")
        pos += 12 + length
        if kind == b"IHDR":
            ihdr = struct.unpack(">IIBBBBB", body[:13])
        elif kind == b"PLTE":
            plte = np.frombuffer(body, dtype=np.uint8).reshape(-1, 3)
        elif kind == b"tRNS":
            trns = body
        elif kind == b"IDAT":
            idat.append(body)
        elif kind == b"IEND":
            break
    if ihdr is None or not idat:
        raise PngError("missing IHDR or IDAT")
    width, height, depth, ctype, comp, filt, interlace = ihdr
    if ctype not in _CHANNELS or depth not in _DEPTHS[ctype]:
        raise PngError(f"colour type {ctype} depth {depth} invalid")
    if comp != 0 or filt != 0 or interlace > 1 or width == 0 or height == 0:
        raise PngError("invalid IHDR")
    if ctype == 3 and plte is None:
        raise PngError("palette image without PLTE")
    channels = _CHANNELS[ctype]
    try:
        raw = zlib.decompress(b"".join(idat))
    except zlib.error as e:
        raise PngError(f"zlib: {e}")

    s = np.zeros((height, width, channels), dtype=np.int32)
    if interlace == 0:
        rows, _ = _unfilter(raw, 0, width, height, channels, depth)
        s[:] = _samples(rows, width, channels, depth)
    else:
        p = 0
        for x0, y0, dx, dy in _ADAM7:
            pw = (width - x0 + dx - 1) // dx if width > x0 else 0
            ph = (height - y0 + dy - 1) // dy if height > y0 else 0
            if pw == 0 or ph == 0:
                continue
            rows, p = _unfilter(raw, p, pw, ph, channels, depth)
            s[y0::dy, x0::dx] = _samples(rows, pw, channels, depth)

    out = np.empty((height, width, 4), dtype=np.uint8)
    if ctype == 3:
        if s.max() >= len(plte):
            raise PngError("palette index out of range")
        alpha = np.full(len(plte), 255, dtype=np.uint8)
        if trns is not None:
            t = np.frombuffer(trns[:len(plte)], dtype=np.uint8)
            alpha[:len(t)] = t
        idx = s[:, :, 0]
        out[:, :, :3] = plte[idx]
        out[:, :, 3] = alpha[idx]
        return width, height, out

    def to8(v):
        if depth == 16:
            return (v >> 8).astype(np.uint8)
        if depth < 8:
            return (v * (255 // ((1 << depth) - 1))).astype(np.uint8)
        return v.astype(np.uint8)

    if ctype in (0, 4):
        g = to8(s[:, :, 0])
        out[:, :, 0] = out[:, :, 1] = out[:, :, 2] = g
        if ctype == 4:
            out[:, :, 3] = to8(s[:, :, 1])
        else:
            out[:, :, 3] = 255
            if trns is not None and len(trns) >= 2:
                tv = struct.unpack(">H", trns[:2])[0]
                out[:, :, 3][s[:, :, 0] == tv] = 0
    else:
        for c in range(3):
            out[:, :, c] = to8(s[:, :, c])
        if ctype == 6:
            out[:, :, 3] = to8(s[:, :, 3])
        else:
            out[:, :, 3] = 255
            if trns is not None and len(trns) >= 6:
                tv = struct.unpack(">HHH", trns[:6])
                m = (s[:, :, 0] == tv[0]) & (s[:, :, 1] == tv[1]) & (s[:, :, 2] == tv[2])
                out[:, :, 3][m] = 0
    return width, height, out


def premultiply(img):
    """HdPackBitmapInfo init (spec 4.5): c = ((a+1)*c) >> 8 for pixels with alpha < 255."""
    out = img.copy()
    for y0 in range(0, img.shape[0], 512):     # bands: whole-image int32 temporaries of a
        band = img[y0:y0 + 512]                # 251 MP map cost ~6 GB
        a = band[:, :, 3].astype(np.int32)
        m = a < 255
        for c in range(3):
            ch = band[:, :, c].astype(np.int32)
            out[y0:y0 + 512, :, c] = np.where(m, ((a + 1) * ch) >> 8, ch).astype(np.uint8)
    return out
