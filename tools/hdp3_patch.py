"""Format-3 section 14: ROM patches as the hardware applies them (stage 5, HDPACK_DESIGN.md
"Stage 5 - ROM patches").

The core cannot run an IPS/BPS/UPS decoder byte-stream style while the ROM streams in: a patch
can change the cart's size and type (Metroid's mmm.ips turns 128 KB PRG + CHR-RAM into 256 KB
PRG + 128 KB CHR-ROM), BPS copies from anywhere in the source and the target, and BPS/UPS are
only valid once their target CRC-32 checks. So the converter compiles each <patch> into one
uniform list of RUNS, and the hardware builds the target in DDR3 from the staged source ROM:

    target byte t comes from the run covering t, else (base = source) source[t], 0 past its end

    run mode   byte at target offset tgt_off + i
    LIT  0     data[arg + i]
    SRC  1     source[arg + i]                         (0 past the source's end)
    TGT  2     target[arg + i], already built          (byte by byte: may overlap, BPS)
    XOR  3     source[tgt_off + i] ^ data[arg + i]     (UPS)
    FILL 4     arg & 0xFF                               (IPS RLE)

Runs are sorted by tgt_off and do not overlap. The target size is
    exact:      tgt_size                                   (BPS, UPS)
    from src:   max(len(source), tgt_size)                 (IPS: tgt_size = the last record's end)
    truncated:  min(that, trunc)                           (IPS truncation extension)
and the target CRC-32 is checked when the patch carries one (BPS, UPS), else the result is used
as is (IPS has none). A failed check leaves the ROM unpatched, as Mesen does when its patcher
refuses.

Section 14 layout (little-endian; every part 64-byte aligned, offsets inside the section):
    0x00 u32 entry count, 0x04 u32 0, then 56 bytes 0
    entries, 64 bytes each:
        0x00 sha1[20]   of the whole source file (iNES header included), as Mesen matches
        0x14 u8   base: 1 = gaps are the source, 0 = runs cover the whole target
        0x15 u8   flags: bit 0 target CRC valid, bit 1 size from the source, bit 2 truncated,
                         bit 3 source size checked
        0x16 u16  0
        0x18 u32  source size (bit 3)
        0x1C u32  target size
        0x20 u32  truncation length (bit 2)
        0x24 u32  target CRC-32 (bit 0)
        0x28 u32  runs offset   0x2C u32 run count
        0x30 u32  data offset   0x34 u32 data length
        0x38 8 bytes 0
    runs, 16 bytes each: u32 tgt_off, u32 len, u32 arg, u8 mode, 3 bytes 0
    data bytes

    compile_patch(patch_bytes) -> dict     raises rompatch.PatchError
    section(entries)  -> bytes             entries = [(sha1_hex, compiled), ...]
    parse(section_bytes) -> [entries]      (the hardware's view, for tests and hdp3_read)
    build(entry, source) -> bytes or None  the hardware model: the target, or None when a check
                                           fails (the ROM is then used unpatched)
"""
import struct
import zlib

import rompatch
from rompatch import PatchError, _vlq

LIT, SRC, TGT, XOR, FILL = 0, 1, 2, 3, 4
F_CRC, F_FROMSRC, F_TRUNC, F_SRCSIZE = 1, 2, 4, 8


def _merge(runs):
    """Sort, check no overlap, merge adjacent runs that continue each other."""
    runs.sort(key=lambda r: r[0])
    out = []
    for r in runs:
        if out:
            p = out[-1]
            if r[0] < p[0] + p[1]:
                raise PatchError("internal: overlapping runs")
            if (r[0] == p[0] + p[1] and r[3] == p[3] and r[3] in (LIT, SRC, XOR)
                    and r[2] == p[2] + p[1]):
                out[-1] = (p[0], p[1] + r[1], p[2], p[3])
                continue
            if r[0] == p[0] + p[1] and r[3] == FILL == p[3] and r[2] == p[2]:
                out[-1] = (p[0], p[1] + r[1], p[2], p[3])
                continue
        out.append(r)
    return out


def _compile_ips(p):
    """IPS: later records overwrite earlier ones. Resolved here into non-overlapping LIT/FILL
    runs (each byte remembers the record that wrote it last), gaps = the source."""
    owner = {}                    # target offset -> ("lit", data index) or ("fill", byte)
    data = bytearray()
    i, end_max, trunc = 5, 0, None
    while True:
        if i + 3 > len(p):
            raise PatchError("IPS: no EOF marker")
        off = int.from_bytes(p[i:i + 3], "big")
        i += 3
        if off == 0x454F46:
            if i + 3 == len(p):
                trunc = int.from_bytes(p[i:i + 3], "big")
            elif i != len(p):
                raise PatchError("IPS: data after EOF")
            break
        if i + 2 > len(p):
            raise PatchError("IPS: truncated record")
        size = int.from_bytes(p[i:i + 2], "big")
        i += 2
        if size:
            if i + size > len(p):
                raise PatchError("IPS: truncated data")
            base = len(data)
            data += p[i:i + size]
            for k in range(size):
                owner[off + k] = ("lit", base + k)
            i += size
        else:
            if i + 3 > len(p):
                raise PatchError("IPS: truncated RLE")
            size = int.from_bytes(p[i:i + 2], "big")
            b = p[i + 2]
            for k in range(size):
                owner[off + k] = ("fill", b)
            i += 3
        end_max = max(end_max, off + size)
    runs = []
    for t in sorted(owner):
        kind, v = owner[t]
        if kind == "lit":
            runs.append((t, 1, v, LIT))
        else:
            runs.append((t, 1, v, FILL))
    runs = _merge(runs)
    # repack the literal data so the runs' data is contiguous in target order (smaller, and
    # the hardware reads it sequentially)
    packed, out = bytearray(), []
    for t, n, a, m in runs:
        if m == LIT:
            out.append((t, n, len(packed), m))
            packed += data[a:a + n]
        else:
            out.append((t, n, a, m))
    flags = F_FROMSRC | (F_TRUNC if trunc is not None else 0)
    return dict(base=1, flags=flags, src_size=0, tgt_size=end_max, trunc=trunc or 0, crc=0,
                runs=out, data=bytes(packed))


def _crc_tail(p, kind):
    n = len(p)
    src, dst, pat = (int.from_bytes(p[n + j:n + j + 4], "little") for j in (-12, -8, -4))
    if zlib.crc32(p[:-4]) != pat:
        raise PatchError(f"{kind}: patch CRC mismatch")
    return src, dst


def _compile_bps(p):
    if len(p) < 16:
        raise PatchError("BPS: too short")
    _, dst_crc = _crc_tail(p, "BPS")
    i = 4
    src_size, i = _vlq(p, i)
    dst_size, i = _vlq(p, i)
    meta, i = _vlq(p, i)
    i += meta
    runs, data = [], bytearray()
    o = src_rel = dst_rel = 0
    end = len(p) - 12
    while i < end:
        cmd, i = _vlq(p, i)
        length, action = (cmd >> 2) + 1, cmd & 3
        if o + length > dst_size:
            raise PatchError("BPS: write past target")
        if action == 0:
            runs.append((o, length, o, SRC))
        elif action == 1:
            runs.append((o, length, len(data), LIT))
            data += p[i:i + length]
            i += length
        else:
            d, i = _vlq(p, i)
            d = -(d >> 1) if d & 1 else d >> 1
            if action == 2:
                src_rel += d
                if src_rel < 0:
                    raise PatchError("BPS: source copy before the start")
                runs.append((o, length, src_rel, SRC))
                src_rel += length
            else:
                dst_rel += d
                if dst_rel < 0 or dst_rel >= o:
                    raise PatchError("BPS: target copy from an unwritten byte")
                runs.append((o, length, dst_rel, TGT))
                dst_rel += length
        o += length
    if o != dst_size:
        raise PatchError("BPS: target not fully written")
    return dict(base=0, flags=F_CRC | F_SRCSIZE, src_size=src_size, tgt_size=dst_size, trunc=0,
                crc=dst_crc, runs=_merge(runs), data=bytes(data))


def _compile_ups(p):
    if len(p) < 16:
        raise PatchError("UPS: too short")
    _, dst_crc = _crc_tail(p, "UPS")
    i = 4
    src_size, i = _vlq(p, i)
    dst_size, i = _vlq(p, i)
    runs, data = [], bytearray()
    o = 0
    end = len(p) - 12
    while i < end:
        skip, i = _vlq(p, i)
        o += skip
        st = i
        while i < end and p[i]:
            i += 1
        n = min(i - st, max(0, dst_size - o))
        if n:
            runs.append((o, n, len(data), XOR))
            data += p[st:st + n]
        o += i - st
        i += 1
        o += 1
    return dict(base=1, flags=F_CRC | F_SRCSIZE, src_size=src_size, tgt_size=dst_size, trunc=0,
                crc=dst_crc, runs=_merge(runs), data=bytes(data))


def compile_patch(patch):
    patch = bytes(patch)
    if patch[:5] == b"PATCH":
        return _compile_ips(patch)
    if patch[:4] == b"BPS1":
        return _compile_bps(patch)
    if patch[:4] == b"UPS1":
        return _compile_ups(patch)
    raise PatchError("unknown patch format")


def _pad64(b):
    return bytes(b) + bytes(-len(b) % 64)


def section(entries):
    """entries: [(sha1 hex, compiled dict)] -> section 14 bytes."""
    head = struct.pack("<II", len(entries), 0) + bytes(56)
    table_len = 64 * len(entries)
    body = bytearray()
    recs = []
    pos = 64 + table_len
    for sha, c in entries:
        runs = b"".join(struct.pack("<IIIB3x", t, n, a, m) for t, n, a, m in c["runs"])
        roff = pos + len(body)
        body += _pad64(runs)
        doff = pos + len(body)
        body += _pad64(c["data"])
        recs.append(bytes.fromhex(sha) + struct.pack(
            "<BBHIIIIIIII8x", c["base"], c["flags"], 0, c["src_size"], c["tgt_size"], c["trunc"],
            c["crc"], roff, len(c["runs"]), doff, len(c["data"])))
    return head + b"".join(recs) + bytes(body)


def parse(sec):
    """section 14 bytes -> [dict] with sha1 (hex upper) and the compiled fields."""
    if not sec:
        return []
    n = struct.unpack_from("<I", sec, 0)[0]
    out = []
    for k in range(n):
        e = sec[64 + 64 * k: 128 + 64 * k]
        (base, flags, _, src_size, tgt_size, trunc, crc, roff, rn, doff, dlen) = struct.unpack_from(
            "<BBHIIIIIIII", e, 20)
        runs = [struct.unpack_from("<IIIB", sec, roff + 16 * j) for j in range(rn)]
        out.append(dict(sha1=e[:20].hex().upper(), base=base, flags=flags, src_size=src_size,
                        tgt_size=tgt_size, trunc=trunc, crc=crc, runs=runs,
                        data=bytes(sec[doff:doff + dlen])))
    return out


def build(e, src):
    """The hardware model: target bytes, or None when a check fails (ROM used unpatched)."""
    src = bytes(src)
    if e["flags"] & F_SRCSIZE and len(src) != e["src_size"]:
        return None
    size = e["tgt_size"]
    if e["flags"] & F_FROMSRC:
        size = max(len(src), size)
    if e["flags"] & F_TRUNC:
        size = min(size, e["trunc"])
    s = lambda i: src[i] if i < len(src) else 0
    out = bytearray(size)
    if e["base"]:
        for i in range(size):
            out[i] = s(i)
    data = e["data"]
    for t, n, a, m in e["runs"]:
        for k in range(n):
            if t + k >= size:
                break
            if m == LIT:
                b = data[a + k]
            elif m == SRC:
                b = s(a + k)
            elif m == TGT:
                b = out[a + k]
            elif m == XOR:
                b = s(t + k) ^ data[a + k]
            elif m == FILL:
                b = a & 0xFF
            else:
                return None
            out[t + k] = b
    out = bytes(out)
    if e["flags"] & F_CRC and zlib.crc32(out) != e["crc"]:
        return None
    return out
