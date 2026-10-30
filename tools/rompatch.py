"""Apply an IPS, BPS or UPS patch to a ROM file, as Mesen's <patch> does (format detected from
the patch header; the whole file, iNES header included, is patched).

    apply(rom_bytes, patch_bytes) -> patched bytes      raises PatchError

BPS and UPS check the source, target and patch CRC-32s and refuse on any mismatch, so a patch
meant for a different ROM never yields a plausible-looking result.
"""
import zlib


class PatchError(Exception):
    pass


def _vlq(p, i):
    """BPS/UPS variable-length number at p[i]; returns (value, next index)."""
    value, shift = 0, 1
    while True:
        if i >= len(p):
            raise PatchError("truncated number")
        b = p[i]
        i += 1
        value += (b & 0x7F) * shift
        if b & 0x80:
            return value, i
        shift <<= 7
        value += shift


def apply_ips(rom, p):
    out = bytearray(rom)
    i = 5
    while True:
        if i + 3 > len(p):
            raise PatchError("IPS: no EOF marker")
        off = int.from_bytes(p[i:i + 3], "big")
        i += 3
        if off == 0x454F46:                       # "EOF"
            if i + 3 == len(p):                   # optional truncation extension
                out = out[:int.from_bytes(p[i:i + 3], "big")]
            elif i != len(p):
                raise PatchError("IPS: data after EOF")
            return bytes(out)
        if i + 2 > len(p):
            raise PatchError("IPS: truncated record")
        size = int.from_bytes(p[i:i + 2], "big")
        i += 2
        if size:
            data = p[i:i + size]
            if len(data) != size:
                raise PatchError("IPS: truncated data")
            i += size
        else:                                     # RLE record
            if i + 3 > len(p):
                raise PatchError("IPS: truncated RLE")
            size = int.from_bytes(p[i:i + 2], "big")
            data = p[i + 2:i + 3] * size
            i += 3
        if off + size > len(out):
            out.extend(bytes(off + size - len(out)))
        out[off:off + size] = data


def _crcs(p, rom, out, kind):
    n = len(p)       # absolute offsets: p[-4:-4 + 4] would be p[-4:0], an empty slice
    src, dst, pat = (int.from_bytes(p[n + j:n + j + 4], "little") for j in (-12, -8, -4))
    if zlib.crc32(p[:-4]) != pat:
        raise PatchError(f"{kind}: patch CRC mismatch")
    if zlib.crc32(rom) != src:
        raise PatchError(f"{kind}: source ROM CRC mismatch")
    if out is not None and zlib.crc32(out) != dst:
        raise PatchError(f"{kind}: result CRC mismatch")


def apply_bps(rom, p):
    if len(p) < 4 + 12:
        raise PatchError("BPS: too short")
    _crcs(p, rom, None, "BPS")
    i = 4
    src_size, i = _vlq(p, i)
    dst_size, i = _vlq(p, i)
    meta, i = _vlq(p, i)
    i += meta
    if src_size != len(rom):
        raise PatchError("BPS: source size mismatch")
    out = bytearray(dst_size)
    o = src_rel = dst_rel = 0
    end = len(p) - 12
    while i < end:
        cmd, i = _vlq(p, i)
        length, action = (cmd >> 2) + 1, cmd & 3
        if o + length > dst_size:
            raise PatchError("BPS: write past target")
        if action == 0:                           # SourceRead
            out[o:o + length] = rom[o:o + length]
        elif action == 1:                         # TargetRead
            out[o:o + length] = p[i:i + length]
            i += length
        else:
            d, i = _vlq(p, i)
            d = -(d >> 1) if d & 1 else d >> 1
            if action == 2:                       # SourceCopy
                src_rel += d
                out[o:o + length] = rom[src_rel:src_rel + length]
                src_rel += length
            else:                                 # TargetCopy (may overlap: byte by byte)
                dst_rel += d
                for k in range(length):
                    out[o + k] = out[dst_rel + k]
                dst_rel += length
        o += length
    if o != dst_size:
        raise PatchError("BPS: target not fully written")
    out = bytes(out)
    _crcs(p, rom, out, "BPS")
    return out


def apply_ups(rom, p):
    if len(p) < 4 + 12:
        raise PatchError("UPS: too short")
    _crcs(p, rom, None, "UPS")
    i = 4
    src_size, i = _vlq(p, i)
    dst_size, i = _vlq(p, i)
    if src_size != len(rom):
        raise PatchError("UPS: source size mismatch")
    out = bytearray(rom[:dst_size]) + bytearray(max(0, dst_size - len(rom)))
    o = 0
    end = len(p) - 12
    while i < end:
        skip, i = _vlq(p, i)
        o += skip
        while i < end and p[i]:
            if o < dst_size:
                out[o] ^= p[i]
            o += 1
            i += 1
        i += 1
        o += 1
    out = bytes(out)
    _crcs(p, rom, out, "UPS")
    return out


def apply(rom, patch):
    rom, patch = bytes(rom), bytes(patch)
    if patch[:5] == b"PATCH":
        return apply_ips(rom, patch)
    if patch[:4] == b"BPS1":
        return apply_bps(rom, patch)
    if patch[:4] == b"UPS1":
        return apply_ups(rom, patch)
    raise PatchError("unknown patch format")
