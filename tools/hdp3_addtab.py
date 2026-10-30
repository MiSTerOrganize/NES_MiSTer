"""Format-3 addition tables (HDPACK_DESIGN.md "8a"): sections 16 and 17, built from the additions
(section 11) and the fallback map (section 10). hdp3_convert.py writes them; this module is also
usable on its own to add them to an existing .hdp.

  section 16  addition-key table, the key table's exact format: 64-byte buckets of four 16-byte
              entries {q0 = palette | body_lo << 32, q1 = body_hi | list_off/8 << 32 | kind << 62
              | valid << 63}, bucket = CRC-32({palette, body lo, body hi}) & (buckets - 1) with
              the format's CRC-32 (hdp3_convert.crc32 - NOT zlib's), linear probing, an empty
              entry ends the probe. Palette ANY (0xFFFFFFFF) = the additions that ignore the
              palette; no real palette can be ANY (its low three bytes are colour indices <= 0x3F).
  section 17  lists: at list_off {u32 count, u32 0}, then per addition 3 qwords
              {index 16 | ox 16 << 16 | oy 16 << 32 | added kind << 48}, added palette, added body.
  header      0x4C = section 16's bucket count, 0x50 = its maximum probe (0 / 0 = no table).

A record (palette P, body B) matches, in file order, the union of the lists of (P, B) and
(ANY, B) and - in a CHR-ROM game with a fallback map - (P, F) and (ANY, F) for F = fallback(B)
when F >= 0 and F != B; exactly hdref.additions_for.

A pack where an added key would itself match an addition is refused (a chain: Mesen would grow
the list it is walking; the hardware does not model that, and the corpus has none).

Library: build(add_bytes, nadd) -> (table, nbuckets, max_probe, lists); chain(add_bytes, nadd,
fb) -> message or None; lookup / matched: the hardware's view of the tables.
CLI: python hdp3_addtab.py IN.hdp OUT.hdp   (adds sections 16/17 to a pack without them)
"""
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hdp3_convert import crc32, Refused  # noqa: E402

ANY = 0xFFFFFFFF
REC = "<IIQIIQiiI"
HEADER_SIZE = 512
ALIGN = 64
S_FALLBACK, S_ADDITIONS, S_ADDKEYS, S_ADDLISTS = 10, 11, 16, 17
N_SECTIONS = 18


def records(add_bytes, nadd):
    return [struct.unpack_from(REC, add_bytes, 64 * i) for i in range(nadd)]


def table_key(r):
    opal, okind, obody, _, _, _, _, _, flags = r
    return (ANY if flags & 1 else opal, okind, obody & 0xFFFFFFFF, obody >> 32)


def build(add_bytes, nadd):
    if nadd == 0:
        return b"", 0, 0, b""
    if nadd > 0xFFFF:
        raise Refused(f"{nadd} additions; the list entries index at most 65,535")
    groups = {}
    for i, r in enumerate(records(add_bytes, nadd)):
        _, _, _, apal, akind, abody, ox, oy, _ = r
        if not (-32768 <= ox <= 32767 and -32768 <= oy <= 32767):
            raise Refused(f"addition {i}: offset ({ox}, {oy}) does not fit 16 bits")
        groups.setdefault(table_key(r), []).append((i, ox, oy, akind, apal, abody))
    lists = bytearray()
    entries = []
    for key, items in groups.items():
        off = len(lists)
        lists += struct.pack("<II", len(items), 0)
        for i, ox, oy, akind, apal, abody in items:
            lists += struct.pack("<QQQ", i | ((ox & 0xFFFF) << 16) | ((oy & 0xFFFF) << 32)
                                 | (akind << 48), apal, abody)
        entries.append(key + (off,))
    n = len(entries)
    nb = 1
    while nb * 4 < 2 * n:
        nb *= 2
    buckets = [[] for _ in range(nb)]
    max_probe = 0
    for pal, kind, lo, hi, off in entries:
        h = crc32(struct.pack("<III", pal, lo, hi)) & (nb - 1)
        for probe in range(nb):
            b = (h + probe) & (nb - 1)
            if len(buckets[b]) < 4:
                buckets[b].append((pal, kind, lo, hi, off))
                max_probe = max(max_probe, probe)
                break
    table = bytearray(nb * 64)
    for b, ents in enumerate(buckets):
        for j, (pal, kind, lo, hi, off) in enumerate(ents):
            struct.pack_into("<QQ", table, b * 64 + j * 16, pal | (lo << 32),
                             hi | ((off // 8) << 32) | (kind << 62) | (1 << 63))
    return bytes(table), nb, max_probe, bytes(lists)


def lookup(table, nb, max_probe, lists, pal, kind, body):
    """The hardware's lookup: probe the table, return the list's entries (or [])."""
    if nb == 0:
        return []
    lo, hi = body & 0xFFFFFFFF, body >> 32
    h = crc32(struct.pack("<III", pal, lo, hi)) & (nb - 1)
    for probe in range(max_probe + 1):
        b = ((h + probe) & (nb - 1)) * 64
        for j in range(4):
            q0, q1 = struct.unpack_from("<QQ", table, b + j * 16)
            if not q1 >> 63:
                return []
            if ((q0 & 0xFFFFFFFF) == pal and q0 >> 32 == lo and (q1 & 0xFFFFFFFF) == hi
                    and (q1 >> 62) & 1 == kind):
                off = ((q1 >> 32) & 0x3FFFFFFF) * 8
                cnt = struct.unpack_from("<I", lists, off)[0]
                out = []
                for e in range(cnt):
                    e0, apal, abody = struct.unpack_from("<QQQ", lists, off + 8 + 24 * e)
                    ox = (e0 >> 16) & 0xFFFF
                    oy = (e0 >> 32) & 0xFFFF
                    out.append((e0 & 0xFFFF, ox - 0x10000 if ox & 0x8000 else ox,
                                oy - 0x10000 if oy & 0x8000 else oy, (e0 >> 48) & 1, apal, abody))
                return out
    return []


def matched(table, nb, max_probe, lists, pal, kind, body, fb):
    """Every addition a record (pal, kind, body) matches, in file order (the union of the
    up-to-four lookups; fb = fallback map list, empty for a CHR-RAM game)."""
    bodies = [body]
    if kind == 0 and 0 <= body < len(fb) and fb[body] >= 0 and fb[body] != body:
        bodies.append(fb[body])
    hit = {}
    for b in bodies:
        for p in (pal, ANY):
            for e in lookup(table, nb, max_probe, lists, p, kind, b):
                hit[e[0]] = e
    return [hit[i] for i in sorted(hit)]


def chain(add_bytes, nadd, fb):
    """An added key that would itself match an addition: a message, or None."""
    t, nb, mp, li = build(add_bytes, nadd)
    for i, r in enumerate(records(add_bytes, nadd)):
        _, _, _, apal, akind, abody, _, _, _ = r
        m = matched(t, nb, mp, li, apal, akind, abody, fb)
        if m:
            return (f"addition {i}'s added key matches addition {m[0][0]} (a chain); "
                    "the hardware does not support chained additions")
    return None


def patch(src, out):
    d = bytearray(open(src, "rb").read())
    if d[:4] != b"NHDP":
        raise Refused("not an .hdp file")
    nadd, nfb = struct.unpack_from("<II", d, 0x40)
    sec = [struct.unpack_from("<II", d, 0x80 + 8 * i) for i in range(N_SECTIONS)]
    if sec[S_ADDKEYS] != (0, 0) or sec[S_ADDLISTS] != (0, 0):
        raise Refused("the pack already has sections 16/17")
    add_bytes = bytes(d[sec[S_ADDITIONS][0]:sec[S_ADDITIONS][0] + 64 * nadd])
    fb = list(struct.unpack_from(f"<{nfb}i", d, sec[S_FALLBACK][0])) if nfb else []
    msg = chain(add_bytes, nadd, fb)
    if msg:
        raise Refused(msg)
    table, nb, mp, lists = build(add_bytes, nadd)
    for i, s in ((S_ADDKEYS, table), (S_ADDLISTS, lists)):
        if not s:
            continue
        if len(d) % ALIGN:
            d += bytes(ALIGN - len(d) % ALIGN)
        struct.pack_into("<II", d, 0x80 + 8 * i, len(d), len(s))
        d += s
    struct.pack_into("<II", d, 0x4C, nb, mp)
    struct.pack_into("<I", d, 0x20, len(d))          # the header's total-size field
    open(out, "wb").write(bytes(d))
    return nadd, nb, mp, len(lists)


if __name__ == "__main__":
    try:
        n, nb, mp, nl = patch(sys.argv[1], sys.argv[2])
    except Refused as e:
        sys.exit(f"refused: {e}")
    print(f"{sys.argv[2]}: {n} additions, table {nb} buckets (max probe {mp}), lists {nl} bytes")
