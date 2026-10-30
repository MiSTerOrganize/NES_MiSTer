"""Read a format-3 .hdp file the way the hardware will, and check it against the pack.

Library: Hdp3(data) parses the header and gives the lookups the engine performs (key table,
rule lists, condition decode, background index). The reference renderer and the sims use it.

Check mode compares every structure with the pack as hdpack_model parsed it, using code paths
written separately from the converter:
  - every key is found through the hash table and its rule list matches the file-order list;
  - each rule's conditions decode back to the pack's conditions (as a set per class), flags
    and prefix_fdc are right;
  - each rule's block equals the tile re-derived per pixel from the PNG (naive code);
  - the game's CHR kind, every kept addition record, and the whole fallback map (re-derived
    from the ROM when automaticFallbackTiles is set);
  - background rules, records and every map cell; for indexed levels, the index selects the
    same first background as a plain linear scan for random and boundary memory states;
  - section 14: every <patch> present as an independent compile of its file (decoded here,
    not by hdp3_patch.parse); with --rom, the patched ROM built as the hardware builds it
    equals rompatch.apply.

usage: python hdp3_read.py <file.hdp> <pack_dir> [--samples N] [--rom game.nes] [--base dir ...]
"""
import argparse
import os
import random
import struct
import sys
from collections import OrderedDict

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hdp3_addtab  # noqa: E402
import hdpack_model as M  # noqa: E402
import pngdec  # noqa: E402
from hdp3_convert import crc32, crc64  # noqa: E402  (hash definitions are the format)
from hdp3_convert import load_pack, patch_rom, read_rom_chr  # noqa: E402  (file access, not under test)
from hdp3_convert import DEFAULT_ROMINDEX  # noqa: E402


def rom_chr_ram(path, pack):
    """Mesen's BaseMapper::HasChrRam() for the patched ROM, written separately from the
    converter: NES 2.0 -> byte 11 low nibble non-zero; iNES 1.0 -> only when there is no
    CHR-ROM (a database/mapper CHR-RAM on an iNES 1.0 CHR-ROM cart is not modelled)."""
    with open(path, "rb") as f:
        d = patch_rom(f.read(), pack)
    if (d[7] >> 2) & 3 == 2:
        return (d[11] & 15) > 0
    return d[5] == 0

TYPES = ["memoryCheck", "memoryCheckConstant", "frameRange", "tileAtPosition",
         "spriteAtPosition", "tileNearby", "spriteNearby", "positionCheckX", "positionCheckY",
         "originPositionCheckX", "originPositionCheckY", "hmirror", "vmirror", "bgpriority"]
OPS = ["eq", "ne", "gt", "lt", "le", "ge"]


class Hdp3:
    def __init__(self, data):
        self.d = data
        if data[:4] != b"NHDP":
            raise ValueError("not an .hdp file")
        (self.ver, self.scale, self.flags, self.pack_version, self.options) = \
            struct.unpack_from("<HBBII", data, 4)
        if self.ver != 3:
            raise ValueError(f"format {self.ver}, expected 3")
        self.overscan = struct.unpack_from("<4i", data, 0x10)
        (self.size, self.nbuckets, self.max_probe, self.nconds, self.nframe, self.nwatched,
         self.longest, self.nblocks, self.nadd, self.nfallback, self.nratios) = \
            struct.unpack_from("<11I", data, 0x20)
        self.sec = [struct.unpack_from("<II", data, 0x80 + 8 * i) for i in range(18)]
        self.add_nb, self.add_mp = struct.unpack_from("<II", data, 0x4C)    # section 16
        self.watched = list(struct.unpack_from(f"<{self.nwatched}I", data, self.sec[9][0]))

    def s(self, i):
        return self.sec[i][0]

    # key table -----------------------------------------------------------------------------
    def find_key(self, pal, kind, lo, hi):
        """Returns the rule-list offset (in the rules section) or None."""
        h = crc32(struct.pack("<III", pal, lo, hi)) & (self.nbuckets - 1)
        for probe in range(self.max_probe + 1):
            b = self.s(0) + ((h + probe) & (self.nbuckets - 1)) * 64
            for j in range(4):
                q0, q1 = struct.unpack_from("<QQ", self.d, b + j * 16)
                if not q1 >> 63:
                    return None          # empty slot ends the probe sequence
                if ((q0 & 0xFFFFFFFF) == pal and q0 >> 32 == lo and (q1 & 0xFFFFFFFF) == hi
                        and (q1 >> 62) & 1 == kind):
                    return ((q1 >> 32) & 0x3FFFFFFF) * 8
        return None

    def rules(self, off):
        n, _ = struct.unpack_from("<II", self.d, self.s(1) + off)
        out = []
        for i in range(n):
            w = struct.unpack_from("<IIII", self.d, self.s(1) + off + 8 + 16 * i)
            out.append({"payload": w[0], "frame": w[1], "walk": w[2],
                        "src": w[3] & 0xFFFFFF, "fdc": (w[3] >> 24) & 1,
                        "prefix": (w[3] >> 25) & 1})
        return out

    def clist(self, off):
        out = []
        if off == 0:
            return out
        p = self.s(3) + off
        while True:
            (w,) = struct.unpack_from("<I", self.d, p)
            out.append(w & 0xFFFFFF)
            p += 4
            if w >> 31:
                return out

    def cond(self, i):
        q0, q1, q2, q3 = struct.unpack_from("<QQQQ", self.d, self.s(2) + 32 * i)
        t = TYPES[q0 & 0xFF]
        return {"type": t, "negate": bool(q0 >> 8 & 1), "ignore": bool(q0 >> 9 & 1),
                "data": bool(q0 >> 10 & 1), "frame": bool(q0 >> 11 & 1),
                "sens": bool(q0 >> 24 & 1),
                "op": OPS[(q0 >> 12) & 0xF] if ((q0 >> 12) & 0xF) < 6 else None,
                "mask": (q0 >> 16) & 0xFF, "memo": q0 >> 32, "q1": q1, "q2": q2, "q3": q3}

    def block(self, i):
        o = self.s(4) + i * 1024
        return np.frombuffer(self.d, dtype=np.uint8, count=1024, offset=o).reshape(16, 16, 4)

    def bg_level(self, lvl):
        return struct.unpack_from("<II", self.d, self.s(5) + lvl * 8)

    def bg_rec(self, i):
        v = struct.unpack_from("<IIIIIHHB", self.d, self.s(6) + 32 * i)
        return dict(zip(("map", "w", "h", "left", "top", "hr", "vr", "blend"), v))

    def ratio(self, i, s):
        return struct.unpack_from("<i", self.d, self.s(8) + i * 2048 + s * 4)[0]

    def bg_index_candidates(self, ioff, snap):
        """All entry indices any signature yields for the memory snapshot (slot -> value)."""
        base = self.s(13) + ioff
        (nsig,) = struct.unpack_from("<I", self.d, base)
        cands = []
        for k in range(nsig):
            p = base + 4 + 48 * k
            nterms = self.d[p]
            pairs = [(self.d[p + 2 + 2 * j], self.d[p + 3 + 2 * j]) for j in range(nterms)]
            toff, nb, maxp = struct.unpack_from("<III", self.d, p + 34)
            vals = bytes((snap[sl] & mk) for sl, mk in pairs) + bytes(16 - nterms)
            h = crc32(vals) & (nb - 1)
            for probe in range(maxp + 1):
                e = self.s(13) + toff + 32 * ((h + probe) & (nb - 1))
                lst, cnt, valid = struct.unpack_from("<III", self.d, e + 16)
                if not valid:
                    break
                if self.d[e:e + 16] == vals:
                    cands += list(struct.unpack_from(f"<{cnt}I", self.d, self.s(13) + lst))
                    break
        return sorted(set(cands))


# ------------------------------------------------------------------------------------------
# Independent re-derivations used by the check
# ------------------------------------------------------------------------------------------
def naive_bright(c, b):
    return c if b == 255 else min(255, (b * (c + 1)) >> 8)


def naive_tile(img, x, y, scale, bright):
    """Per-pixel: Mesen tile copy, premultiply already done in img, brightness, then 2x."""
    size = 8 * scale
    h, w = (img.shape[:2]) if img is not None else (0, 0)
    src = [[(0, 0, 0, 0)] * size for _ in range(size)]
    off = y * w + x
    if img is not None and 0 <= off and off + (size - 1) * w + size <= h * w:
        flat = img.reshape(-1, 4)
        for r in range(size):
            for c in range(size):
                px = flat[off + r * w + c]
                src[r][c] = tuple(naive_bright(int(px[k]), bright) for k in range(3)) + (int(px[3]),)
    return naive_2x(src, scale)


def naive_2x(src, scale):
    out = np.zeros((16, 16, 4), dtype=np.uint8)
    for oy in range(16):
        for ox in range(16):
            if scale == 1:
                out[oy, ox] = src[oy // 2][ox // 2]
            elif scale == 2:
                out[oy, ox] = src[oy][ox]
            else:
                acc, tot = [0, 0, 0, 0], 0
                for sy in range(len(src)):
                    wy = max(0, min((oy + 1) * scale, 2 * sy + 2) - max(oy * scale, 2 * sy))
                    if not wy:
                        continue
                    for sx in range(len(src)):
                        wx = max(0, min((ox + 1) * scale, 2 * sx + 2) - max(ox * scale, 2 * sx))
                        if wx:
                            for k in range(4):
                                acc[k] += src[sy][sx][k] * wy * wx
                            tot += wy * wx
                out[oy, ox] = [(a * 2 + tot) // (2 * tot) for a in acc]
    return out


def cond_matches(dec, c, slot):
    """Does decoded condition `dec` encode model condition `c`?"""
    if dec["type"] != c.type or dec["negate"] != c.negate:
        return False
    a = c.args
    if c.type == "memoryCheck":
        return (dec["op"] == a[1] and dec["mask"] == a[3] and dec["q1"] & 0xFF == slot[a[0]]
                and (dec["q1"] >> 8) & 0xFF == slot[a[2]])
    if c.type == "memoryCheckConstant":
        return (dec["op"] == a[1] and dec["mask"] == a[3] and dec["q1"] & 0xFF == slot[a[0]]
                and (dec["q1"] >> 8) & 0xFF == a[2] & 0xFF)
    if c.type == "frameRange":
        return dec["q1"] == a[0] | (a[1] << 16)
    if c.type.startswith(("position", "originPosition")):
        return dec["op"] == a[0] and dec["q1"] == a[1]
    if c.type in ("hmirror", "vmirror", "bgpriority"):
        return True
    x, y, idx, data, pal, ignore = a
    if dec["ignore"] != ignore or dec["q1"] != ((x & 0xFFFFFFFF) | ((y & 0xFFFFFFFF) << 32)):
        return False
    if dec["q3"] & 0xFFFFFFFF != pal:
        return False
    if data is not None:
        return dec["data"] and dec["q2"] == crc64(data)
    return not dec["data"] and dec["q2"] == idx & 0xFFFFFFFFFFFFFFFF


def eval_frame_cond(c, snap_by_addr, frame):
    """Plain evaluation of a memory/frameRange condition, for the background index test."""
    if c.type == "memoryCheckConstant":
        a, op, b, mask = c.args
        x, y = snap_by_addr[a] & mask, b
    elif c.type == "memoryCheck":
        a, op, b, mask = c.args
        x, y = snap_by_addr[a] & mask, snap_by_addr[b] & mask
    elif c.type == "frameRange":
        r = frame % c.args[0] >= c.args[1]
        return r != c.negate
    else:
        return None                     # tile/sprite position: decided by the caller
    r = {"eq": x == y, "ne": x != y, "gt": x > y, "lt": x < y, "le": x <= y, "ge": x >= y}[op]
    return r != c.negate


def check_additions_fallback(h, p, rom_chr, has_chr_ram=None):
    """Game CHR kind, additions and the fallback map, re-derived without the converter.
    has_chr_ram: Mesen's HasChrRam() for the ROM (decides which additions survive); None when
    no ROM was given, then it is the game's CHR kind."""
    errors = []
    known, ram_flag = bool(h.flags >> 5 & 1), bool(h.flags >> 4 & 1)
    if rom_chr is not None:
        want = len(rom_chr) == 0
    else:
        kinds = set()
        for t in p.tiles:
            kinds.add(t.key.chr_ram)
        for a in p.additions:
            kinds.add(a.added.chr_ram)
        want = kinds.pop() if len(kinds) == 1 else None
    if (want is None) != (not known) or (want is not None and want != ram_flag):
        errors.append(f"game CHR kind: file says known={known} ram={ram_flag}, expected {want}")
        return errors
    game_ram = want
    keep_ram = game_ram if has_chr_ram is None else has_chr_ram
    adds = [a for a in p.additions if a.added.chr_ram == keep_ram]
    if h.nadd != len(adds) or h.sec[11][1] != 64 * len(adds):
        errors.append(f"additions: file has {h.nadd}, expected {len(adds)}")
        return errors

    def body(k):
        return (1, crc64(k.data)) if k.chr_ram else (0, k.index & 0xFFFFFFFFFFFFFFFF)

    for i, a in enumerate(adds):
        rec = struct.unpack_from("<IIQIIQiiI", h.d, h.s(11) + 64 * i)
        ok_, ob = body(a.original)
        ak, ab = body(a.added)
        want_rec = (a.original.palette, ok_, ob, a.added.palette, ak, ab, a.ox, a.oy,
                    int(a.ignore_palette))
        if rec != want_rec:
            errors.append(f"addition {i}: {rec} != {want_rec}")
    # fallback map
    table = {}
    if not game_ram:
        if p.options & M.OPT_AUTOMATIC_FALLBACK_TILES:
            if rom_chr is None:
                errors.append("cannot check automatic fallbacks without --rom")
                return errors
            n = len(rom_chr) // 16
            first = {}
            key = bytearray(16)                     # one reused HdTileKey, zero-initialised
            for t in p.tiles:
                i = t.key.index
                address = (i * 16) % (1 << 32)      # uint32 parameter of CopyChrTile
                if address <= len(rom_chr) - 16:    # otherwise memcpy is skipped
                    key[:] = rom_chr[address:address + 16]
                d = bytes(key)
                if d not in first:                  # unordered_set::emplace keeps the first
                    first[d] = i
            for i in range(n):
                d = bytes(rom_chr[16 * i:16 * i + 16])
                if d != bytes(16) and d in first and first[d] != i:
                    table[i] = first[d]
        for i, f in p.fallbacks:
            if i >= 0:
                table[i] = f
    got = list(struct.unpack_from(f"<{h.nfallback}i", h.d, h.s(10))) if h.nfallback else []
    want_arr = [-1] * (max(table) + 1 if table else 0)
    for i, f in table.items():
        want_arr[i] = f
    if got != want_arr:
        errors.append(f"fallback map differs ({len(got)} vs {len(want_arr)} entries)")
    errors += check_rewrite_sensitive(h, p, got)
    errors += check_add_tables(h)
    return errors


def check_add_tables(h):
    """Sections 16/17 (8a): every key that some addition is filed under probes, through the
    hardware's lookup, to exactly the additions filed under it, in file order, with their
    offsets and added keys. Expected lists come from a plain scan of section 11, not from the
    table builder."""
    errors = []
    if h.nadd == 0:
        if h.sec[16] != (0, 0) or h.sec[17] != (0, 0) or (h.add_nb, h.add_mp) != (0, 0):
            errors.append("addition tables present with no additions")
        return errors
    if h.add_nb == 0 or h.add_nb & (h.add_nb - 1) or h.sec[16][1] != 64 * h.add_nb:
        errors.append(f"addition table: {h.add_nb} buckets, section 16 is {h.sec[16][1]} bytes")
        return errors
    want = {}
    for i in range(h.nadd):
        opal, okind, obody, apal, akind, abody, ox, oy, flags = \
            struct.unpack_from("<IIQIIQiiI", h.d, h.s(11) + 64 * i)
        key = (0xFFFFFFFF if flags & 1 else opal, okind, obody)
        want.setdefault(key, []).append((i, ox, oy, akind, apal, abody))
    table = h.d[h.s(16):h.s(16) + h.sec[16][1]]
    lists = h.d[h.s(17):h.s(17) + h.sec[17][1]]
    for (pal, kind, body), items in want.items():
        got = hdp3_addtab.lookup(table, h.add_nb, h.add_mp, lists, pal, kind, body)
        if got != items:
            if [g[0] for g in got] != [it[0] for it in items]:
                what = f"additions {[g[0] for g in got]}, expected {[it[0] for it in items]}"
            else:
                g, w = next((g, w) for g, w in zip(got, items) if g != w)
                what = (f"addition {w[0]} entry (index, ox, oy, kind, palette, body) "
                        f"{g}, expected {w}")
            errors.append(f"addition key {pal:08x}/{kind}/{body:x}: table gives {what}")
            if len(errors) > 10:
                break
    return errors


def check_rewrite_sensitive(h, p, fb):
    """Condition bit 24, re-derived by brute force: for every looked-up record (palette P of a
    key in the pack, tile i), apply Mesen's rewrite (exact key missing, fallback key present
    for P or the default palette) and ask whether it can change the condition's tile test.
    The converter derives the same set by a shortcut (2-hop table); this enumerates."""
    errors = []

    def fbk(i):
        return fb[i] if 0 <= i < len(fb) else -1

    keys = p.tile_by_key
    pals = {k[2] for k in keys if k[0] == "rom"}

    def has(pal, i):
        return ("rom", M.to_int32(i), pal) in keys

    def tmatch(i, t):
        return i == t or fbk(i) == t

    # every (palette, index) a rewrite can produce from: index i with a fallback
    cands = [i for i in range(len(fb)) if fb[i] >= 0]
    sens_by = {}
    for ci in range(h.nconds):
        d = h.cond(ci)
        if d["type"] not in ("tileAtPosition", "spriteAtPosition", "tileNearby",
                             "spriteNearby") or d["data"]:
            if d["sens"]:
                errors.append(f"condition {ci}: sensitive flag on a non-index condition")
            continue
        t = d["q2"] - (1 << 64) if d["q2"] >> 63 else d["q2"]
        cpal = d["q3"] & 0xFFFFFFFF
        key = (t, cpal, d["ignore"])
        if key not in sens_by:
            want = False
            for i in cands:
                f = fbk(i)
                if tmatch(i, t) == tmatch(f, t):
                    continue
                if d["ignore"]:
                    want = any(not has(P, i) and (has(P, f) or has(0xFFFFFFFF, f))
                               for P in pals)
                else:
                    want = not has(cpal, i) and (has(cpal, f) or has(0xFFFFFFFF, f))
                if want:
                    break
            sens_by[key] = want
        if d["sens"] != sens_by[key]:
            errors.append(f"condition {ci} (tile {t}): sensitive flag {d['sens']}, "
                          f"expected {sens_by[key]}")
    return errors


def read_patches(h):
    """Section 14 decoded here, separately from hdp3_patch.parse: {sha1 upper: entry}."""
    off, n = h.sec[14]
    if not n:
        return {}
    d = h.d[off:off + n]
    out = {}
    for k in range(struct.unpack_from("<I", d, 0)[0]):
        e = 64 + 64 * k
        sha = d[e:e + 20].hex().upper()
        base, flags = d[e + 20], d[e + 21]
        (ssz, tsz, trunc, crc, roff, rn, doff, dl) = struct.unpack_from("<8I", d, e + 24)
        runs = []
        for j in range(rn):
            t, ln, arg = struct.unpack_from("<3I", d, roff + 16 * j)
            runs.append((t, ln, arg, d[roff + 16 * j + 12]))
        out[sha] = dict(base=base, flags=flags, src_size=ssz, tgt_size=tsz, trunc=trunc,
                        crc=crc, runs=runs, data=bytes(d[doff:doff + dl]))
    return out


def check_patches(h, p, rom_path=None, romindex=None):
    """Every <patch> the pack lists (and each MD5 line resolved through the ROM index,
    rommatch.section_keys) is in section 14 as an independent compile of the patch file; runs
    are sorted and disjoint. With --rom, the entry the core picks for that ROM (its file hash,
    then its iNES 1.0 form - rommatch.core_lookup), built the way the hardware builds it from
    that form, equals rompatch.apply of the same form."""
    import hdp3_patch
    import rommatch
    import rompatch
    errors = []
    got = read_patches(h)
    want = {}
    for sha, name in rommatch.section_keys(p.patches, p.patches_md5, romindex).items():
        try:
            want[sha] = hdp3_patch.compile_patch(p.read(name))
        except (rompatch.PatchError, FileNotFoundError):
            pass                                  # left out on purpose: loads unpatched
    if set(got) != set(want):
        errors.append(f"patch entries {sorted(got)} vs pack {sorted(want)}")
    for sha, c in want.items():
        e = got.get(sha)
        if e is None:
            continue
        for k in ("base", "flags", "src_size", "tgt_size", "trunc", "crc", "data"):
            if e[k] != c[k]:
                errors.append(f"patch {sha[:8]}: {k} differs")
        if [tuple(r) for r in e["runs"]] != [tuple(r) for r in c["runs"]]:
            errors.append(f"patch {sha[:8]}: runs differ")
        end = 0
        for t, ln, _a, m in e["runs"]:
            if t < end or ln == 0 or m > 4:
                errors.append(f"patch {sha[:8]}: runs not sorted/disjoint")
                break
            end = t + ln
    if rom_path:
        with open(rom_path, "rb") as f:
            rom = f.read()
        hit = rommatch.core_lookup(rom, set(got))
        if hit:
            src, sha, _label = hit
            name = rommatch.section_keys(p.patches, p.patches_md5, romindex).get(sha)
            try:
                ref = rompatch.apply(src, p.read(name)) if name else None
            except (rompatch.PatchError, FileNotFoundError):
                ref = None
            if hdp3_patch.build(got[sha], src) != ref:
                errors.append(f"patch {sha[:8]}: hardware build differs from rompatch")
    return errors


def check_audio(h, p, limit=300000):
    """Section 15 against the pack's <bgm>/<sfx> lines: one entry per line, sorted by key, each
    stream inside the section, and each stream checked against Mesen's PCM rather than against
    the converter's encoder: oggdecode render gives the PCM (Mesen's decoder and resampler),
    tools/audio/hdaudio decodes the stored ADPCM with the hardware's rules from state0, and the
    two must agree to an SNR floor; frames and loop frame must equal render's, and decoding up
    to the loop frame must arrive at exactly statel. Streams are decoded up to `limit` frames
    (the loop-state check runs only when the loop frame is inside that)."""
    import subprocess
    import tempfile
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(here, "audio"))
    import hdaudio as A
    errors = []
    off, ln = h.sec[15]
    want = {(0 << 16) | i: (f, lp) for i, (f, lp) in p.bgm.items()}
    want.update({(1 << 16) | i: (f, 0) for i, f in p.sfx.items()})
    if not ln:
        if want:
            errors.append(f"pack has {len(want)} audio lines but no section 15")
        return errors
    sec = h.d[off:off + ln]
    try:
        ents = A.parse_section(sec)
    except ValueError as e:
        return [f"audio: {e}"]
    keys = [e.key for e in ents]
    if keys != sorted(keys) or len(set(keys)) != len(keys):
        errors.append("audio keys not sorted/unique")
    if set(keys) != set(want):
        errors.append(f"audio keys {len(keys)} vs pack lines {len(want)}")
    if not ents:
        return errors
    data_off = struct.unpack_from("<I", sec, 8)[0]
    dec = os.path.join(here, "audio", "oggdecode.exe")
    seen = {}
    with tempfile.TemporaryDirectory() as tmp:
        src, out = os.path.join(tmp, "in.ogg"), os.path.join(tmp, "out.pcm")
        for e in ents:
            if e.key not in want:
                continue
            fname, loop = want[e.key]
            if e.frames and not (data_off <= e.start and e.start + e.frames <= ln):
                errors.append(f"audio {e.key:#x}: stream outside the section")
                continue
            if e.frames and e.loop >= e.frames:
                errors.append(f"audio {e.key:#x}: loop frame past the end")
            k = (fname, loop)
            if k not in seen:
                with open(src, "wb") as f:
                    f.write(p.read(fname))
                r = subprocess.run([dec, "render", src, out, str(A.RATE), str(loop)],
                                   capture_output=True, text=True)
                if r.returncode:
                    seen[k] = None
                else:
                    kv = dict(x.split("=", 1) for x in r.stdout.split())
                    with open(out, "rb") as f:
                        pcm = f.read(4 * limit)
                    seen[k] = (int(kv["frames"]), int(kv["loopframe"]), pcm, None)
            ref = seen[k]
            if ref is None:
                if e.frames:
                    errors.append(f"audio {fname}: does not decode, but has {e.frames} frames")
                continue
            frames, lf, pcm, done = ref
            if (e.frames, e.loop) != (frames, lf):
                errors.append(f"audio {fname}: frames/loop {e.frames}/{e.loop} vs "
                              f"render {frames}/{lf}")
                continue
            if done is not None and done[0] == e.start and done[1] == (e.state0, e.statel):
                continue                      # same stream and states already verified
            n = min(e.frames, len(pcm) // 4)
            pl, il, pr, ir = A.unpack_state(e.state0)
            if not e.loop and e.statel != e.state0:
                errors.append(f"audio {fname}: no loop, but statel != state0")
            sig = err = 0
            # decode at least to the loop frame: the statel check must not depend on the limit
            for i in range(max(n, e.loop)):
                if i == e.loop and e.loop and A.pack_state(pl, il, pr, ir) != e.statel:
                    errors.append(f"audio {fname}: state at loop frame != statel")
                b = sec[e.start + i]
                pl, il = A.ima_decode(pl, il, b & 15)
                pr, ir = A.ima_decode(pr, ir, b >> 4)
                if i < n:
                    l, r = struct.unpack_from("<hh", pcm, 4 * i)
                    sig += l * l + r * r
                    err += (l - pl) ** 2 + (r - pr) ** 2
            if e.loop and e.loop == max(n, e.loop) and A.pack_state(pl, il, pr, ir) != e.statel:
                errors.append(f"audio {fname}: state at loop frame != statel")
            if sig and 10 * np.log10(sig / max(err, 1)) < 12:
                errors.append(f"audio {fname}: decoded stream off Mesen's PCM "
                              f"({10 * np.log10(sig / max(err, 1)):.1f} dB)")
            seen[k] = (frames, lf, pcm, (e.start, (e.state0, e.statel)))
    errors += check_audio_checkpoints(sec, ents, dec)
    return errors


def check_audio_checkpoints(sec, ents, dec):
    """The section's restore-checkpoint table (hdaudio.build_section, since 2026-10-29): present,
    at hdaudio's spacing, one state per CK_SPACING frames of every entry in table order, and each
    state the one decoding the stream gives there (oggdecode ckscan, over the whole stream - the
    ARM restores from these, so a wrong one plays noise after a savestate load)."""
    import subprocess
    import tempfile
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(here, "audio"))
    import hdaudio as A
    errors = []
    t = A.parse_checkpoints(sec)
    if t is None:
        return ["audio: no checkpoint table (a section built before 2026-10-29 - convert again)"]
    spacing, ck = t
    want = sum(e.frames // A.CK_SPACING + 1 for e in ents)
    if spacing != A.CK_SPACING or len(ck) != want:
        return [f"audio: checkpoint table spacing {spacing} / {len(ck)} states, want "
                f"{A.CK_SPACING} / {want}"]
    at, done = 0, {}
    with tempfile.TemporaryDirectory() as tmp:
        src, out = os.path.join(tmp, "s.ima"), os.path.join(tmp, "s.ck")
        for e in ents:
            n = e.frames // A.CK_SPACING + 1
            mine = ck[at:at + n]
            at += n
            key = (e.start, e.frames, e.state0)
            if key not in done:
                with open(src, "wb") as f:
                    f.write(sec[e.start:e.start + e.frames])
                r = subprocess.run([dec, "ckscan", src, out, "%016x" % e.state0],
                                   capture_output=True, text=True)
                if r.returncode:
                    errors.append(f"audio {e.key:#x}: ckscan failed ({r.stderr.strip()})")
                    done[key] = None
                    continue
                b = open(out, "rb").read()
                done[key] = list(struct.unpack("<%dQ" % (len(b) // 8), b))
            ref = done[key]
            if ref is not None and mine != ref:
                bad = next(i for i, (x, y) in enumerate(zip(mine, ref)) if x != y) \
                    if len(mine) == len(ref) else -1
                errors.append(f"audio {e.key:#x}: checkpoint {bad} differs from the stream's "
                              f"own state")
    return errors


def check(h, p, samples, rom_chr=None, has_chr_ram=None, rom_path=None, romindex=None):
    errors = check_additions_fallback(h, p, rom_chr, has_chr_ram)
    errors += check_patches(h, p, rom_path, romindex)
    errors += check_audio(h, p)
    slot = {a: i for i, a in enumerate(h.watched)}
    if h.watched != sorted(p.watched):
        errors.append("watched list differs")
    img_cache = OrderedDict()             # bounded: big packs decode to several GB
    cache_bytes = [0]

    def img(name):
        if name in img_cache:
            img_cache.move_to_end(name)
            return img_cache[name]
        try:
            _, _, im = pngdec.decode_fast(p.read(name))
            im = pngdec.premultiply(im)
        except (pngdec.PngError, OSError):
            im = None
        img_cache[name] = im
        cache_bytes[0] += im.nbytes if im is not None else 0
        while cache_bytes[0] > (1 << 30) and len(img_cache) > 1:
            _, old = img_cache.popitem(last=False)
            cache_bytes[0] -= old.nbytes if old is not None else 0
        return im

    # tiles
    checked_blocks = set()
    for key, idxs in p.tile_by_key.items():
        if key[0] == "rom":
            pal, kind, lo, hi = key[2], 0, key[1] & 0xFFFFFFFF, 0
        else:
            f = crc64(key[2])
            pal, kind, lo, hi = key[1], 1, f & 0xFFFFFFFF, f >> 32
        off = h.find_key(pal, kind, lo, hi)
        if off is None:
            errors.append(f"key {key[:2]} not found")
            continue
        rl = h.rules(off)
        if [r["src"] for r in rl] != [i & 0xFFFFFF for i in idxs]:
            errors.append(f"key {key[:2]}: rule order differs")
            continue
        prefix = 0
        for r, ti in zip(rl, idxs):
            t = p.tiles[ti]
            prefix |= int(t.force_disable_cache)
            if r["fdc"] != int(t.force_disable_cache) or r["prefix"] != prefix:
                errors.append(f"tile {ti}: fdc flags")
            dec = [h.cond(i) for i in h.clist(r["frame"]) + h.clist(r["walk"])]
            if any(d["frame"] != (d["type"] in M.Condition.PER_FRAME) for d in dec):
                errors.append(f"tile {ti}: class bit")
            if any(d["frame"] for d in (h.cond(i) for i in h.clist(r["walk"]))):
                errors.append(f"tile {ti}: frame condition in walk list")
            want = list(t.conditions)
            for c in want:
                if not any(cond_matches(d, c, slot) for d in dec):
                    errors.append(f"tile {ti}: condition {c.name} missing")
                    break
            if len({(c.type, c.negate, c.args) for c in want}) != len(dec):
                errors.append(f"tile {ti}: {len(dec)} conditions, pack has {len(want)}")
            if ti in checked_blocks or len(checked_blocks) >= samples:
                continue
            checked_blocks.add(ti)
            exp = naive_tile(img(p.images[t.bitmap]), t.x, t.y, p.scale, t.brightness)
            if not exp.any():
                if r["payload"] != 0xFFFFFFFF:
                    errors.append(f"tile {ti}: transparent tile has a block")
            elif r["payload"] == 0xFFFFFFFF or not np.array_equal(h.block(r["payload"]), exp):
                errors.append(f"tile {ti}: block pixels differ")

    # backgrounds
    rec_i = 0
    rng = random.Random(1)
    map_seen = {}
    spot = []                     # deferred pixel spot-checks, run grouped by file below
    dims = {}                     # file -> (w, h) or None; decoding per ENTRY thrashed the
                                  # cache on Metroid HD (50,903 entries over three 1 GB maps)

    def dims_of(name):
        if name not in dims:
            im = img(name)
            dims[name] = None if im is None else (im.shape[1], im.shape[0])
        return dims[name]

    for lvl, entries in enumerate(p.backgrounds):
        roff, ioff = h.bg_level(lvl)
        if not entries:
            if roff:
                errors.append(f"level {lvl}: unexpected list")
            continue
        rl = h.rules(roff)
        if len(rl) != len(entries):
            errors.append(f"level {lvl}: {len(rl)} entries, pack has {len(entries)}")
            continue
        for ei, (r, bg) in enumerate(zip(rl, entries)):
            dec = [h.cond(i) for i in h.clist(r["frame"])]
            if r["walk"] or not all(d["frame"] for d in dec):
                errors.append(f"level {lvl} entry {ei}: background with a walk condition")
            for d in dec:
                if not any(cond_matches(d, c, slot) for c in bg.conditions):
                    errors.append(f"level {lvl} entry {ei}: stray condition {d['type']}")
            for c in bg.conditions:
                eq = (c.type == "memoryCheckConstant" and not c.negate and c.args[1] == "eq")
                if not any(cond_matches(d, c, slot) for d in dec) and not (ioff and eq):
                    errors.append(f"level {lvl} entry {ei}: condition {c.name} missing")
            rec = h.bg_rec(r["payload"])
            if r["payload"] != rec_i + ei:
                errors.append(f"level {lvl} entry {ei}: record index")
            dm = dims_of(bg.file)
            wn, hn = (dm[0] // p.scale, dm[1] // p.scale) if dm is not None else (0, 0)
            if (rec["w"], rec["h"], rec["left"], rec["top"]) != (wn, hn, bg.left, bg.top):
                errors.append(f"level {lvl} entry {ei}: record fields")
            if rec["blend"] != {"Alpha": 0, "Add": 1, "Subtract": 2}[bg.blend]:
                errors.append(f"level {lvl} entry {ei}: blend")
            for s in (0, 1, 255, 256, 511):
                for ratio, ti in ((bg.hratio, rec["hr"]), (bg.vratio, rec["vr"])):
                    want = int(float(np.float32(s) * np.float32(ratio)))
                    if h.ratio(ti, s) != want:
                        errors.append(f"level {lvl} entry {ei}: ratio table")
            # spot-check map cells against the PNG, pixel by pixel, once per (file, brightness)
            mkey = (bg.file.lower(), bg.brightness)
            if mkey in map_seen:
                if map_seen[mkey] != rec["map"]:
                    errors.append(f"level {lvl} entry {ei}: same image, different map")
            elif dm is not None and wn and hn:
                map_seen[mkey] = rec["map"]
                spot.append((bg.file, bg.brightness, rec["map"], wn, hn, lvl, ei))
        rec_i += len(entries)
        # selection: index (if any) vs linear scan over the pack's own conditions
        addrs = sorted({c.args[0] for bg in entries for c in bg.conditions
                        if c.type in ("memoryCheckConstant", "memoryCheck")}
                       | {c.args[2] for bg in entries for c in bg.conditions
                          if c.type == "memoryCheck"})
        values = {a: sorted({c.args[2] & 0xFF for bg in entries for c in bg.conditions
                             if c.type == "memoryCheckConstant" and c.args[0] == a}) for a in addrs}
        if not ioff:
            continue                     # no index: the decoded-condition checks above suffice
        for trial in range(200):
            snap = {}
            for a in addrs:
                v = values.get(a) or [0]
                snap[a] = rng.choice(v) if rng.random() < 0.9 else rng.randrange(256)
            if trial % 2 == 0:           # aim at one entry: satisfy its EQ terms exactly
                for c in entries[rng.randrange(len(entries))].conditions:
                    if c.type == "memoryCheckConstant" and c.args[1] == "eq" and not c.negate:
                        a, _, v, mask = c.args
                        snap[a] = (snap[a] & ~mask & 0xFF) | (v & mask & 0xFF)
            pos_truth = {}
            frame = rng.randrange(1, 100000)

            def passes(conds):
                for c in conds:
                    r = eval_frame_cond(c, snap, frame)
                    if r is None:
                        key = id(c)
                        if key not in pos_truth:
                            pos_truth[key] = rng.random() < 0.5
                        r = pos_truth[key]
                    if not r:
                        return False
                return True

            linear = next((i for i, bg in enumerate(entries) if passes(bg.conditions)), None)
            snap_slot = [0] * 256
            for a, v in snap.items():
                snap_slot[slot[a]] = v
            got = next((i for i in h.bg_index_candidates(ioff, snap_slot)
                        if passes(entries[i].conditions)), None)
            if got != linear:
                errors.append(f"level {lvl}: index picks {got}, linear scan picks {linear}")
                break
    # map cells against the PNG, pixel by pixel, once per (file, brightness), grouped by file
    # so each image is decoded once
    for file, bright, mp, wn, hn, lvl, ei in sorted(spot, key=lambda t: (t[0].lower(), t[1])):
        im = img(file)
        cw = (wn + 7) // 8
        for _ in range(6):
            cx, cy = rng.randrange(cw), rng.randrange((hn + 7) // 8)
            (bi,) = struct.unpack_from("<I", h.d, h.s(7) + mp + 4 * (cy * cw + cx))
            src = [[(0, 0, 0, 0)] * (8 * p.scale) for _ in range(8 * p.scale)]
            for yy in range(8 * p.scale):
                for xx in range(8 * p.scale):
                    gy, gx = cy * 8 * p.scale + yy, cx * 8 * p.scale + xx
                    if gy < hn * p.scale and gx < wn * p.scale:
                        px = im[gy, gx]
                        src[yy][xx] = tuple(naive_bright(int(px[k]), bright)
                                            for k in range(3)) + (int(px[3]),)
            exp = naive_2x(src, p.scale)
            got = np.zeros((16, 16, 4), np.uint8) if bi == 0xFFFFFFFF else h.block(bi)
            if not np.array_equal(got, exp):
                errors.append(f"level {lvl} entry {ei}: map cell {cx},{cy}")
    return errors


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("hdp")
    ap.add_argument("pack_dir")
    ap.add_argument("--samples", type=int, default=400)
    ap.add_argument("--rom")
    ap.add_argument("--base", action="append", default=[])
    ap.add_argument("--romindex", default=DEFAULT_ROMINDEX,
                    help="the table the converter keyed MD5 <patch> lines with ('none' = no index)")
    a = ap.parse_args()
    with open(a.hdp, "rb") as f:
        h = Hdp3(f.read())
    p = load_pack(a.pack_dir, a.base)
    index = None
    if a.romindex != "none" and os.path.isfile(a.romindex):
        import rommatch
        index = rommatch.load_index(a.romindex)
    errs = check(h, p, a.samples, read_rom_chr(a.rom, p) if a.rom else None,
                 rom_chr_ram(a.rom, p) if a.rom else None, a.rom, index)
    print(f"{a.hdp}: {len(errs)} problems")
    for e in errs[:30]:
        print("  " + e)
    sys.exit(1 if errs else 0)


if __name__ == "__main__":
    main()
