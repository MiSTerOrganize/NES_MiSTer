"""Convert a Mesen HD pack into a format-3 .hdp file (layout: HDPACK_DESIGN.md, "Format 3").

The pack is parsed by hdpack_model (the port of Mesen2's HdPackLoader); this file only lays
the parsed pack out for the hardware. Anything Mesen would do differently from what the format
can express is either refused with a reason or listed in the warnings, never dropped silently.

usage: python hdp3_convert.py <pack_dir> <out.hdp> [--rom game.nes] [--base base_pack_dir ...]
"""
import argparse
import os
import struct
import sys
from collections import Counter, OrderedDict

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hdpack_model as M  # noqa: E402
import pngdec  # noqa: E402

FORMAT_VERSION = 3
HEADER_SIZE = 512
ALIGN = 64
NONE32 = 0xFFFFFFFF
BG_INDEX_MIN = 33              # levels with more entries than this get a value index
SIG_MAX_TERMS = 16
N_SECTIONS = 18             # 16/17 = the addition tables (8a, built by hdp3_addtab.py)
PACK_WINDOW = 192 << 20     # DDR3 0x30000000-0x3BFFFFFF: the file, byte 0 at the base
OVERFLOW = 192 << 20        # DDR3 0x24000000-0x2FFFFFFF: file bytes past PACK_WINDOW (audio)
(S_KEYS, S_RULES, S_CONDS, S_CLISTS, S_BLOCKS, S_BGLEVELS, S_BGRECS, S_BGMAPS, S_RATIOS,
 S_WATCHED, S_FALLBACK, S_ADDITIONS, S_PALETTE, S_BGINDEX, S_PATCHES, S_AUDIO,
 S_ADDKEYS, S_ADDLISTS) = range(18)

COND_TYPES = {"memoryCheck": 0, "memoryCheckConstant": 1, "frameRange": 2,
              "tileAtPosition": 3, "spriteAtPosition": 4, "tileNearby": 5, "spriteNearby": 6,
              "positionCheckX": 7, "positionCheckY": 8, "originPositionCheckX": 9,
              "originPositionCheckY": 10, "hmirror": 11, "vmirror": 12, "bgpriority": 13}
FRAME_CLASS = set(M.Condition.PER_FRAME)
OP_CODES = {"eq": 0, "ne": 1, "gt": 2, "lt": 3, "le": 4, "ge": 5}
# walk-class evaluation order: cheapest first (pure, so order does not change the result)
WALK_ORDER = {"hmirror": 0, "vmirror": 0, "bgpriority": 0, "positionCheckX": 1,
              "positionCheckY": 1, "originPositionCheckX": 1, "originPositionCheckY": 1,
              "tileNearby": 2, "spriteNearby": 2}
BLENDS = {"Alpha": 0, "Add": 1, "Subtract": 2}


class Refused(Exception):
    pass


# ------------------------------------------------------------------------------------------
# Hashes
# ------------------------------------------------------------------------------------------
def _crc_table(poly, bits):
    top = 1 << (bits - 1)
    mask = (1 << bits) - 1
    t = []
    for b in range(256):
        r = b << (bits - 8)
        for _ in range(8):
            r = ((r << 1) ^ poly) & mask if r & top else (r << 1) & mask
        t.append(r)
    return t


_T32 = _crc_table(0x04C11DB7, 32)
_T64 = _crc_table(0x42F0E1EBA9EA3693, 64)


def crc32(data):
    """CRC-32, poly 0x04C11DB7, init 0, MSB first, no reflection, no final XOR."""
    r = 0
    for b in data:
        r = ((r << 8) & 0xFFFFFFFF) ^ _T32[((r >> 24) ^ b) & 0xFF]
    return r


def crc64(data):
    """CRC-64, poly 0x42F0E1EBA9EA3693, init 0, MSB first, no reflection, no final XOR."""
    r = 0
    for b in data:
        r = ((r << 8) & 0xFFFFFFFFFFFFFFFF) ^ _T64[((r >> 56) ^ b) & 0xFF]
    return r


ZERO_FP = crc64(bytes(16))


# ------------------------------------------------------------------------------------------
# Images
# ------------------------------------------------------------------------------------------
def to_2x(px, scale):
    """[H*s, W*s, 4] uint8 (H, W in NES pixels) -> [2H, 2W, 4] uint8, premultiplied RGBA.
    Each NES pixel's s x s source block becomes 2 x 2 output pixels; output pixel o covers
    source half-units [o*s, (o+1)*s) on each axis, weighted by overlap, rounded half up.
    Exact copies for scale 1 (doubling) and 2."""
    if scale == 1:
        return np.repeat(np.repeat(px, 2, axis=0), 2, axis=1)
    if scale == 2:
        return px.copy()
    h, w = px.shape[0] // scale, px.shape[1] // scale
    wt = np.zeros((2, scale), dtype=np.int64)              # [2, s] weights of one NES pixel
    for o in range(2):
        for i in range(scale):
            wt[o, i] = max(0, min((o + 1) * scale, 2 * i + 2) - max(o * scale, 2 * i))
    v = px.reshape(h, scale, w, scale, 4).astype(np.int64)
    out = np.einsum("ia,nambc,jb->nimjc", wt, v, wt).reshape(2 * h, 2 * w, 4)
    den = scale * scale
    return ((out * 2 + den) // (2 * den)).astype(np.uint8)


def adjust_brightness(img, b):
    """HdNesPack::AdjustBrightness on premultiplied RGB, alpha unchanged, packed as Mesen packs
    it (r | g<<8 | b<<16 | a<<24 into a u32, so out-of-range ints spill as in C++)."""
    if b == 255:
        return img
    c = img[:, :, :3].astype(np.int64)
    v = np.minimum(255, (b * (c + 1)) >> 8)
    packed = ((v[:, :, 0] & 0xFFFFFFFF) | ((v[:, :, 1] << 8) & 0xFFFFFFFF)
              | ((v[:, :, 2] << 16) & 0xFFFFFFFF) | (img[:, :, 3].astype(np.int64) << 24))
    packed &= 0xFFFFFFFF
    out = np.empty_like(img)
    for i in range(4):
        out[:, :, i] = (packed >> (8 * i)) & 0xFF
    return out


class Images:
    """Decoded images, kept under a byte budget (least recently used dropped first).
    A big scale-3 pack holds several GB of decoded backgrounds, which once exhausted memory."""
    BUDGET = 1 << 30

    def __init__(self, pack, warn):
        self.p = pack
        self.warn = warn
        self.cache = OrderedDict()
        self.bytes = 0
        self.bad = set()

    def get(self, name):
        """Premultiplied RGBA [h, w, 4], or None when Mesen fails to decode it."""
        if name in self.bad:
            return None
        img = self.cache.get(name)
        if img is not None:
            self.cache.move_to_end(name)
            return img
        try:
            _, _, img = pngdec.decode_fast(self.p.read(name))
            img = pngdec.premultiply(img)
        except pngdec.TooLarge as e:
            raise Refused(f"PNG {name}: {e}")
        except (pngdec.PngError, OSError) as e:
            self.warn(f"PNG {name} is invalid ({e}); Mesen draws nothing from it")
            self.bad.add(name)
            return None
        self.cache[name] = img
        self.bytes += img.nbytes
        while self.bytes > self.BUDGET and len(self.cache) > 1:
            _, old = self.cache.popitem(last=False)
            self.bytes -= old.nbytes
        return img


def tile_pixels(img, x, y, size):
    """HdPackTileInfo::Init: rows copied linearly from Y*W+X (u32 arithmetic); a tile that does
    not fit the bitmap's data stays all zero."""
    out = np.zeros((size, size, 4), dtype=np.uint8)
    if img is None:
        return out
    h, w = img.shape[:2]
    off = (y * w + x) & 0xFFFFFFFF
    need = (off + (size - 1) * w + size) & 0xFFFFFFFF
    if h * w < need:
        return out
    if off + (size - 1) * w + size > h * w:     # only reachable through u32 wrap
        raise Refused(f"tile at {x},{y} wraps the 32-bit offset; Mesen would read out of bounds")
    flat = img.reshape(-1, 4)
    for r in range(size):
        out[r] = flat[off + r * w: off + r * w + size]
    return out


# ------------------------------------------------------------------------------------------
# Converter
# ------------------------------------------------------------------------------------------
class Converter:
    def __init__(self, pack, rom=None, has_chr_ram=None, romindex=None):
        self.p = pack
        self.rom = rom
        self.has_chr_ram = has_chr_ram      # BaseMapper::HasChrRam(); None = no ROM given
        self.romindex = romindex            # rommatch.load_index rows, for MD5 <patch> lines
        self.warnings = Counter()
        self.images = Images(pack, self.warn)
        self.blocks = []            # bytes(1024)
        self.block_of = {}
        self.conds = []             # encoded 32-byte records
        self.cond_of = {}
        self.n_frame = 0
        self.clists = bytearray(4)  # offset 0 means "no list"
        self.clist_of = {}
        self.fps = {}               # fingerprint -> data (collision check)

    def warn(self, msg):
        self.warnings[msg] += 1

    # -- fingerprints --------------------------------------------------------------------
    def fp(self, data):
        f = crc64(data)
        prev = self.fps.setdefault(f, data)
        if prev != data:
            raise Refused(f"fingerprint collision between {prev.hex()} and {data.hex()}")
        return f

    # -- watched addresses ---------------------------------------------------------------
    def build_watched(self):
        self.watched = sorted(self.p.watched)
        if len(self.watched) > 256:
            raise Refused(f"{len(self.watched)} watched addresses (format limit 256)")
        self.slot = {a: i for i, a in enumerate(self.watched)}

    # -- conditions ----------------------------------------------------------------------
    def cond_index(self, c):
        key = (c.type, c.negate, c.args)
        if key in self.cond_of:
            return self.cond_of[key]
        t = COND_TYPES[c.type]
        flags = t | (int(c.negate) << 8)
        frame = c.type in FRAME_CLASS
        q1 = q2 = q3 = 0
        mask = 0
        op = 0
        memo = 0
        if frame:
            flags |= 1 << 11
            memo = self.n_frame
            self.n_frame += 1
        if c.type == "memoryCheck":
            a, op_s, b, mask = c.args
            op = OP_CODES[op_s]
            q1 = self.slot[a] | (self.slot[b] << 8)
        elif c.type == "memoryCheckConstant":
            a, op_s, b, mask = c.args
            op = OP_CODES[op_s]
            q1 = self.slot[a] | ((b & 0xFF) << 8)
        elif c.type == "frameRange":
            a, b = c.args
            if a == 0:
                raise Refused(f"frameRange {c.name} with A = 0 (Mesen divides by zero)")
            q1 = a | (b << 16)
        elif c.type in ("positionCheckX", "positionCheckY", "originPositionCheckX",
                        "originPositionCheckY"):
            op_s, v = c.args
            op = OP_CODES[op_s]
            q1 = v & 0xFFFFFFFF
        elif c.type in ("tileAtPosition", "spriteAtPosition", "tileNearby", "spriteNearby"):
            x, y, idx, data, pal, ignore = c.args
            flags |= int(ignore) << 9
            q1 = (x & 0xFFFFFFFF) | ((y & 0xFFFFFFFF) << 32)
            if data is not None:
                flags |= 1 << 10
                q2 = self.fp(data)
            else:
                q2 = idx & 0xFFFFFFFFFFFFFFFF
            q3 = (pal & 0xFFFFFFFF) | (((y * 256 + x) & 0xFFFFFFFF) << 32)
            if c.type in ("tileAtPosition", "spriteAtPosition") and not 0 <= y * 256 + x < 61440:
                self.warn(f"{c.type} {c.name} at {x},{y} is off screen; Mesen reads outside "
                          "the frame record (undefined), treated as false")
        q0 = flags | (op << 12) | ((mask & 0xFF) << 16) | (memo << 32)
        rec = struct.pack("<QQQQ", q0, q1, q2, q3)
        i = len(self.conds)
        self.conds.append(rec)
        self.cond_of[key] = i
        return i

    def clist(self, conds):
        """Condition list -> byte offset in the condition-list section (0 = empty)."""
        if not conds:
            return 0
        idx = []
        for c in conds:
            ci = self.cond_index(c)
            if ci not in idx:
                idx.append(ci)
        key = tuple(idx)
        if key not in self.clist_of:
            self.clist_of[key] = len(self.clists)
            for j, ci in enumerate(idx):
                self.clists += struct.pack("<I", ci | (0x80000000 if j == len(idx) - 1 else 0))
        return self.clist_of[key]

    def split(self, conds):
        frame = [c for c in conds if c.type in FRAME_CLASS]
        walk = sorted((c for c in conds if c.type not in FRAME_CLASS),
                      key=lambda c: WALK_ORDER[c.type])
        return frame, walk

    # -- blocks --------------------------------------------------------------------------
    def block(self, px16):
        """Deduplicated block index; NONE32 only when every byte is 0. A zero-alpha pixel can
        still carry colour (brightness above 255 lifts premultiplied RGB off 0), and Mesen adds
        that colour in, so alpha alone does not decide transparency."""
        if not px16.any():
            return NONE32
        b = px16.tobytes()
        i = self.block_of.get(b)
        if i is None:
            i = len(self.blocks)
            self.blocks.append(b)
            self.block_of[b] = i
        return i

    def tile_block(self, t):
        size = 8 * self.p.scale
        img = self.images.get(self.p.images[t.bitmap])
        px = adjust_brightness(tile_pixels(img, t.x, t.y, size), t.brightness)
        return self.block(to_2x(px, self.p.scale))

    # -- keys and rules ------------------------------------------------------------------
    def build_rules(self):
        p = self.p
        self.rules = bytearray(8)       # offset 0 = "no list" for the background level table
        self.key_entries = []          # (palette, body_lo, body_hi, kind, rule_list_offset)
        tile_block = {}
        longest = 0
        for key, idxs in p.tile_by_key.items():
            off = len(self.rules)
            self.rules += struct.pack("<II", len(idxs), 0)
            prefix = 0
            for ti in idxs:
                t = p.tiles[ti]
                if ti not in tile_block:
                    tile_block[ti] = self.tile_block(t)
                frame, walk = self.split(t.conditions)
                prefix |= int(t.force_disable_cache)
                flags = int(t.force_disable_cache) | (prefix << 1)
                self.rules += struct.pack("<IIII", tile_block[ti], self.clist(frame),
                                          self.clist(walk), (ti & 0xFFFFFF) | (flags << 24))
            longest = max(longest, len(idxs))
            if key[0] == "rom":
                _, index, pal = key
                self.key_entries.append((pal, index & 0xFFFFFFFF, 0, 0, off))
            else:
                _, pal, data = key
                f = self.fp(data)
                self.key_entries.append((pal, f & 0xFFFFFFFF, f >> 32, 1, off))
        self.longest = longest

    def build_keys(self):
        n = len(self.key_entries)
        nb = 1
        while nb * 4 < 2 * n or nb < 1:
            nb *= 2
        table = [[] for _ in range(nb)]
        max_probe = 0
        for pal, lo, hi, kind, off in self.key_entries:
            h = crc32(struct.pack("<III", pal, lo, hi)) & (nb - 1)
            for probe in range(nb):
                b = (h + probe) & (nb - 1)
                if len(table[b]) < 4:
                    table[b].append((pal, lo, hi, kind, off))
                    max_probe = max(max_probe, probe)
                    break
        self.nbuckets, self.max_probe = nb, max_probe
        out = bytearray(nb * 64)
        for b, ents in enumerate(table):
            for j, (pal, lo, hi, kind, off) in enumerate(ents):
                q0 = pal | (lo << 32)
                q1 = hi | ((off // 8) << 32) | (kind << 62) | (1 << 63)
                struct.pack_into("<QQ", out, b * 64 + j * 16, q0, q1)
        self.keys = out

    # -- backgrounds ---------------------------------------------------------------------
    def ratio(self, r):
        bits = struct.unpack("<I", struct.pack("<f", r))[0]
        if bits not in self.ratio_of:
            f = np.float32(r)
            tab = []
            for s in range(512):
                v = float(np.float32(s) * f)
                tab.append(max(-0x80000000, min(0x7FFFFFFF, int(v))))   # trunc toward zero
            self.ratio_of[bits] = len(self.ratio_tabs)
            self.ratio_tabs.append(struct.pack("<512i", *tab))
        return self.ratio_of[bits]

    def bg_map(self, name, brightness):
        key = (name.lower(), brightness)
        if key in self.map_of:
            return self.map_of[key]
        s = self.p.scale
        img = self.images.get(name)
        if img is None:
            res = (0, 0, 0)
        else:
            h, w = img.shape[:2]
            wn, hn = w // s, h // s
            cw, ch = (wn + 7) // 8, (hn + 7) // 8
            off = len(self.bgmaps)
            cells = []
            band = 8                             # cell rows per pass: bounds the int64 temporaries
            for cy0 in range(0, ch, band):
                rows = min(band, ch - cy0)
                y0, y1 = cy0 * 8 * s, min((cy0 + rows) * 8 * s, hn * s)
                pad = np.zeros((rows * 8 * s, cw * 8 * s, 4), dtype=np.uint8)
                pad[:y1 - y0, :wn * s] = adjust_brightness(img[y0:y1, :wn * s], brightness)
                big = to_2x(pad, s)              # [rows*16, cw*16, 4]
                cells += [self.block(np.ascontiguousarray(big[r * 16:r * 16 + 16, cx * 16:cx * 16 + 16]))
                          for r in range(rows) for cx in range(cw)]
            self.bgmaps += struct.pack(f"<{len(cells)}I", *cells)
            res = (off, wn, hn)
        self.map_of[key] = res
        return res

    def build_backgrounds(self):
        self.ratio_of, self.ratio_tabs = {}, []
        self.map_of = {}
        self.bgmaps = bytearray(4)
        self.bgrecs = bytearray()
        self.bglevels = bytearray(40 * 8)
        self.bgindex = bytearray(4)
        nrec = 0
        # Build every map grouped by file first: in level order Metroid HD's three 1 GB maps
        # interleave and evict each other from the 1 GB image cache, re-decoding each time.
        keys = {(bg.file, bg.brightness) for entries in self.p.backgrounds for bg in entries}
        for name, bright in sorted(keys, key=lambda k: (k[0].lower(), k[1])):
            self.bg_map(name, bright)
        for lvl, entries in enumerate(self.p.backgrounds):
            if not entries:
                continue
            indexed = len(entries) >= BG_INDEX_MIN
            rules = []
            sigs = {}
            for ei, bg in enumerate(entries):
                mp, wn, hn = self.bg_map(bg.file, bg.brightness)
                self.bgrecs += struct.pack("<IIIIIHHB7x", mp, wn, hn, bg.left, bg.top,
                                           self.ratio(bg.hratio), self.ratio(bg.vratio),
                                           BLENDS[bg.blend])
                conds = bg.conditions
                if indexed:
                    eq, rest = {}, []
                    possible = True
                    for c in conds:
                        if (c.type == "memoryCheckConstant" and not c.negate and c.args[1] == "eq"
                                and len(eq) < SIG_MAX_TERMS):
                            a, _, v, mask = c.args
                            k = (self.slot[a], mask & 0xFF)
                            if k in eq and eq[k] != (v & 0xFF):
                                possible = False
                            eq[k] = v & 0xFF
                        else:
                            rest.append(c)
                    if possible:
                        sig = tuple(sorted(eq))
                        vals = tuple(eq[k] for k in sig)
                        sigs.setdefault(sig, {}).setdefault(vals, []).append(ei)
                    conds = rest
                rules.append(struct.pack("<IIII", nrec, self.clist(conds), 0, ei & 0xFFFFFF))
                nrec += 1
            roff = len(self.rules)
            self.rules += struct.pack("<II", len(rules), 0) + b"".join(rules)
            ioff = self.build_bg_index(sigs) if indexed else 0
            struct.pack_into("<II", self.bglevels, lvl * 8, roff, ioff)
        self.n_bgrecs = nrec

    def build_bg_index(self, sigs):
        """Value index for one level; returns its offset inside the bg-index section."""
        recs = []
        for sig, table in sigs.items():
            nb = 1
            while nb < 2 * len(table):
                nb *= 2
            slots = [None] * nb
            maxp = 0
            for vals, ents in table.items():
                key = bytes(vals) + bytes(16 - len(vals))
                h = crc32(key) & (nb - 1)
                for probe in range(nb):
                    b = (h + probe) & (nb - 1)
                    if slots[b] is None:
                        lst = len(self.bgindex)
                        self.bgindex += struct.pack(f"<{len(ents)}I", *ents)
                        slots[b] = (key, lst, len(ents))
                        maxp = max(maxp, probe)
                        break
            recs.append((sig, slots, nb, maxp))
        base = len(self.bgindex)                 # header: u32 count + 48 bytes per signature
        self.bgindex += bytearray(4 + len(recs) * 48)
        pos = base + 4
        struct.pack_into("<I", self.bgindex, base, len(recs))
        for sig, slots, nb, maxp in recs:
            self.bgindex += bytes(-len(self.bgindex) % 32)    # tables start on 32 bytes
            toff = len(self.bgindex)
            for s in slots:
                if s is None:
                    self.bgindex += bytes(32)
                else:
                    key, lst, cnt = s
                    self.bgindex += key + struct.pack("<IIII", lst, cnt, 1, 0)   # 32 bytes
            assert len(self.bgindex) - toff == 32 * nb, "bg index entry size"
            pairs = b"".join(struct.pack("<BB", sl, mk) for sl, mk in sig)
            pairs += bytes(32 - len(pairs))
            struct.pack_into("<BB", self.bgindex, pos, len(sig), 0)
            self.bgindex[pos + 2:pos + 34] = pairs
            struct.pack_into("<III", self.bgindex, pos + 34, toff, nb, maxp)
            pos += 48
        return base

    # -- fallbacks, additions ------------------------------------------------------------
    def game_type(self):
        """Is the game CHR-RAM? From the ROM when given, else from the pack when every tile and
        addition key is one kind. None = unknown. Mesen asks the mapper (HasChrRom/HasChrRam)."""
        if self.rom is not None:
            return len(self.rom) == 0
        kinds = {k[0] for k in self.p.tile_by_key}
        kinds |= {"ram" if a.added.chr_ram else "rom" for a in self.p.additions}
        return {frozenset({"ram"}): True, frozenset({"rom"}): False}.get(frozenset(kinds))

    def build_fallback(self):
        fb = {}
        if self.chr_ram_game:
            # HdNesPack::InitializeFallbackTiles: the whole table, explicit <fallback> lines
            # included, is built only when the mapper has CHR-ROM.
            if self.p.fallbacks:
                self.warn(f"{len(self.p.fallbacks)} <fallback> lines ignored (CHR-RAM game, as in Mesen)")
            self.fallback, self.n_fallback, self.fb_arr = b"", 0, []
            return
        if self.p.options & M.OPT_AUTOMATIC_FALLBACK_TILES:
            chr_rom = self.rom
            n = len(chr_rom) // 16
            used = {}
            prev = bytes(16)        # HdNesPack reuses one key; it starts zeroed
            for t in self.p.tiles:
                i = t.key.index
                # BaseMapper::CopyChrTile(TileIndex * 16): 32-bit address, and an address
                # past the ROM copies NOTHING, so the key keeps the previous tile's data.
                # That data is already in the set (first insertion wins), so such a tile adds
                # nothing; a leading run adds the all-zero key, which the scan below skips.
                addr = (i * 16) & 0xFFFFFFFF
                if addr <= len(chr_rom) - 16:
                    prev = bytes(chr_rom[addr:addr + 16])
                else:
                    self.warn("tile index past CHR-ROM in automatic fallback: no entry "
                              "(Mesen keeps the previous tile's data, already in the set)")
                used.setdefault(prev, i)
            for i in range(n):
                d = chr_rom[i * 16:i * 16 + 16]
                if not any(d):
                    continue
                j = used.get(bytes(d))
                if j is not None and j != i:
                    fb[i] = j
        for i, f in self.p.fallbacks:
            fb[i] = f
        if any(i < 0 for i in fb):
            self.warn("negative fallback tile index ignored")
            fb = {i: f for i, f in fb.items() if i >= 0}
        n = max(fb) + 1 if fb else 0
        if n > 1 << 22:
            raise Refused(f"fallback tile index {n - 1} too large")
        arr = [-1] * n
        for i, f in fb.items():
            arr[i] = f
        self.fallback = struct.pack(f"<{n}i", *arr)
        self.n_fallback = n
        self.fb_arr = arr

    def mark_rewrite_sensitive(self):
        """Condition bit 24: a fallback REWRITE can change this tile condition's result.

        GetMatchingTile rewrites a looked-up record's TileIndex to its fallback when the exact
        key is missing and the fallback key (same palette, or the default key) exists (spec
        3.16); a tile condition read after that sees the rewritten index. tmatch(rec, T) is
        `idx == T or fallback(idx) == T`, so a rewrite (idx -> f) can change the answer only
        when the condition names T and (a) T itself is rewritable, or (b) some rewritable i has
        fallback(fallback(i)) == T, and the rewrite actually changes the test. "Rewritable" is
        checked for the condition's palette, or for IGNOREPALETTE for any palette a key in the
        pack uses. The hardware walks keys whose lists hold a
        flagged condition one pixel at a time and tracks lookup order (HDPACK_DESIGN.md, "one
        walk per tile instance"). hdp3_read re-derives the flag by brute force."""
        arr, keys = self.fb_arr, self.p.tile_by_key
        if not arr:
            return

        def fbk(i):
            return arr[i] if 0 <= i < len(arr) else -1

        def has(pal, i):
            return ("rom", M.to_int32(i), pal) in keys

        def rewritable(pal, i):
            f = fbk(i)
            return f >= 0 and not has(pal, i) and (has(pal, f) or has(0xFFFFFFFF, f))

        def tmatch(i, t):
            return i == t or fbk(i) == t

        # a rewrite i -> fallback(i) changes tmatch(., T) only for i == T or
        # fallback(fallback(i)) == T (i with fallback(i) == T matches before and after)
        hop2 = {}
        for i in range(len(arr)):
            if fbk(fbk(i)) >= 0:
                hop2.setdefault(fbk(fbk(i)), []).append(i)
        pals = {k[2] for k in keys if k[0] == "rom"}
        tile_types = {COND_TYPES[t] for t in ("tileAtPosition", "spriteAtPosition",
                                              "tileNearby", "spriteNearby")}
        self.n_sensitive = 0
        n_frame = 0
        memo = {}
        for ci, rec in enumerate(self.conds):
            q0, q1, q2, q3 = struct.unpack("<QQQQ", rec)
            if (q0 & 0xFF) not in tile_types or (q0 >> 10) & 1:
                continue
            ignore, pal = bool((q0 >> 9) & 1), q3 & 0xFFFFFFFF
            T = q2 - (1 << 64) if q2 >> 63 else q2
            mk = (T, pal, ignore)
            if mk not in memo:
                cands = [i for i in [T] + hop2.get(T, [])
                         if fbk(i) >= 0 and tmatch(i, T) != tmatch(fbk(i), T)]
                memo[mk] = any(any(rewritable(P, i) for P in pals) if ignore
                               else rewritable(pal, i) for i in cands)
            if memo[mk]:
                self.conds[ci] = struct.pack("<QQQQ", q0 | (1 << 24), q1, q2, q3)
                self.n_sensitive += 1
                n_frame += (q0 >> 11) & 1
        if self.n_sensitive:
            # frame-class ones (tileAtPosition, spriteAtPosition) are memoised at FIRST USE
            # in Mesen, so their result depends on what was looked up before that point
            self.warn(f"{self.n_sensitive} tile conditions a fallback rewrite can change "
                      f"(flagged, walked in lookup order; {n_frame} of them frame-class)")

    def key_fields(self, k):
        if k.chr_ram:
            f = self.fp(k.data)
            return 1, k.palette, f
        return 0, k.palette, k.index & 0xFFFFFFFFFFFFFFFF

    def build_additions(self):
        out = bytearray()
        # HdNesPack::CleanupInvalidRules deletes, at load, additions whose added tile kind is not
        # mapper->HasChrRam(). That is NOT the opposite of HasChrRom(): a cart can have both
        # (Metroid patched by a034's mmm.ips: NES 2.0 header, 128 KB CHR-ROM + 8 KB CHR-RAM),
        # and then every index-keyed addition is dropped although the tiles are index-keyed.
        # Without a ROM, a CHR-RAM game has CHR-RAM and a CHR-ROM game is assumed to have none.
        ram = self.has_chr_ram if self.has_chr_ram is not None else bool(self.chr_ram_game)
        keep = [a for a in self.p.additions if a.added.chr_ram == ram]
        if len(keep) != len(self.p.additions):
            self.warn(f"{len(self.p.additions) - len(keep)} additions dropped (wrong CHR kind "
                      "for this game, as in Mesen)")
        self.kept_additions = keep
        for a in keep:
            ok, opal, obody = self.key_fields(a.original)
            ak, apal, abody = self.key_fields(a.added)
            out += struct.pack("<IIQIIQiiI20x", opal, ok, obody, apal, ak, abody,
                               a.ox, a.oy, int(a.ignore_palette))
        self.additions = out

    def build_patches(self):
        """Section 14: each <patch> compiled to the run list the core builds the patched ROM
        from (hdp3_patch.py). An entry is keyed by a SHA1; the core hashes the ROM file and then
        its iNES 1.0 form (rommatch.py) and takes the first entry either hash names. An MD5 line
        becomes one entry per library ROM form with that MD5 (--romindex; without it the line is
        dropped, as Mesen drops it). A patch that cannot be parsed is left out, so that ROM loads
        unpatched - what Mesen does when its patcher refuses."""
        import hdp3_patch
        import rommatch
        import rompatch
        keyed = rommatch.section_keys(self.p.patches, self.p.patches_md5, self.romindex)
        for md5, name in sorted(self.p.patches_md5.items()):
            if self.romindex is None:
                self.warn(f"<patch> {name}: MD5 line needs --romindex to reach the core; dropped")
                continue
            shas = rommatch.resolve_md5(md5, self.romindex)
            if not shas:
                self.warn(f"<patch> {name}: MD5 {md5} is no ROM in the index; dropped")
            for sha in shas:
                if keyed.get(sha) == name and sha not in self.p.patches:
                    self.warn(f"<patch> {name}: MD5 line keyed as SHA1 {sha} (beyond Mesen)")
        ents = []
        for sha, name in sorted(keyed.items()):
            try:
                ents.append((sha.lower(), hdp3_patch.compile_patch(self.p.read(name))))
            except (rompatch.PatchError, FileNotFoundError) as e:
                self.warn(f"<patch> {name}: {e}; that ROM will load unpatched")
        for line, what in self.p.compat:
            self.warn(f"line {line}: {what}")
        self.patch_section = hdp3_patch.section(ents) if ents else b""

    # -- audio (stage 6) -----------------------------------------------------------------
    def build_audio(self):
        """Section 15 (HDPACK_DESIGN.md "Stage 6 - audio"): each <bgm>/<sfx> line decoded with
        Mesen's own stb_vorbis + Hermite resampler at 48 kHz and IMA-ADPCM encoded by
        tools/audio/oggdecode.exe. Lines naming the same file share its stream. A line whose
        file does not decode stays as an entry with 0 frames, so the hardware gives Mesen's
        answer for it (a BGM play is refused, an SFX play sets the error bit)."""
        p = self.p
        self.audio_section = b""
        if not (p.bgm or p.sfx):
            return
        import subprocess
        import tempfile
        here = os.path.dirname(os.path.abspath(__file__))
        dec = os.path.join(here, "audio", "oggdecode.exe")
        if not os.path.isfile(dec):
            raise Refused("pack has <bgm>/<sfx>: build tools/audio/oggdecode.exe "
                          "(sh tools/audio/build.sh)")
        sys.path.insert(0, os.path.join(here, "audio"))
        import hdaudio
        jobs = [(False, i, f, lp) for i, (f, lp) in p.bgm.items()]
        jobs += [(True, i, f, 0) for i, f in p.sfx.items()]
        streams = {}        # file -> bytes (shared by every line naming it)
        done = {}           # (file, loop position) -> entry fields, or None (does not decode)
        lines = []
        with tempfile.TemporaryDirectory() as tmp:
            src, out = os.path.join(tmp, "in.ogg"), os.path.join(tmp, "out.ima")
            for is_sfx, tid, fname, loop in sorted(jobs, key=lambda j: (j[2], j[3], j[0], j[1])):
                if (fname, loop) not in done:
                    with open(src, "wb") as f:
                        f.write(p.read(fname))
                    r = subprocess.run([dec, "adpcm", src, out, str(hdaudio.RATE), str(loop)],
                                       capture_output=True, text=True)
                    if r.returncode:
                        self.warn(f"audio {fname}: does not decode ({r.stderr.strip()}); kept "
                                  f"as an entry that cannot play, as Mesen fails to open it")
                        done[(fname, loop)] = None
                    else:
                        kv = dict(x.split("=", 1) for x in r.stdout.split())
                        if fname not in streams:
                            with open(out, "rb") as f:
                                streams[fname] = f.read()
                        with open(out + ".ck", "rb") as f:
                            ckb = f.read()
                        cks = list(struct.unpack("<%dQ" % (len(ckb) // 8), ckb))
                        done[(fname, loop)] = (int(kv["frames"]), int(kv["loopframe"]),
                                               int(kv["state0"], 16), int(kv["statel"], 16),
                                               cks)
                d = done[(fname, loop)]
                if d is None:
                    lines.append((is_sfx, tid, b"", 0, 0, 0, 0, [0]))
                else:
                    lines.append((is_sfx, tid, streams[fname]) + d)
        try:
            self.audio_section = hdaudio.build_section(lines)
        except ValueError as e:
            raise Refused(f"audio: {e}")

    # -- output --------------------------------------------------------------------------
    def build(self):
        p = self.p
        self.chr_ram_game = self.game_type()           # decided before the slow part
        needs_type = bool(p.additions or p.fallbacks
                          or p.options & M.OPT_AUTOMATIC_FALLBACK_TILES)
        if self.chr_ram_game is None and needs_type:
            raise Refused("pack mixes CHR-RAM and CHR-ROM keys; give the ROM with --rom")
        if (self.chr_ram_game is False and self.rom is None
                and p.options & M.OPT_AUTOMATIC_FALLBACK_TILES):
            raise Refused("CHR-ROM game with automaticFallbackTiles; give the ROM with --rom")
        self.build_watched()
        self.build_rules()
        self.build_keys()
        self.build_backgrounds()
        self.build_fallback()
        self.mark_rewrite_sensitive()
        self.build_additions()
        self.build_patches()
        self.build_audio()

        sections = [b""] * N_SECTIONS
        sections[S_KEYS] = bytes(self.keys)
        sections[S_RULES] = bytes(self.rules)
        sections[S_CONDS] = b"".join(self.conds)
        sections[S_CLISTS] = bytes(self.clists)
        sections[S_BLOCKS] = b"".join(self.blocks)
        sections[S_BGLEVELS] = bytes(self.bglevels)
        sections[S_BGRECS] = bytes(self.bgrecs)
        sections[S_BGMAPS] = bytes(self.bgmaps)
        sections[S_RATIOS] = b"".join(self.ratio_tabs)
        sections[S_WATCHED] = struct.pack(f"<{len(self.watched)}I", *self.watched)
        sections[S_FALLBACK] = self.fallback
        sections[S_ADDITIONS] = bytes(self.additions)
        if p.palette:
            sections[S_PALETTE] = struct.pack("<64I", *(c & 0xFFFFFF for c in p.palette))
        sections[S_BGINDEX] = bytes(self.bgindex)
        sections[S_PATCHES] = self.patch_section
        # the addition tables hd3_dict looks records up in (8a); imported here because
        # hdp3_addtab imports this module
        import hdp3_addtab
        add_bytes = sections[S_ADDITIONS]
        msg = hdp3_addtab.chain(add_bytes, len(self.kept_additions), self.fb_arr)
        if msg:
            raise Refused(msg)
        (sections[S_ADDKEYS], add_nb, add_mp,
         sections[S_ADDLISTS]) = hdp3_addtab.build(add_bytes, len(self.kept_additions))

        sections[S_AUDIO] = self.audio_section

        # Audio is laid out last: the file loads verbatim into the 192 MB pack window and any
        # bytes past it go to the overflow region (HDPACK_DESIGN.md "Stage 6 - audio"), so only
        # the audio may cross 192 MB.
        out = bytearray(HEADER_SIZE)
        table = [(0, 0)] * N_SECTIONS
        for i in [k for k in range(N_SECTIONS) if k != S_AUDIO] + [S_AUDIO]:
            s = sections[i]
            if i == S_AUDIO and not s:
                continue        # no trailing pad: a pack without audio is byte-identical to before
            if len(out) % ALIGN:
                out += bytes(ALIGN - len(out) % ALIGN)
            # the audio header + entry table are read with unaligned bursts, so they must not
            # cross into the overflow region; only the 64-byte-aligned stream data may
            if i == S_AUDIO and len(out) + struct.unpack_from("<I", s, 8)[0] > PACK_WINDOW:
                raise Refused(f"images and tables end at {len(out)} bytes: no room for the "
                              f"audio entry table inside the {PACK_WINDOW >> 20} MB pack window")
            table[i] = (len(out) if s else 0, len(s))
            out += s
        if len(out) > PACK_WINDOW + OVERFLOW:
            raise Refused(f"pack is {len(out)} bytes, more than the "
                          f"{(PACK_WINDOW + OVERFLOW) >> 20} MB of pack window + overflow")
        flags = (int(p.version < 100) | (int(p.palette is not None) << 1)
                 | (int(any(k[0] == "ram" for k in p.tile_by_key)) << 2)
                 | (int(any(k[0] == "rom" for k in p.tile_by_key)) << 3)
                 | (int(bool(self.chr_ram_game)) << 4)
                 | (int(self.chr_ram_game is not None) << 5))
        ov = p.overscan if p.overscan else (-1, -1, -1, -1)
        struct.pack_into("<4sHBBII4iIIIIIIIIIII", out, 0, b"NHDP", FORMAT_VERSION, p.scale,
                         flags, p.version, p.options, *ov, len(out), self.nbuckets,
                         self.max_probe, len(self.conds), self.n_frame, len(self.watched),
                         self.longest, len(self.blocks), len(self.kept_additions), self.n_fallback,
                         len(self.ratio_tabs))
        for i, (o, n) in enumerate(table):
            struct.pack_into("<II", out, 0x80 + i * 8, o, n)
        struct.pack_into("<II", out, 0x4C, add_nb, add_mp)     # section 16's buckets, max probe
        return bytes(out)


def load_pack(folder, bases=()):
    """Load a pack, optionally laid over base folders. A file in `folder` hides a file of the
    same pack-relative name in a base (earlier bases hide later ones): the result is what a
    player gets by copying a variant's files over the base pack, which is how variant and
    add-on folders (music patches, alternate maps, hack add-ons) are installed."""
    if not bases:
        return M.load(folder)
    where = {}
    for d in (folder, *bases):
        for dp, _dn, fn in os.walk(d):
            for f in fn:
                rel = os.path.relpath(os.path.join(dp, f), d)
                k = M.norm_name(rel)
                if k is not None and k not in where:
                    where[k] = (rel, os.path.join(dp, f))
    paths = {rel: full for rel, full in where.values()}

    def reader(rel):
        with open(paths[rel], "rb") as fh:
            return fh.read()

    return M.load(folder, list(paths), reader)


def patch_rom(d, pack, notes=None):
    """Apply the pack's <patch> for this ROM file the way the core does (rommatch.py): Mesen's
    whole-file SHA1 first, then the file with an iNES 1.0 header (bytes 7-15 zero), and MD5
    lines as well as SHA1 ones. The last two go beyond Mesen and are reported."""
    import rompatch
    import rommatch
    if not (pack.patches or pack.patches_md5):
        return d
    hit = rommatch.find(d, pack.patches, pack.patches_md5)
    if hit:
        cand, name, label = hit
        try:
            out = rompatch.apply(cand, pack.read(name))
        except (rompatch.PatchError, FileNotFoundError) as e:
            raise Refused(f"<patch> {name}: {e}")
        if notes is not None:
            notes.append(f"applied <patch> {name}{label}")
        return out
    if notes is not None:
        notes.append("ROM matches no <patch> hash; using it unpatched")
    return d


def rom_has_chr_ram(d, chr_rom_size, notes=None):
    """BaseMapper::HasChrRam() (_chrRamSize > 0) for an iNES file, as Mesen 2.1.1 decides it.

    NES 2.0 header: the header's CHR-RAM size (byte 11 low nibble, 64 << n, 0 = none) is used
    as is; Mesen consults no database for NES 2.0. No CHR-ROM and no size given: Mesen assumes
    CHR-RAM (mapper default or 8 KB). iNES 1.0 with CHR-ROM: Mesen takes the size from its game
    database or the mapper's default, which this tool cannot see; assumed none, and said so.
    Only the <addition> cleanup uses this; tile keys and fallbacks follow HasChrRom()."""
    if d[7] & 0x0C == 0x08:
        n = d[11] & 0x0F
        return n != 0
    if chr_rom_size == 0:
        return True
    if notes is not None:
        notes.append("iNES 1.0 header with CHR-ROM: assumed no CHR-RAM (Mesen may add some "
                     "from its game database or the mapper); only affects <addition> lines")
    return False


def read_rom(path, pack=None, notes=None):
    """(CHR-ROM bytes, HasChrRam) of the ROM after the pack's patch."""
    with open(path, "rb") as f:
        d = f.read()
    if pack is not None:
        d = patch_rom(d, pack, notes)
    chr_rom = _chr_of(d, path)
    return chr_rom, rom_has_chr_ram(d, len(chr_rom), notes)


def read_rom_chr(path, pack=None, notes=None):
    return read_rom(path, pack, notes)[0]


def _chr_of(d, path):
    if d[:4] != b"NES\x1a":
        raise Refused(f"{path} is not an iNES file")
    prg, chr_ = d[4] * 16384, d[5] * 8192
    if d[7] & 0x0C == 0x08:                      # NES 2.0: size MSBs in byte 9
        if d[9] >> 4 == 0xF or d[9] & 0xF == 0xF:
            raise Refused(f"{path}: NES 2.0 exponent-form ROM sizes not supported")
        prg = ((d[9] & 0xF) << 8 | d[4]) * 16384
        chr_ = ((d[9] >> 4) << 8 | d[5]) * 8192
    off = 16 + (512 if d[6] & 4 else 0)
    if len(d) < off + prg + chr_:
        raise Refused(f"{path} is shorter than its header says")
    return d[off + prg:off + prg + chr_]


DEFAULT_ROMINDEX = os.path.join(os.path.dirname(os.path.abspath(__file__)), "corpus", "romindex.tsv")


def convert(pack, rom_chr=None, has_chr_ram=None, romindex=None):
    c = Converter(pack, rom_chr, has_chr_ram, romindex)
    return c.build(), c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pack_dir")
    ap.add_argument("out")
    ap.add_argument("--rom")
    ap.add_argument("--base", action="append", default=[],
                    help="base pack folder this variant is copied over (repeatable)")
    ap.add_argument("--romindex", default=DEFAULT_ROMINDEX,
                    help="romindex.py table used to key MD5 <patch> lines by SHA1 "
                         "(default tools/corpus/romindex.tsv; 'none' to drop MD5 lines as Mesen does)")
    a = ap.parse_args()
    try:
        pack = load_pack(a.pack_dir, a.base)
        notes = []
        rom_chr, has_chr_ram = read_rom(a.rom, pack, notes) if a.rom else (None, None)
        index = None
        if a.romindex != "none" and os.path.isfile(a.romindex):
            import rommatch
            index = rommatch.load_index(a.romindex)
        data, c = convert(pack, rom_chr, has_chr_ram, index)
        for n in notes:
            print("  rom:", n)
    except (M.PackRejected, Refused) as e:
        raise SystemExit(f"refused: {e}")
    with open(a.out, "wb") as f:
        f.write(data)
    print(f"{a.out}: {len(data):,} bytes, {len(pack.tile_by_key)} keys in {c.nbuckets} buckets "
          f"(max probe {c.max_probe}), {len(pack.tiles)} rules, {len(c.blocks)} blocks, "
          f"{len(c.conds)} conditions ({c.n_frame} frame), {len(c.watched)} watched, "
          f"{c.n_bgrecs} backgrounds")
    for line, msg in pack.errors[:10]:
        print(f"  pack line {line}: {msg}")
    for msg, n in sorted(c.warnings.items()):
        print(f"  warning ({n}x): {msg}")


if __name__ == "__main__":
    main()
