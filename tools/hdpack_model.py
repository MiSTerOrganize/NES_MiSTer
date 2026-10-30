"""Mesen 2 HD pack loader, ported line for line from Core/NES/HdPacks/HdPackLoader.cpp (2.1.1).

This is the single parser shared by the format-3 converter and the reference renderer, so
both agree with Mesen by construction. Citations "LDR:n" are lines of HdPackLoader.cpp,
"DATA:n" of HdData.h (see MESEN_HD_RUNTIME_SPEC.md).

Error semantics copied from the C++:
  checkConstraint(x)    false -> log an error and drop THIS tag only           (LDR:18)
  checkConstraintEx(x)  version >= 109: as checkConstraint; else only log       (LDR:19)
  std::stoi / stof / stoul throwing, or a missing <img> file -> the WHOLE pack fails to
  load (LDR:164-166, LDR:215-218). That is PackRejected here.
A token index past the end of a tag is undefined behaviour in Mesen (vector operator[]);
it is treated as PackRejected and named, never guessed.

usage (inspection):  python hdpack_model.py <pack_dir>
"""
import math
import os
import re
import struct
import sys
from collections import defaultdict

CURRENT_VERSION = 109          # BaseHdNesPack::CurrentVersion
NO_TILE = -1
PPU_MEMORY_MARKER = 0x80000000

OPT_NO_SPRITE_LIMIT = 1
OPT_ALTERNATE_REGISTER_RANGE = 2
OPT_DISABLE_CACHE = 8
OPT_DONT_RENDER_ORIGINAL_TILES = 16
OPT_AUTOMATIC_FALLBACK_TILES = 32

OPS = {"==": "eq", "!=": "ne", ">": "gt", "<": "lt", "<=": "le", ">=": "ge"}
BLEND_MODES = ("Alpha", "Add", "Subtract")


class PackRejected(Exception):
    """Mesen would refuse to load this pack at all."""


class _TagDropped(Exception):
    """checkConstraint failed: this tag is ignored (error already logged)."""


# ------------------------------------------------------------------------------------------
# C++ number parsing
# ------------------------------------------------------------------------------------------
_WS = " \t\n\v\f\r"
_INT = re.compile(r"[ \t\n\v\f\r]*([+-]?\d+)")
_FLT = re.compile(r"[ \t\n\v\f\r]*([+-]?(?:\d+\.?\d*(?:[eE][+-]?\d+)?|\.\d+(?:[eE][+-]?\d+)?"
                  r"|0[xX][0-9a-fA-F]*\.?[0-9a-fA-F]*(?:[pP][+-]?\d+)?|inf(?:inity)?|nan))",
                  re.IGNORECASE)


def stoi(text):
    m = _INT.match(text)
    if not m:
        raise PackRejected(f"std::stoi: invalid argument {text!r}")
    v = int(m.group(1))
    if not -(1 << 31) <= v < (1 << 31):
        raise PackRejected(f"std::stoi: out of range {text!r}")
    return v


def stoul(text):
    m = _INT.match(text)
    if not m:
        raise PackRejected(f"std::stoul: invalid argument {text!r}")
    v = int(m.group(1))
    if v < 0:                              # strtoul negates in unsigned long (32-bit on MSVC)
        v &= 0xFFFFFFFF
    if v >= 1 << 32:
        raise PackRejected(f"std::stoul: out of range {text!r}")
    return v


def stof(text):
    """std::stof -> single-precision float."""
    m = _FLT.match(text)
    if not m:
        raise PackRejected(f"std::stof: invalid argument {text!r}")
    s = m.group(1)
    try:
        v = float.fromhex(s) if "x" in s.lower() else float(s)
    except ValueError:
        raise PackRejected(f"std::stof: invalid argument {text!r}")
    if math.isfinite(v) and abs(v) > 3.4028234663852886e38:
        raise PackRejected(f"std::stof: out of range {text!r}")
    return struct.unpack("<f", struct.pack("<f", v))[0]


def from_hex(text):
    """HexUtilities::FromHex: every character shifts in a nibble (non-hex as 0), in an int."""
    v = 0
    for ch in text:
        v = (v << 4) & 0xFFFFFFFF
        if "0" <= ch <= "9":
            v |= ord(ch) - 48
        elif "A" <= ch <= "F":
            v |= ord(ch) - 55
        elif "a" <= ch <= "f":
            v |= ord(ch) - 87
    return v


def to_int32(v):
    v &= 0xFFFFFFFF
    return v - (1 << 32) if v & 0x80000000 else v


def cpp_int(f):
    """(int)float: truncation toward zero (UB outside int range; clamp)."""
    if not math.isfinite(f):
        return -(1 << 31)
    return max(-(1 << 31), min((1 << 31) - 1, int(f)))


def split(text, delim=","):
    """StringUtilities::Split: every delimiter splits, empty tokens kept."""
    return text.split(delim)


# ------------------------------------------------------------------------------------------
# Data model
# ------------------------------------------------------------------------------------------
class TileKey:
    """HdTileKey (DATA:12-74). CHR-ROM key = (TileIndex, PaletteColors);
    CHR-RAM key = (PaletteColors, TileData[16])."""
    __slots__ = ("palette", "data", "index", "chr_ram")

    def __init__(self, palette=0, data=bytes(16), index=0, chr_ram=False):
        self.palette = palette & 0xFFFFFFFF
        self.data = bytes(data)
        self.index = index
        self.chr_ram = chr_ram

    def key(self, default=False):
        pal = 0xFFFFFFFF if default else self.palette
        if self.chr_ram:
            return ("ram", pal, self.data)
        return ("rom", to_int32(self.index), pal)

    def is_sprite(self):
        return (self.palette & 0xFF000000) == 0xFF000000


class Condition:
    """One HdPackCondition object. `negate` is True for the '!' twin (DATA:193-195)."""
    __slots__ = ("name", "type", "negate", "args")

    def __init__(self, name, ctype, negate, args):
        self.name, self.type, self.negate, self.args = name, ctype, negate, args

    # Conditions whose result Mesen caches once per frame (_useCache = true).
    PER_FRAME = ("memoryCheck", "memoryCheckConstant", "frameRange", "tileAtPosition",
                 "spriteAtPosition")


class TileDef:
    __slots__ = ("key", "bitmap", "x", "y", "brightness", "default", "conditions",
                 "force_disable_cache", "chr_bank", "line")


class Background:
    __slots__ = ("file", "brightness", "hratio", "vratio", "priority", "left", "top",
                 "blend", "conditions", "line")


class Addition:
    __slots__ = ("original", "added", "ox", "oy", "ignore_palette")


def norm_name(name):
    """Pack-relative name -> index key. Mesen combines the pack folder and the name and hands it
    to the OS; the packs are authored on Windows, where '/' and '\\' are equivalent and names are
    case-insensitive, so the index is too. A name that climbs out of the pack folder returns None
    (refused, never followed: a pack is untrusted input)."""
    parts = []
    for seg in name.replace("\\", "/").split("/"):
        if seg in ("", "."):
            continue
        if seg == "..":
            if not parts:
                return None
            parts.pop()
            continue
        parts.append(seg)
    if not parts or ":" in parts[0]:
        return None
    return "/".join(parts).lower()


class HdPack:
    def __init__(self, folder, files=None, reader=None):
        """folder: the pack directory (hires.txt lives here).
        files:  optional iterable of pack-relative names, for a pack still inside an archive.
        reader: optional callable(relative_name) -> bytes, used with `files`."""
        self.folder = folder
        self._reader = reader
        self._index = None
        if files is not None:
            self._index = {}
            for f in files:
                k = norm_name(f)
                if k is not None:
                    self._index.setdefault(k, f)
        self.version = 0
        self.scale = 1
        self.options = 0
        self.overscan = None              # (top, right, bottom, left) or None
        self.images = []                  # <img> file names, index = BitmapIndex
        self.tiles = []                   # TileDef, file order
        self.conditions = {}              # name -> Condition (latest definition wins, LDR:588)
        self.backgrounds = [[] for _ in range(40)]   # BackgroundsByPriority, file order
        self.background_files = []        # distinct PNG names, first-use order
        self.additions = []
        self.fallbacks = []               # (tile, fallback) file order
        self.watched = set()              # WatchedMemoryAddresses
        self.patches = {}                 # SHA1 (upper) -> file
        # Beyond Mesen (rommatch.py): <patch> lines Mesen rejects but the core can still use -
        # an MD5 where Mesen wants a SHA1, and a file name corrected by pack_fixes.py. They are
        # NOT in self.patches (that stays exactly Mesen's set) and each is listed in self.compat.
        self.patches_md5 = {}             # MD5 (upper) -> file
        self.compat = []                  # (line, what was done beyond Mesen)
        self.bgm = {}                     # id -> (file, loop_position)
        self.sfx = {}                     # id -> file
        self.palette = None               # 64 x 0xFFRRGGBB from palette.dat, else None
        self.tile_by_key = {}             # key -> [tile indices], file order (LDR:846-863)
        self.errors = []                  # (line, message), as MessageManager would log

    def _build_index(self):
        self._index = {}
        for dp, _dn, fn in os.walk(self.folder):
            for f in fn:
                rel = os.path.relpath(os.path.join(dp, f), self.folder)
                self._index.setdefault(norm_name(rel), rel)

    def resolve(self, name):
        """Actual pack-relative name for `name`, or None if the pack has no such file."""
        if self._index is None:
            self._build_index()
        k = norm_name(name)
        return None if k is None else self._index.get(k)

    def has(self, name):
        return self.resolve(name) is not None

    def read(self, name):
        rel = self.resolve(name)
        if rel is None:
            raise FileNotFoundError(name)
        if self._reader is not None:
            return self._reader(rel)
        with open(os.path.join(self.folder, rel), "rb") as f:
            return f.read()

    def path(self, name):
        """Filesystem path of a pack file (folder packs only)."""
        rel = self.resolve(name)
        return None if rel is None else os.path.join(self.folder, rel)


# ------------------------------------------------------------------------------------------
# Loader
# ------------------------------------------------------------------------------------------
class _Loader:
    def __init__(self, pack):
        self.p = pack
        self.line = 0
        self.by_name = {}
        self.bg_by_name = {}

    def err(self, msg):
        self.p.errors.append((self.line, msg))

    def check(self, ok, msg):
        if not ok:
            self.err(msg)
            raise _TagDropped()

    def check_ex(self, ok, msg):
        if self.p.version >= 109:
            self.check(ok, msg)
        elif not ok:
            self.err(msg)

    @staticmethod
    def tok(tokens, i, what):
        if i >= len(tokens):
            raise PackRejected(f"{what}: token {i} missing (undefined behaviour in Mesen)")
        return tokens[i]

    def add_global(self, name, ctype, args=()):
        for neg in (False, True):
            n = ("!" if neg else "") + name
            c = Condition(n, ctype, neg, tuple(args))
            self.by_name[n] = c

    def run(self):
        p = self.p
        try:
            data = p.read("hires.txt")
        except OSError:
            raise PackRejected("hires.txt not found")

        # InitializeGlobalConditions runs before any line is read, while Version is 0, so the
        # sppalette conditions (version >= 107 only) are never registered (LDR:114, 249-261).
        self.add_global("hmirror", "hmirror")
        self.add_global("vmirror", "vmirror")
        self.add_global("bgpriority", "bgpriority")
        if p.version >= 107:
            for n in range(4):
                self.add_global(f"sppalette{n}", "sppalette", (n,))

        text = data.decode("latin-1")         # std::string is bytes; keep them 1:1
        pos, n = 0, len(text)
        while pos < n:
            start = pos
            while pos < n:
                if text[pos] == "\n":
                    pos += 1
                    break
                pos += 1
            line = text[start:pos - 1] if pos < n else text[start:n]
            self.line += 1
            if not line:
                continue
            line = line.rstrip("\r\n")
            conds = []
            if line[:1] == "[":
                end = line.find("]", 1)
                if end < 0:
                    self.err("Invalid condition tag: " + line)
                    continue
                conds = self.parse_conditions(line[1:end])
                line = line[end + 1:]
            try:
                self.dispatch(line, conds)
            except _TagDropped:
                pass

        self.load_palette()
        for i, t in enumerate(p.tiles):        # InitializeHdPack (LDR:846-863)
            p.tile_by_key.setdefault(t.key.key(False), []).append(i)
            if t.default:
                p.tile_by_key.setdefault(t.key.key(True), []).append(i)
        p.conditions = dict(self.by_name)

    def dispatch(self, line, conds):
        p = self.p
        if line[:6] == "<tile>":
            self.tile_tag(split(line[6:]), conds)
        elif line[:12] == "<background>":
            self.background_tag(split(line[12:]), conds)
        elif line[:11] == "<condition>":
            tokens = split(line[11:])
            for neg in (False, True):
                try:
                    self.condition_tag(list(tokens), neg)
                except _TagDropped:
                    pass
        elif line[:5] == "<img>":
            name = line[5:]
            if not p.has(name):
                self.err(f"Error loading HDPack: PNG file {name} could not be read.")
                raise PackRejected(f"<img> {name} not found")
            p.images.append(name)
        elif line[:10] == "<addition>":
            self.addition_tag(split(line[10:]))
        elif line[:10] == "<fallback>":
            self.fallback_tag(split(line[10:]))
        elif line[:5] == "<bgm>":
            self.bgm_tag(split(line[5:]))
        elif line[:5] == "<sfx>":
            self.sfx_tag(split(line[5:]))
        elif line[:5] == "<ver>":
            p.version = stoi(line[5:])
            if p.version > CURRENT_VERSION:
                self.err("This HD Pack was built with a more recent version of Mesen")
                raise PackRejected(f"pack version {p.version} > {CURRENT_VERSION}")
        elif line[:7] == "<scale>":
            p.scale = stoi(line[7:])
            if p.scale > 10:
                self.err("Scale ratios higher than 10 are not supported.")
                raise PackRejected(f"scale {p.scale} > 10")
        elif line[:10] == "<overscan>":
            t = split(line[10:])
            p.overscan = tuple(stoi(self.tok(t, i, "overscan")) for i in range(4))
        elif line[:7] == "<patch>":
            self.patch_tag(split(line[7:]))
        elif line[:9] == "<options>":
            for token in split(line[9:]):
                opt = {"disableSpriteLimit": OPT_NO_SPRITE_LIMIT,
                       "alternateRegisterRange": OPT_ALTERNATE_REGISTER_RANGE,
                       "disableCache": OPT_DISABLE_CACHE,
                       "disableOriginalTiles": OPT_DONT_RENDER_ORIGINAL_TILES,
                       "automaticFallbackTiles": OPT_AUTOMATIC_FALLBACK_TILES,
                       "disableContours": 0}.get(token)
                if opt is None:
                    self.err("Invalid option: " + token)
                else:
                    p.options |= opt

    def parse_conditions(self, s):
        """ParseConditionString (LDR:790-820): whitespace ignored, '&' separates, unknown
        names logged and dropped (the tile becomes LESS restricted)."""
        out, name = [], ""

        def flush():
            nonlocal name
            c = self.by_name.get(name)
            if c is not None:
                out.append(c)
            else:
                self.err("Condition not found: " + name)
            name = ""

        for ch in s:
            if ch in " \n\r\t":
                continue
            if ch == "&":
                flush()
            else:
                name += ch
        flush()
        return out

    def read_tile_data(self, tile_tok, pal_tok):
        """ReadTileData (LDR:263-282)."""
        k = TileKey()
        if len(tile_tok) >= 32:
            k.data = bytes(from_hex(tile_tok[i * 2:i * 2 + 2]) & 0xFF for i in range(16))
            k.index, k.chr_ram = -1, True
        else:
            k.index = stoi(tile_tok) if self.p.version <= 102 else to_int32(from_hex(tile_tok))
            k.chr_ram = False
        k.palette = from_hex(pal_tok)
        return k

    def tile_tag(self, tokens, conds):
        p = self.p
        t = TileDef()
        t.line = self.line
        i = 0
        if p.version < 100:
            k = TileKey()
            k.index = stoi(self.tok(tokens, 0, "tile"))
            t.bitmap = stoi(self.tok(tokens, 1, "tile"))
            c = [stoi(self.tok(tokens, j, "tile")) for j in (2, 3, 4)]
            k.palette = (c[2] | (c[1] << 8) | (c[0] << 16)) & 0xFFFFFFFF
            i = 5
        else:
            t.bitmap = stoi(self.tok(tokens, 0, "tile"))
            k = self.read_tile_data(self.tok(tokens, 1, "tile"), self.tok(tokens, 2, "tile"))
            i = 3
        t.x = stoi(self.tok(tokens, i, "tile")); i += 1
        t.y = stoi(self.tok(tokens, i, "tile")); i += 1
        t.conditions = conds
        t.force_disable_cache = False
        for c in conds:
            if c.type in ("spriteNearby", "positionCheckX", "positionCheckY",
                          "originPositionCheckX", "originPositionCheckY"):
                t.force_disable_cache = True
            elif c.type == "tileNearby":
                x, y = c.args[0], c.args[1]
                if math.fmod(x, 8) > 0 or math.fmod(y, 8) > 0:   # C++ % keeps the sign
                    t.force_disable_cache = True
        if p.version >= 105:
            t.brightness = cpp_int(stof(self.tok(tokens, i, "tile")) * 255); i += 1
        elif p.version > 0:
            t.brightness = cpp_int(stof(self.tok(tokens, i, "tile")) * 255) & 0xFF; i += 1
        else:
            t.brightness = 255
        b = self.tok(tokens, i, "tile"); i += 1
        if b not in ("Y", "N"):
            self.err("Invalid boolean value: " + b)
        t.default = b == "Y"
        t.chr_bank = k.index // 256 if k.index >= 0 else -((-k.index) // 256)
        if p.version < 100:
            if len(tokens) >= 24:
                k.data = bytes(stoi(self.tok(tokens, i + j, "tile")) & 0xFF for j in range(16))
                i += 16
                k.chr_ram = True
            else:
                k.chr_ram = False
        else:
            if k.chr_ram and len(tokens) > i:
                t.chr_bank = stoul(tokens[i]); i += 1
            if k.chr_ram and len(tokens) > i:
                k.index = stoi(tokens[i]); i += 1
        t.key = k
        self.check_ex(len(tokens) == i, "Tile tag contains too many parameters")
        self.check(t.bitmap < len(p.images) if t.bitmap >= 0 else False,
                   f"Invalid bitmap index: {t.bitmap}")
        p.tiles.append(t)

    def condition_tag(self, tokens, negate):
        p = self.p
        self.check(len(tokens) >= 4, "Condition tag should contain at least 4 parameters")
        self.check(len(tokens[0]) > 0, "Condition name may not be empty")
        self.check("!" not in tokens[0], "Condition name may not contain '!' characters")
        ctype = tokens[1]
        if ctype in ("memoryCheck", "ppuMemoryCheck"):
            kind = "memoryCheck"
        elif ctype in ("memoryCheckConstant", "ppuMemoryCheckConstant"):
            kind = "memoryCheckConstant"
        elif ctype in ("tileAtPosition", "tileNearby", "spriteAtPosition", "spriteNearby",
                       "frameRange", "positionCheckX", "positionCheckY",
                       "originPositionCheckX", "originPositionCheckY"):
            kind = ctype
        else:
            self.err("Invalid condition type: " + ctype)
            return
        name = tokens[0].rstrip(" \n\r\t")
        if negate:
            name = "!" + name
        idx = 2
        if kind in ("tileNearby", "tileAtPosition", "spriteNearby", "spriteAtPosition"):
            self.check(len(tokens) >= 6, "Condition tag should contain at least 6 parameters")
            self.check_ex(len(tokens) < 9, "Condition tag contains too many parameters")
            x = stoi(tokens[idx]); y = stoi(tokens[idx + 1]); token = tokens[idx + 2]
            idx += 3
            tile_index, tile_data = -1, None
            if len(token) == 32:
                tile_data = bytes(from_hex(token[i * 2:i * 2 + 2]) & 0xFF for i in range(16))
            else:
                tile_index = stoi(token) if p.version < 104 else to_int32(from_hex(token))
            palette = from_hex(tokens[idx]); idx += 1
            ignore = False
            if len(tokens) >= 7:
                self.check_ex(p.version >= 108,
                              "Condition tag ignore palette feature requires version 108+")
                if p.version >= 108:
                    b = tokens[idx]; idx += 1
                    if b not in ("Y", "N"):
                        self.err("Invalid boolean value: " + b)
                    ignore = b == "Y"
            args = (x, y, tile_index, tile_data, palette, ignore)
        elif kind in ("memoryCheck", "memoryCheckConstant"):
            self.check(p.version >= 101, "This feature requires version 101+ of HD Packs")
            self.check(len(tokens) >= 5, "Condition tag should contain at least 5 parameters")
            self.check_ex(len(tokens) < 7, "Condition tag contains too many parameters")
            ppu = tokens[1][:3] == "ppu"
            a = from_hex(tokens[idx]); idx += 1
            if ppu:
                self.check(a <= 0x3FFF, "Out of range memoryCheck operand")
                a |= PPU_MEMORY_MARKER
            else:
                self.check(a <= 0xFFFF, "Out of range memoryCheck operand")
            op = OPS.get(tokens[idx]); idx += 1
            self.check(op is not None, "Invalid operator.")
            b = from_hex(tokens[idx]); idx += 1
            mask = 0xFF
            if len(tokens) > 5 and p.version >= 103:
                self.check(b <= 0xFF, "Out of range memoryCheck mask")    # (BUG? see spec 3.10)
                mask = from_hex(tokens[idx]) & 0xFF; idx += 1
            if kind == "memoryCheck":
                if ppu:
                    self.check(b <= 0x3FFF, "Out of range memoryCheck operand")
                    b |= PPU_MEMORY_MARKER
                else:
                    self.check(b <= 0xFFFF, "Out of range memoryCheck operand")
                p.watched.add(b)
            else:
                self.check(b <= 0xFF, "Out of range memoryCheckConstant operand")
            p.watched.add(a)
            args = (a, op, b, mask)
        elif kind in ("positionCheckX", "positionCheckY", "originPositionCheckX",
                      "originPositionCheckY"):
            self.check(p.version >= 108, "This condition type requires version 108+")
            self.check_ex(len(tokens) < 5, "Condition tag contains too many parameters")
            op = OPS.get(tokens[idx]); idx += 1
            self.check(op is not None, "Invalid operator.")
            v = stoi(tokens[idx]) & 0xFFFFFFFF
            self.check(v <= 0xFFFF, "Out of range positionCheck operand")
            args = (op, v)
        else:   # frameRange
            self.check(p.version >= 101, "This condition type requires version 101+")
            self.check_ex(len(tokens) < 5, "Condition tag contains too many parameters")
            if p.version == 101:
                a, b = to_int32(from_hex(tokens[idx])), to_int32(from_hex(tokens[idx + 1]))
            else:
                a, b = stoi(tokens[idx]), stoi(tokens[idx + 1])
            self.check(0 <= a <= 0xFFFF and 0 <= b <= 0xFFFF, "Out of range frameRange operand")
            args = (a, b)
        self.by_name[name] = Condition(name, kind, negate, args)

    def background_tag(self, tokens, conds):
        p = self.p
        self.check(len(tokens) >= 2, "Background tag should contain at least 2 parameters")
        self.check_ex(len(tokens) < 9, "Background tag contains too many parameters")
        name = tokens[0]
        if name not in self.bg_by_name:
            if p.has(name):
                self.bg_by_name[name] = name
                p.background_files.append(name)
        if name not in self.bg_by_name:
            fixed = self._fixed_file_name("BACKGROUND_FILE_FIXES", name)
            if fixed is None:
                self.check(False, "Error while loading background: " + name)
            # pack_fixes.py: the line is kept for the core under the file the pack does ship;
            # Mesen's own message is still logged, so self.errors stays Mesen's
            self.err("Error while loading background: " + name)
            self.p.compat.append((self.line, f"<background> {name} not in the pack; using "
                                             f"{fixed} (pack_fixes.py)"))
            self.bg_by_name[name] = fixed
            if fixed not in p.background_files:
                p.background_files.append(fixed)
        bg = Background()
        bg.line = self.line
        bg.file = self.bg_by_name[name]
        f = stof(tokens[1]) * 255
        bg.brightness = cpp_int(f) if p.version >= 105 else cpp_int(f) & 0xFF
        bg.hratio = bg.vratio = 0.0
        bg.priority, bg.left, bg.top, bg.blend = 10, 0, 0, "Alpha"
        bg.conditions = []
        for c in conds:
            if c.type in ("tileAtPosition", "spriteAtPosition", "memoryCheck",
                          "memoryCheckConstant", "frameRange"):
                bg.conditions.append(c)
            else:
                self.err("Invalid condition type for background: " + name)
        if len(tokens) > 2:
            self.check(p.version >= 101, "This feature requires version 101+ of HD Packs")
            bg.hratio = stof(tokens[2])
            if len(tokens) > 3:
                bg.vratio = stof(tokens[3])
            if len(tokens) > 4:
                self.check(p.version >= 102, "This feature requires version 102+ of HD Packs")
                if p.version >= 106:
                    pr = stoi(tokens[4])
                    self.check(0 <= pr < 40, "Invalid background priority value")
                    bg.priority = pr
                else:
                    b = tokens[4]
                    if b not in ("Y", "N"):
                        self.err("Invalid boolean value: " + b)
                    bg.priority = 0 if b == "Y" else 10
            if len(tokens) > 6:
                self.check(p.version >= 105, "This feature requires version 105+ of HD Packs")
                bg.left = max(0, stoi(tokens[5]))
                bg.top = max(0, stoi(tokens[6]))
            if len(tokens) > 7:
                self.check_ex(p.version >= 107, "Background blend mode requires version 107+")
                if tokens[7] in BLEND_MODES:
                    bg.blend = tokens[7]
                else:
                    self.err("Invalid blend mode: " + tokens[7])
        p.backgrounds[bg.priority].append(bg)

    def addition_tag(self, tokens):
        p = self.p
        self.check(p.version >= 107, "Addition tag requires version 107+ of HD Packs")
        self.check(len(tokens) >= 6, "Addition tag should contain at least 6 parameters")
        self.check_ex(len(tokens) < 8, "Addition tag contains too many parameters")
        a = Addition()
        a.original = self.read_tile_data(tokens[0], tokens[1])
        a.ox, a.oy = stoi(tokens[2]), stoi(tokens[3])
        a.added = self.read_tile_data(tokens[4], tokens[5])
        a.ignore_palette = False
        p.additions.append(a)       # pushed before the version-108 check, as in LDR:708
        if len(tokens) >= 7:
            self.check(p.version >= 108, "Addition tag ignore palette requires version 108+")
            b = tokens[6]
            if b not in ("Y", "N"):
                self.err("Invalid boolean value: " + b)
            a.ignore_palette = b == "Y"

    def fallback_tag(self, tokens):
        p = self.p
        self.check(p.version >= 107, "Fallback tag requires version 107+ of HD Packs")
        self.check(len(tokens) >= 2, "Fallback tag should contain at least 2 parameters")
        self.check_ex(len(tokens) < 3, "Fallback tag contains too many parameters")
        p.fallbacks.append((to_int32(from_hex(tokens[0])), to_int32(from_hex(tokens[1]))))

    def sound_id(self, album, track, filename):
        a = stoi(album)
        if not 0 <= a <= 255:
            self.err("Invalid album value: " + album)
            return -1
        t = stoi(track)
        if not 0 <= t <= 255:
            self.err("Invalid track value: " + track)
            return -1
        if not self.p.has(filename):
            self.err("OGG file not found: " + filename)
            return -1
        return a * 256 + t

    def bgm_tag(self, tokens):
        self.check(len(tokens) >= 3, "BGM tag should contain at least 3 parameters")
        self.check_ex(len(tokens) < 5, "BGM tag contains too many parameters")
        tid = self.sound_id(tokens[0], tokens[1], tokens[2])
        if tid >= 0:
            loop = stoul(tokens[3]) if len(tokens) > 3 else 0
            self.p.bgm[tid] = (tokens[2], loop)
        else:
            fx = self._sound_fix(tokens)
            if fx is not None:
                self.p.bgm[fx[0]] = (fx[1], fx[2])

    def sfx_tag(self, tokens):
        self.check(len(tokens) >= 3, "SFX tag should contain at least 3 parameters")
        self.check_ex(len(tokens) < 4, "SFX tag contains too many parameters")
        tid = self.sound_id(tokens[0], tokens[1], tokens[2])
        if tid >= 0:
            self.p.sfx[tid] = tokens[2]
        else:
            fx = self._sound_fix(tokens)
            if fx is not None:
                self.p.sfx[fx[0]] = fx[1]

    def _sound_fix(self, tokens):
        """pack_fixes.py SOUND_FILE_FIXES for a <bgm>/<sfx> line whose OGG is missing:
        (id, file, loop) to keep for the core, or None. `named` matches the file field, or the
        whole remainder of the line when the file name itself holds a comma (Mesen splits it,
        so the line can never load there). Mesen's own error has already been logged."""
        a, t = stoi(tokens[0]), stoi(tokens[1])
        if not (0 <= a <= 255 and 0 <= t <= 255):
            return None
        for named, loop_tok in ((tokens[2], tokens[3] if len(tokens) > 3 else None),
                                (",".join(tokens[2:]), None)):
            fixed = self._fixed_file_name("SOUND_FILE_FIXES", named)
            if fixed is not None:
                loop = stoul(loop_tok) if loop_tok is not None else 0
                self.p.compat.append((self.line, f"<bgm/sfx> {named} not in the pack; using "
                                                 f"{fixed} (pack_fixes.py)"))
                return a * 256 + t, fixed, loop
        return None

    def _fixed_file_name(self, table, name):
        """The file pack_fixes.py's `table` names for `name`, when the pack ships it with the
        recorded MD5; else None."""
        import hashlib
        import pack_fixes
        for f in getattr(pack_fixes, table):
            if f["named"] == name and self.p.has(f["actual"]) and \
                    hashlib.md5(self.p.read(f["actual"])).hexdigest() == f["actual_md5"]:
                return self.p.resolve(f["actual"])
        return None

    def patch_tag(self, tokens):
        self.check(len(tokens) >= 2, "Patch tag requires more parameters")
        self.check_ex(len(tokens) < 3, "Patch tag contains too many parameters")
        # the two extensions below record the line for the core, then let Mesen's own check
        # fail exactly as before, so self.patches and self.errors stay Mesen's
        name = self._fixed_patch_name(tokens[0], tokens[1])
        if (len(tokens[1]) == 32 and all(c in "0123456789abcdefABCDEF" for c in tokens[1])
                and self.p.has(name)):
            self.p.patches_md5[tokens[1].upper()] = name
            self.p.compat.append((self.line, f"<patch> {name}: MD5 line kept for the core "
                                             "(Mesen requires a SHA1 and rejects it)"))
        self.check(len(tokens[1]) == 40, f"Invalid SHA1 hash for patch ({tokens[0]}): {tokens[1]}")
        if name != tokens[0] and self.p.has(name):
            self.p.patches[tokens[1].upper()] = name
            self.p.compat.append((self.line, f"<patch> {tokens[0]} not in the pack; using {name} "
                                             "(pack_fixes.py)"))
            self.err("Patch file not found: " + tokens[1])     # what Mesen logs for this line
            return
        self.check(self.p.has(tokens[0]), "Patch file not found: " + tokens[1])
        self.p.patches[tokens[1].upper()] = tokens[0]

    def _fixed_patch_name(self, name, digest):
        """pack_fixes.py's corrected file for a <patch> line naming a file the pack lacks, when
        the corrected file is present with the recorded MD5; else the name unchanged."""
        if self.p.has(name):
            return name
        import hashlib
        import pack_fixes
        for f in pack_fixes.PATCH_FILE_FIXES:
            if f["named"].lower() == name.lower() and f["hash"].upper() == digest.upper():
                if self.p.has(f["actual"]) and \
                        hashlib.md5(self.p.read(f["actual"])).hexdigest() == f["actual_md5"]:
                    return self.p.resolve(f["actual"])
        return name

    def load_palette(self):
        if self.p.has("palette.dat"):
            d = self.p.read("palette.dat")
            pal = []
            for i in range(0, len(d), 3):                     # reads past an odd tail as 0s
                c = d[i:i + 3] + bytes(3)
                pal.append(0xFF000000 | (c[0] << 16) | (c[1] << 8) | c[2])
            if len(pal) == 0x40:
                self.p.palette = pal


def load(folder, files=None, reader=None):
    """Load a pack. Raises PackRejected where Mesen would refuse the whole pack.
    `files`/`reader` load a pack still inside an archive (see HdPack)."""
    p = HdPack(folder, files, reader)
    _Loader(p).run()
    return p


def main():
    p = load(sys.argv[1])
    print(f"version {p.version} scale {p.scale} options 0x{p.options:02X} overscan {p.overscan}")
    print(f"images {len(p.images)}  tiles {len(p.tiles)}  keys {len(p.tile_by_key)}  "
          f"conditions {len(p.conditions)}  backgrounds "
          f"{sum(len(b) for b in p.backgrounds)}  additions {len(p.additions)}  "
          f"fallbacks {len(p.fallbacks)}  watched {len(p.watched)}  bgm {len(p.bgm)}  "
          f"sfx {len(p.sfx)}  patches {len(p.patches)}")
    cond_tiles = sum(1 for t in p.tiles if t.conditions)
    ram = sum(1 for t in p.tiles if t.key.chr_ram)
    print(f"conditional tiles {cond_tiles}  CHR-RAM tiles {ram}  errors {len(p.errors)}")
    for line, msg in p.errors[:20]:
        print(f"  line {line}: {msg}")


if __name__ == "__main__":
    main()
