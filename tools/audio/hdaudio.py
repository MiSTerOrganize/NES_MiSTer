"""Stage-6 HD audio: shared codec, section-15 format and the reference model of the device.

Source for every behaviour: Mesen2 HdAudioDevice.cpp / OggMixer.cpp / OggReader.cpp
(tools/audio/mesen/), summarised in HDPACK_DESIGN.md "Stage 6 - audio" and
MESEN_HD_RUNTIME_SPEC.md section 8.

The ADPCM decoder here is the hardware's (rtl/hdpack/hd3_audio.sv) and the C encoder's
(tools/audio/oggdecode.cpp); tools/audio/test_ima.py checks the three agree.
"""
import struct

STEP = (
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45, 50, 55, 60,
    66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371,
    408, 449, 494, 544, 598, 658, 724, 796, 876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878,
    2066, 2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845,
    8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086,
    29794, 32767)
IDX = (-1, -1, -1, -1, 2, 4, 6, 8)

RATE = 48000
MAX_ENTRIES = 512
SFX_VOICES = 7          # plus one BGM voice
MAGIC = b"HDAU"
DC_SHIFT = 10           # NES DC blocker pole: 1 - 2^-DC_SHIFT
# Restore checkpoints (since 2026-10-29): the ADPCM state before every CK_SPACING-th frame of
# each entry, so a savestate load decodes at most CK_SPACING - 1 frames whatever its frame
# (tools/armdec/aud_arm.h AUD_CK must equal it, or the ARM ignores the table and builds its own)
CK_SPACING = 16384


def clamp16(v):
    return -32768 if v < -32768 else 32767 if v > 32767 else v


def ima_decode(pred, idx, n):
    step = STEP[idx]
    diff = step >> 3
    if n & 4:
        diff += step
    if n & 2:
        diff += step >> 1
    if n & 1:
        diff += step >> 2
    pred = clamp16(pred - diff if n & 8 else pred + diff)
    idx = min(88, max(0, idx + IDX[n & 7]))
    return pred, idx


def ima_encode(pred, idx, sample):
    step = STEP[idx]
    delta = sample - pred
    n = 0
    if delta < 0:
        n, delta = 8, -delta
    if delta >= step:
        n |= 4
        delta -= step
    if delta >= step >> 1:
        n |= 2
        delta -= step >> 1
    if delta >= step >> 2:
        n |= 1
    pred, idx = ima_decode(pred, idx, n)
    return n, pred, idx


def _best_last(pred, idx, sample):
    """smallest squared error any nibble gives for sample (oggdecode.cpp best_last)"""
    neg = 8 if sample - pred < 0 else 0
    lo = 0
    for m in range(8):
        p, _ = ima_decode(pred, idx, m | neg)
        if (p >= sample) if neg else (p <= sample):
            lo = m
        else:
            break
    best = None
    for m in range(lo, min(lo + 2, 8)):
        p, _ = ima_decode(pred, idx, m | neg)
        e = (sample - p) ** 2
        if best is None or e < best:
            best = e
    p, _ = ima_decode(pred, idx, 0 if neg else 8)   # other sign, magnitude 0 (the clamp case)
    return min(best, (sample - p) ** 2)


def _best_cost(pred, idx, xs):
    if len(xs) == 1:
        return _best_last(pred, idx, xs[0])
    best = None
    for n in range(16):
        p, i = ima_decode(pred, idx, n)
        c = (xs[0] - p) ** 2 + _best_cost(p, i, xs[1:])
        if best is None or c < best:
            best = c
    return best


def ima_encode_la(pred, idx, xs):
    """Lookahead encoder (oggdecode.cpp ima_encode_la): the nibble with the smallest total
    squared error over xs (this sample and the next len(xs)-1). Ties go to the lowest nibble."""
    best_n = best = None
    for n in range(16):
        p, i = ima_decode(pred, idx, n)
        c = (xs[0] - p) ** 2 + (_best_cost(p, i, xs[1:]) if len(xs) > 1 else 0)
        if best is None or c < best:
            best, best_n = c, n
    pred, idx = ima_decode(pred, idx, best_n)
    return best_n, pred, idx


def pack_state(pl, il, pr, ir):
    return (pl & 0xFFFF) | (il << 16) | ((pr & 0xFFFF) << 32) | (ir << 48)


def unpack_state(s):
    def s16(v):
        return v - 0x10000 if v & 0x8000 else v
    return s16(s & 0xFFFF), (s >> 16) & 0x7F, s16((s >> 32) & 0xFFFF), (s >> 48) & 0x7F


def encode_stereo(pcm, loop_frame, depth=0):
    """pcm: list of (l, r). Returns (bytes, state0, statel) as oggdecode does with the same
    depth. depth 0 = the greedy encoder (fast; the testbench fixtures use it); the converter
    uses depth 3 (oggdecode.cpp kDepth). Depth 3 is slow in Python: test slices only."""
    if not pcm:
        return b"", 0, 0
    pl, il, pr, ir = pcm[0][0], 0, pcm[0][1], 0
    state0 = pack_state(pl, il, pr, ir)
    statel = state0
    out = bytearray(len(pcm))
    for i, (l, r) in enumerate(pcm):
        if i == loop_frame:
            statel = pack_state(pl, il, pr, ir)
        if depth:
            ahead = pcm[i:i + depth]
            nl, pl, il = ima_encode_la(pl, il, [x[0] for x in ahead])
            nr, pr, ir = ima_encode_la(pr, ir, [x[1] for x in ahead])
        else:
            nl, pl, il = ima_encode(pl, il, l)
            nr, pr, ir = ima_encode(pr, ir, r)
        out[i] = nl | (nr << 4)
    return bytes(out), state0, statel


def vol(x, v):
    """Mesen: (int32_t)(sample * volume / 255), C division truncates toward zero."""
    p = x * v
    q = abs(p) // 255
    return -q if p < 0 else q


# ---------------------------------------------------------------------------- section 15
class Entry:
    __slots__ = ("key", "start", "frames", "loop", "state0", "statel")

    def __init__(self, key, start, frames, loop, state0, statel):
        self.key, self.start, self.frames, self.loop = key, start, frames, loop
        self.state0, self.statel = state0, statel


def checkpoints(stream, frames, state0, spacing=CK_SPACING):
    """The packed ADPCM state before frame k * spacing, k = 0 .. frames // spacing (state 0 first;
    the last one is the state after every frame when spacing divides frames)."""
    pl, il, pr, ir = unpack_state(state0)
    out = [state0]
    for i in range(frames):
        b = stream[i]
        pl, il = ima_decode(pl, il, b & 15)
        pr, ir = ima_decode(pr, ir, b >> 4)
        if (i + 1) % spacing == 0:
            out.append(pack_state(pl, il, pr, ir))
    return out


def build_section(lines):
    """lines: list of (is_sfx, id, stream_bytes, frames, loopframe, state0, statel[, cks]);
    entries that share a stream pass the same bytes object. cks = checkpoints() of the stream
    (the converter takes them from oggdecode; computed here when absent). Returns the section
    bytes: header {MAGIC, n, data_off, ck_off}, the entry table, the streams, then at ck_off
    {u32 spacing, u32 count, count x u64}: every entry's checkpoints in table order."""
    lines = [tuple(t[:7]) + (t[7] if len(t) > 7 else checkpoints(t[2], t[3], t[5]),)
             for t in lines]
    for t in lines:
        if len(t[7]) != t[3] // CK_SPACING + 1 or t[7][0] != t[5]:
            raise ValueError("audio checkpoints do not match the stream")
    lines = sorted(lines, key=lambda t: (int(t[0]) << 16) | t[1])
    keys = [(int(t[0]) << 16) | t[1] for t in lines]
    if len(set(keys)) != len(keys):
        raise ValueError("duplicate audio key")
    if len(lines) > MAX_ENTRIES:
        raise ValueError("more than %d audio entries" % MAX_ENTRIES)
    n = len(lines)
    data_off = (16 + 32 * n + 63) & ~63
    blobs, where, body = {}, [], bytearray()
    for t in lines:
        b = t[2]
        if id(b) not in blobs:
            blobs[id(b)] = data_off + len(body)
            body += b
            body += bytes((-len(body)) & 7)
        where.append(blobs[id(b)])
    out = bytearray(MAGIC + struct.pack("<III", n, data_off, 0))
    for (is_sfx, tid, b, frames, loop, s0, sl, _), start in zip(lines, where):
        out += struct.pack("<IIIIQQ", (int(is_sfx) << 16) | tid, start, frames, loop, s0, sl)
    out += bytes(data_off - len(out))
    out += body
    out += bytes((-len(out)) & 7)
    ck = [s for t in lines for s in t[7]]
    struct.pack_into("<I", out, 12, len(out))
    out += struct.pack("<II", CK_SPACING, len(ck)) + struct.pack("<%dQ" % len(ck), *ck)
    out += bytes((-len(out)) & 63)
    return bytes(out)


def parse_checkpoints(sec):
    """The checkpoint table, (spacing, [u64...]) in table order, or None (a section built before
    2026-10-29 has ck_off 0)."""
    (ck_off,) = struct.unpack_from("<I", sec, 12)
    if not ck_off:
        return None
    spacing, count = struct.unpack_from("<II", sec, ck_off)
    return spacing, list(struct.unpack_from("<%dQ" % count, sec, ck_off + 8))


def parse_section(sec):
    if len(sec) < 16 or sec[:4] != MAGIC:
        raise ValueError("not an HD audio section")
    n, data_off, _ = struct.unpack_from("<III", sec, 4)
    ents = []
    for i in range(n):
        key, start, frames, loop, s0, sl = struct.unpack_from("<IIIIQQ", sec, 16 + 32 * i)
        ents.append(Entry(key, start, frames, loop, s0, sl))
    return ents


# ---------------------------------------------------------------------------- the device
class Voice:
    def __init__(self, ent, loop, serial):
        self.e = ent
        self.loop = loop
        self.pos = 0
        self.pl, self.il, self.pr, self.ir = unpack_state(ent.state0)
        self.serial = serial      # start order (SFX mixing order)

    def next(self, sec):
        """one frame (l, r); returns None after the last frame of a non-looping stream"""
        e = self.e
        if self.pos >= e.frames:
            if not self.loop:
                return None
            self.pos = e.loop
            self.pl, self.il, self.pr, self.ir = unpack_state(e.statel)
        b = sec[e.start + self.pos]
        self.pl, self.il = ima_decode(self.pl, self.il, b & 15)
        self.pr, self.ir = ima_decode(self.pr, self.ir, b >> 4)
        self.pos += 1
        return self.pl, self.pr


class Device:
    """Register semantics of Mesen's HdAudioDevice and the mix of OggMixer, one 48 kHz frame at a
    time. A voice that has produced its last frame is dropped before the next tick (Mesen marks
    it done inside the mix call and erases it after the call)."""

    def __init__(self, sec, alternate=False):
        self.sec = sec
        self.alt = alternate
        self.ent_list = parse_section(sec) if sec else []
        self.ents = {e.key: e for e in self.ent_list}
        self.ent_index = {id(e): i for i, e in enumerate(self.ent_list)}
        self.album = 0
        self.options = 0
        self.paused = False
        self.bgm_vol = 128
        self.sfx_vol = 128
        self.err = False
        self.bgm = None
        self.sfx = []
        self.serial = 0
        self.dc_x = 0
        self.dc_y = 0

    # bus -----------------------------------------------------------------
    def maps_write(self, addr):
        if self.alt:
            return addr in tuple(0x3002 + 0x10 * i for i in range(7))
        return 0x4100 <= addr <= 0x4106

    def maps_read(self, addr):
        return addr in (0x4018, 0x4019) if self.alt else 0x4100 <= addr <= 0x4106

    def write(self, addr, v):
        reg = (addr & 0xF) if addr > 0x4100 else ((addr & 0xF0) >> 4)
        if reg == 0:
            self.options = v
            if self.bgm:
                self.bgm.loop = bool(v & 1)
        elif reg == 1:
            self.paused = bool(v & 1)
            if v & 2:
                self.bgm = None
            if v & 4:
                self.sfx = []
        elif reg == 2:
            self.bgm_vol = v
        elif reg == 3:
            self.sfx_vol = v
        elif reg == 4:
            self.album = v
        elif reg == 5:
            # PlayBgmTrack returns true when the track is defined AND opens; a track whose file
            # does not decode (an entry with 0 frames) leaves the current BGM playing
            e = self.ents.get(self.album * 256 + v)
            ok = e is not None and e.frames > 0
            if ok:
                self.bgm = Voice(e, bool(self.options & 1), 0)
            self.err = ok
        elif reg == 6:
            # PlaySfx returns !Play(...) when defined (true only if the file fails to open), and
            # false when undefined
            e = self.ents.get(0x10000 | (self.album * 256 + v))
            if e is not None and e.frames > 0:
                self.serial += 1
                if len(self.sfx) >= SFX_VOICES:  # Mesen has no limit: replace the oldest
                    self.sfx.pop(0)
                self.sfx.append(Voice(e, False, self.serial))
            self.err = e is not None and e.frames == 0

    def read(self, addr):
        r = addr & 7
        if r == 0:
            return (int(self.bgm is not None and not self.paused) | (int(bool(self.sfx)) << 1)
                    | (int(self.err) << 2))
        return {1: 1, 2: 0x4E, 3: 0x45, 4: 0x41}.get(r, 0)

    # savestate -----------------------------------------------------------
    def save(self):
        """The two 64-bit savestate words (hd3_audio.sv / NES.sv SSREG_INDEX_HDAUD_A/B):
        A = album, volumes, loop option, BGM on + entry index + next frame; B = 0. Mesen saves
        the same registers and the BGM's file offset.
        (B held the BGM's ADPCM state until 2026-10-28, when the decoding moved to the ARM
        (tools/armdec/aud_arm.c): the state at a frame is a function of the frame - see load.)"""
        a = (self.album << 56) | (self.bgm_vol << 48) | (self.sfx_vol << 40) | \
            ((self.options & 1) << 39)
        if self.bgm is not None:
            a |= (1 << 38) | (self.ent_index[id(self.bgm.e)] << 29) | self.bgm.pos
        return a, 0

    def state_at(self, e, pos):
        """the ADPCM state before frame pos: frames 0..pos-1 decoded from state 0 (a loop
        restart takes the converter's loop state, which is the first pass's state at the loop
        frame, so this is the state either pass has there)"""
        v = Voice(e, False, 0)
        for _ in range(min(pos, e.frames)):
            b = self.sec[e.start + v.pos]
            v.pl, v.il = ima_decode(v.pl, v.il, b & 15)
            v.pr, v.ir = ima_decode(v.pr, v.ir, b >> 4)
            v.pos += 1
        return v.pl, v.il, v.pr, v.ir

    def load(self, a, b=0):
        """Restore save() words: registers, and the BGM at its saved frame or none. SFX and the
        pause flag are left as they are (Mesen does not save them). B is not read."""
        self.album = (a >> 56) & 0xFF
        self.bgm_vol = (a >> 48) & 0xFF
        self.sfx_vol = (a >> 40) & 0xFF
        self.options = (a >> 39) & 1
        idx = (a >> 29) & 0x1FF
        if (a >> 38) & 1 and idx < len(self.ent_list):
            e = self.ent_list[idx]
            v = Voice(e, bool(self.options & 1), 0)
            v.pos = a & 0x1FFFFFFF
            v.pl, v.il, v.pr, v.ir = self.state_at(e, v.pos)
            self.bgm = v
        else:
            self.bgm = None

    # mix -----------------------------------------------------------------
    def dc_block(self, nes_unsigned):
        """NES path while the device is active: unsigned core sample -> signed, first-order DC
        blocker y = x - x1 + y1 - (y1 >> DC_SHIFT) (arithmetic shift), output clamped to s16."""
        x = (nes_unsigned & 0xFFFF) - 0x8000
        y = x - self.dc_x + self.dc_y - (self.dc_y >> DC_SHIFT)
        self.dc_x, self.dc_y = x, y
        return clamp16(y)

    @staticmethod
    def nes_level(c):
        """NES level relative to HD audio: 5/8 as x/2 + x/8 with flooring shifts (the RTL's
        nes_level; measured against Mesen, HDPACK_DESIGN.md "Stage 6 - NES level")."""
        return (c >> 1) + (c >> 3)

    def voices(self):
        """One frame of the voices: the volume-scaled (l, r) of each in mix order (BGM, then the
        SFX in start order) - what the ARM writes to a PCM slot (tools/armdec/aud_arm.c)."""
        out = []
        if self.bgm is not None and not self.paused:
            s = self.bgm.next(self.sec)
            if s is None:
                self.bgm = None
            else:
                out.append((vol(s[0], self.bgm_vol), vol(s[1], self.bgm_vol)))
                if self.bgm.pos >= self.bgm.e.frames and not self.bgm.loop:
                    self.bgm = None
        keep = []
        for v in self.sfx:
            s = v.next(self.sec)
            if s is None:
                continue
            out.append((vol(s[0], self.sfx_vol), vol(s[1], self.sfx_vol)))
            if v.pos < v.e.frames:
                keep.append(v)
        self.sfx = keep
        return out

    @staticmethod
    def mix(nes_l, nes_r, vs):
        """OggMixer's order: the NES contribution, then each voice, clamped after each."""
        l, r = nes_l, nes_r
        for vl, vr in vs:
            l = clamp16(l + vl)
            r = clamp16(r + vr)
        return l, r

    def tick(self, nes_l, nes_r):
        """nes_l/nes_r: the signed NES contribution for this frame. Returns (l, r)."""
        return self.mix(nes_l, nes_r, self.voices())
