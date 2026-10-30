"""Tests for rompatch: each format applied to a known ROM, plus the refusals.

BPS and UPS patches are built here by small independent encoders (the formats are simple
enough that an encoder is a few lines), so a decoder bug cannot be hidden by sharing code.
usage: python -I test_rompatch.py      exit 0 = all pass
"""
import os
import random
import sys
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rompatch  # noqa: E402

fails = 0


def check(name, ok):
    global fails
    print(("PASS " if ok else "FAIL ") + name)
    fails += not ok


def vlq(n):
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        if n == 0:
            out.append(b | 0x80)
            return bytes(out)
        out.append(b)
        n -= 1


def crcs(body, src, dst):
    body = bytes(body) + zlib.crc32(src).to_bytes(4, "little") + zlib.crc32(dst).to_bytes(4, "little")
    return body + zlib.crc32(body).to_bytes(4, "little")


def make_bps(src, dst):
    """SourceRead where bytes match, TargetRead elsewhere, one SourceCopy and one TargetCopy."""
    p = bytearray(b"BPS1") + vlq(len(src)) + vlq(len(dst)) + vlq(0)
    i = 0
    src_rel = 0
    while i < len(dst):
        if i < len(src) and src[i] == dst[i]:
            j = i
            while j < len(dst) and j < len(src) and src[j] == dst[j]:
                j += 1
            p += vlq(((j - i) - 1) << 2 | 0)
        else:
            j = i
            while j < len(dst) and not (j < len(src) and src[j] == dst[j]):
                j += 1
            if j - i >= 8 and dst[i:j] == bytes([dst[i]]) * (j - i):
                # one TargetRead byte, then an overlapping TargetCopy that repeats it
                p += vlq(0 << 2 | 1) + dst[i:i + 1]
                p += vlq(((j - i - 1) - 1) << 2 | 3) + vlq(i << 1)   # dst_rel 0 -> i
                global used_targetcopy
                used_targetcopy = True
            else:
                p += vlq(((j - i) - 1) << 2 | 1) + dst[i:j]
        i = j
    # exercise SourceCopy: re-encode the first 16 bytes from the source's tail
    del src_rel
    return crcs(p, src, dst)


def make_bps_sourcecopy(src):
    """Target = src reversed in 32-byte blocks, all via SourceCopy (relative seeks)."""
    blocks = [src[k:k + 32] for k in range(0, len(src), 32)][::-1]
    dst = b"".join(blocks)
    p = bytearray(b"BPS1") + vlq(len(src)) + vlq(len(dst)) + vlq(0)
    rel = 0
    for b in range(len(blocks)):
        start = (len(blocks) - 1 - b) * 32
        d = start - rel
        p += vlq((len(blocks[b]) - 1) << 2 | 2) + vlq((abs(d) << 1) | (d < 0))
        rel = start + len(blocks[b])
    return crcs(p, src, dst), dst


def make_ups(src, dst):
    p = bytearray(b"UPS1") + vlq(len(src)) + vlq(len(dst))
    n = max(len(src), len(dst))
    a = src + bytes(n - len(src))
    b = dst + bytes(n - len(dst))
    i = 0
    last = 0
    while i < n:
        if a[i] == b[i]:
            i += 1
            continue
        p += vlq(i - last)
        while i < n and a[i] != b[i]:
            p.append(a[i] ^ b[i])
            i += 1
        p.append(0)
        i += 1
        last = i
    return crcs(p, src, dst)


rng = random.Random(7)
src = bytes(rng.randrange(256) for _ in range(4096))
dst = bytearray(src)
for k in range(40):
    o = rng.randrange(len(dst) - 20)
    n = rng.randrange(1, 20)
    dst[o:o + n] = bytes((src[o + k] + 1 + rng.randrange(255)) & 0xFF for k in range(n))
dst = bytes(dst) + bytes([0x5A]) * 300          # grow, with a run for TargetCopy
assert len(dst) == 4096 + 300

# IPS
ips = (b"PATCH" + (10).to_bytes(3, "big") + (3).to_bytes(2, "big") + b"abc"
       + (20).to_bytes(3, "big") + (0).to_bytes(2, "big") + (5).to_bytes(2, "big") + b"Z"
       + (4100).to_bytes(3, "big") + (2).to_bytes(2, "big") + b"xy" + b"EOF")
want = bytearray(src)
want[10:13] = b"abc"
want[20:25] = b"ZZZZZ"
want += bytes(4100 - len(want)) + b"xy"
check("IPS records + RLE + growth", rompatch.apply(src, ips) == bytes(want))
check("IPS truncation extension", rompatch.apply(src, ips + (100).to_bytes(3, "big")) == bytes(want[:100]))
for name, bad in (("IPS no EOF", ips[:-3]), ("IPS short record", ips[:9])):
    try:
        rompatch.apply(src, bad)
        check(name + " refused", False)
    except rompatch.PatchError:
        check(name + " refused", True)

# BPS
used_targetcopy = False
bps = make_bps(src, dst)
check("BPS encoder exercised TargetCopy", used_targetcopy)
check("BPS SourceRead/TargetRead/TargetCopy", rompatch.apply(src, bps) == dst)
p, d2 = make_bps_sourcecopy(src)
check("BPS SourceCopy with backward seeks", rompatch.apply(src, p) == d2)
try:
    rompatch.apply(src[:-1] + b"\x00", make_bps(src, dst))
    check("BPS wrong source refused", False)
except rompatch.PatchError:
    check("BPS wrong source refused", True)
bp = bytearray(make_bps(src, dst))
bp[10] ^= 1
try:
    rompatch.apply(src, bytes(bp))
    check("BPS corrupt patch refused", False)
except rompatch.PatchError:
    check("BPS corrupt patch refused", True)

# UPS
check("UPS grow", rompatch.apply(src, make_ups(src, dst)) == dst)
check("UPS shrink", rompatch.apply(src, make_ups(src, src[:3000])) == src[:3000])
try:
    rompatch.apply(dst, make_ups(src, dst))
    check("UPS wrong source refused", False)
except rompatch.PatchError:
    check("UPS wrong source refused", True)

try:
    rompatch.apply(src, b"NOPE....")
    check("unknown format refused", False)
except rompatch.PatchError:
    check("unknown format refused", True)

print("ALL PASS" if fails == 0 else f"{fails} FAILED")
sys.exit(1 if fails else 0)
