/* romfix_arm.c - the ARM half of the pack's <patch> (see romfix_arm.h). Moved here from the FPGA
 * (hd3_romfix.sv + hd3_sha1.sv) on 2026-10-28, OPEN_ITEMS item 11: the decision is the one the
 * RTL made, step for step, so the tests that pinned the RTL pin this.
 *
 * Which entry (tools/rommatch.py is the model, user decision 2026-10-22 "full compatibility"):
 * the SHA1 of the whole staged file first; if no entry names it and the file has an iNES header
 * with any of bytes 7-15 set, the SHA1 of the same file with those bytes 0. The patch is built
 * from the form that matched (a BPS checks its source's CRC). The staged original is never
 * changed, so a refused patch still leaves (or brings back) the file as it was loaded.
 *
 * The build follows section 14's run list (tools/hdp3_patch.py "build"), with the checks the RTL
 * made: sizes, the run table and the data inside the section, a run mode 0-4, a TargetCopy that
 * reads at or past the byte it writes, and the target CRC-32 for BPS/UPS. Two differences from
 * the RTL, both on malformed sections only: the run-table and data bounds are checked without
 * the RTL's 32-bit wrap (a wrapped sum let the RTL read past the section), and with base 0 the
 * bytes no run covers are 0 (the RTL left whatever DDR3 held; the converter never leaves any).
 */
#include <string.h>
#include "romfix_arm.h"

static uint32_t le32(const uint8_t *p) {
    return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24;
}

/* ---------------------------------------------------------------- SHA-1 (FIPS 180-4) */
static uint32_t rol(uint32_t x, int n) { return x << n | x >> (32 - n); }

static void sha1_block(uint32_t h[5], const uint8_t *b) {
    uint32_t w[80];
    for (int i = 0; i < 16; i++)
        w[i] = (uint32_t)b[4 * i] << 24 | b[4 * i + 1] << 16 | b[4 * i + 2] << 8 | b[4 * i + 3];
    for (int i = 16; i < 80; i++) w[i] = rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    uint32_t a = h[0], bb = h[1], c = h[2], d = h[3], e = h[4];
    for (int i = 0; i < 80; i++) {
        uint32_t f, k;
        if (i < 20)      { f = (bb & c) | (~bb & d);           k = 0x5A827999; }
        else if (i < 40) { f = bb ^ c ^ d;                     k = 0x6ED9EBA1; }
        else if (i < 60) { f = (bb & c) | (bb & d) | (c & d);  k = 0x8F1BBCDC; }
        else             { f = bb ^ c ^ d;                     k = 0xCA62C1D6; }
        uint32_t t = rol(a, 5) + f + e + k + w[i];
        e = d; d = c; c = rol(bb, 30); bb = a; a = t;
    }
    h[0] += a; h[1] += bb; h[2] += c; h[3] += d; h[4] += e;
}

/* SHA-1 of d with bytes zf..zt (inclusive) read as 0 (zf > zt: none) */
static void sha1_z(const uint8_t *d, size_t n, size_t zf, size_t zt, uint8_t out[20]) {
    uint32_t h[5] = {0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0};
    uint8_t blk[64];
    size_t i = 0;
    for (; i + 64 <= n; i += 64) {
        if (i <= zt && i + 64 > zf) {
            memcpy(blk, d + i, 64);
            for (size_t k = zf; k <= zt; k++) if (k >= i && k < i + 64) blk[k - i] = 0;
            sha1_block(h, blk);
        } else sha1_block(h, d + i);
    }
    size_t r = n - i;
    memset(blk, 0, 64);
    memcpy(blk, d + i, r);
    for (size_t k = zf; k <= zt; k++) if (k >= i && k < i + r) blk[k - i] = 0;
    blk[r] = 0x80;
    if (r >= 56) { sha1_block(h, blk); memset(blk, 0, 64); }
    uint64_t bits = (uint64_t)n * 8;
    for (int k = 0; k < 8; k++) blk[63 - k] = (uint8_t)(bits >> (8 * k));
    sha1_block(h, blk);
    for (int k = 0; k < 5; k++) {
        out[4 * k] = h[k] >> 24; out[4 * k + 1] = h[k] >> 16;
        out[4 * k + 2] = h[k] >> 8; out[4 * k + 3] = h[k];
    }
}

void rf_sha1(const uint8_t *d, size_t n, uint8_t out[20]) { sha1_z(d, n, 1, 0, out); }

/* ---------------------------------------------------------------- CRC-32 (zlib's) */
static uint32_t crc32_of(const uint8_t *d, uint32_t n) {
    static uint32_t tab[256];
    static int init = 0;
    if (!init) {
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t c = i;
            for (int k = 0; k < 8; k++) c = c & 1 ? (c >> 1) ^ 0xEDB88320u : c >> 1;
            tab[i] = c;
        }
        init = 1;
    }
    uint32_t c = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < n; i++) c = tab[(c ^ d[i]) & 0xFF] ^ (c >> 8);
    return ~c;
}

/* ---------------------------------------------------------------- section 14 */
/* the entry whose SHA1 is sha, or -1: entries in order, each wholly inside the section */
static int search(const uint8_t *s14, uint32_t plen, const uint8_t sha[20]) {
    if (!s14 || plen < 64) return -1;
    uint32_t cnt = le32(s14);
    for (uint32_t ek = 0; ; ek++) {
        if (ek >= cnt || ek >= 1024 || (uint64_t)ek * 64 + 128 > plen) return -1;
        if (!memcmp(s14 + 64 + 64 * (size_t)ek, sha, 20)) return (int)ek;
    }
}

uint32_t rf_build(const uint8_t *s14, uint32_t plen, const uint8_t *e, const uint8_t *src,
                  uint32_t L, int p2, uint8_t *tgt, int *why) {
    uint8_t base = e[20], flags = e[21];
    uint32_t ssz = le32(e + 24), tsz = le32(e + 28), trunc = le32(e + 32), crc = le32(e + 36);
    uint32_t roff = le32(e + 40), rn = le32(e + 44), doff = le32(e + 48), dlen = le32(e + 52);
    /* target size: exact, or max(source, last IPS record), then truncated */
    uint32_t szs = ((flags & 2) && L > tsz) ? L : tsz;
    uint32_t size = ((flags & 4) && trunc < szs) ? trunc : szs;
    *why = 3;
    if (((flags & 8) && ssz != L) || size == 0 || size > RF_MAXB || (roff & 63) ||
        (uint64_t)roff + (uint64_t)rn * 16 > plen || (uint64_t)doff + dlen > plen || (rn >> 28))
        return 0;
    const uint8_t *data = s14 + doff;
#define SRCB(i) ((i) >= L || (p2 && (i) >= 7 && (i) <= 15) ? 0 : src[i])
    if (base & 1) for (uint32_t i = 0; i < size; i++) tgt[i] = SRCB(i);
    else memset(tgt, 0, size);
    /* Bytes written by the records, bounded: a converted patch writes each target byte a few
       times at most, while a malformed table (millions of records, each up to the whole target)
       could keep this busy for hours - refuse it instead (the FPGA's wait ends at ~1.5 s). */
    uint64_t budget = 16ull * RF_MAXB;
    for (uint32_t r = 0; r < rn; r++) {
        const uint8_t *rr = s14 + roff + 16 * (size_t)r;
        uint32_t rt = le32(rr), n = le32(rr + 4), ra = le32(rr + 8);
        uint8_t rm = rr[12];
        if (rm > 4) return 0;
        if (n == 0 || rt >= size) continue;
        uint64_t span = (uint64_t)size - rt < n ? (uint64_t)size - rt : n;
        if (span > budget) return 0;
        budget -= span;
        for (uint32_t ri = 0; ; ri++) {
            uint32_t tpos = rt + ri, aidx = ra + ri;     /* 32-bit, as the RTL */
            uint8_t b;
            switch (rm) {
            case 0:  if (aidx >= dlen) return 0; b = data[aidx]; break;          /* LIT */
            case 1:  b = SRCB(aidx); break;                                      /* SRC */
            case 2:  if (aidx >= tpos) return 0; b = tgt[aidx]; break;           /* TGT */
            case 3:  b = SRCB(tpos); if (aidx >= dlen) return 0; b ^= data[aidx]; break;  /* XOR */
            default: b = (uint8_t)ra; break;                                     /* FILL */
            }
            tgt[tpos] = b;
            if (ri + 1 >= n || tpos + 1 >= size) break;
        }
    }
#undef SRCB
    if ((flags & 1) && crc32_of(tgt, size) != crc) { *why = 4; return 0; }
    *why = 1;
    return size;
}

/* ---------------------------------------------------------------- decision */
void rf_init(rf_state *st) { memset(st, 0, sizeof *st); }

int rf_session_check(rf_state *st, uint64_t cmd_now) {
    if (!st->have || cmd_now != 0) return 0;
    rf_init(st);
    return 1;
}

int rf_pending(const rf_state *st, uint64_t s) {
    if (RF_MAGIC(s) != 0xA6 || !RF_STAGED(s)) return 0;
    return !st->have || RF_RSEQ(s) != st->rseq || RF_ASEQ(s) != st->aseq;
}

int rf_decide(rf_state *st, uint64_t s, const uint8_t *rom, const uint8_t *s14, uint32_t s14_len,
              uint8_t *tgt, uint32_t *tgt_len, uint64_t *cmd) {
    *tgt_len = 0;
    if (!rf_pending(st, s)) return 0;
    st->have = 1; st->rseq = RF_RSEQ(s); st->aseq = RF_ASEQ(s);
    uint32_t L = RF_LEN(s);
    if (L > RF_MAXB) L = RF_MAXB;                     /* the FPGA never stages more */
    if (!st->hashed || st->hrseq != st->rseq) {       /* once per ROM */
        rf_sha1(rom, L, st->sha_raw);
        st->n_cand = L >= 16 && !memcmp(rom, "NES\x1a", 4);
        if (st->n_cand) {
            int nz = 0;
            for (int k = 7; k <= 15; k++) nz |= rom[k];
            st->n_cand = nz != 0;
        }
        if (st->n_cand) sha1_z(rom, L, 7, 15, st->sha_n);
        st->hashed = 1; st->hrseq = st->rseq;
    }
    unsigned result = 2, action = 0;
    uint32_t len = 0;
    if (RF_ACTIVE(s) && s14 && s14_len >= 64) {
        int ek = search(s14, s14_len, st->sha_raw), p2 = 0;
        if (ek < 0 && st->n_cand) { ek = search(s14, s14_len, st->sha_n); p2 = 1; }
        if (ek >= 0) {
            int why;
            len = rf_build(s14, s14_len, s14 + 64 + 64 * (size_t)ek, rom, L, p2, tgt, &why);
            result = (unsigned)why;
            if (len) action = 1;
        }
    }
    if (!action && RF_CONTENT(s) != 0) action = 2;    /* the core holds a patched ROM: undo it */
    *tgt_len = action == 1 ? len : 0;
    st->id++;
    if (st->id == RF_LASTID(s)) st->id++;
    *cmd = 0xC7ull << 56 | (uint64_t)st->rseq << 48 | (uint64_t)st->aseq << 40 |
           (uint64_t)st->id << 32 | (uint64_t)action << 30 | (uint64_t)result << 27 | len;
    return 1;
}
