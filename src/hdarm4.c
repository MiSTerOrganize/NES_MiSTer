/* hdarm3.c - hdarm2.c without a per-pixel record array (OPEN_ITEMS item 11, round 3, 2026-10-22).
 *
 * Round 2 left an 8.3 ms floor on an EMPTY frame: writing and reading 61,440 32-byte pixel
 * records. Here a pixel's tile record is rebuilt on demand from the capture's 4-byte entry and
 * a per-line table of span records; only sprite pixels get a stored block, and the fallback
 * rewrite of a tile's index lives in an overlay (bitmap + int32 array) because the original
 * keeps it in the record. The additions pass's copy of each line's first pixel over (0,0) is
 * modelled by pointing (0,0) at that line (P0L) and copying its sprite block. Decisions must
 * stay byte-identical; parity.sh checks it.
 *
 * Round 2 header follows.
 *
 * hdarm2.c - hdarm.c restructured for speed, decisions unchanged (OPEN_ITEMS item 11, 2026-10-22).
 *
 * hdarm.c is the straight port and stays as the baseline. Changes here, each one exact:
 *   - compact records: Rec 24 bytes, sprites in a pool of 4-slot blocks allocated only for
 *     pixels that have a sprite (Pix 32 bytes instead of 168: 2 MB written per frame, not 10)
 *   - key-table lookups memoised per frame (find_key is a pure function of the pack)
 *   - whole get_matching result memoised per frame for keys whose rule list is STATIC: every
 *     condition in it is either frame-memoised (bit 11) or reads only SNAP/FRAME (types 0-2).
 *     Such a list gives the same answer at every pixel once its conditions have been seen,
 *     so the first call walks it for real (in the original order) and later calls replay it,
 *     including the fallback rewrite of the record's tile index.
 *     The static flag is a property of the pack; it is computed lazily and kept across
 *     iterations (a pack builder could store it), everything else is reset every iteration.
 *   - additions indexed by original-tile body, so a cache miss looks at matching additions only
 *   - the additions cache is reset by generation number instead of a 2 MB memset
 * Negative control MUT_MEMOALL memoises EVERY key: if parity still holds with it, these frames
 * do not exercise the static/non-static distinction and say nothing about it.
 *
 * Original header follows.
 *
 * hdarm.c - the stage-3 per-frame DECISION work in plain C, for the ARM-helper feasibility study
 * (OPEN_ITEMS item 11, 2026-10-21).
 *
 * Port of tools/hdref.py's decision path (build_records, select_backgrounds, process_additions,
 * get_matching with the BG 8-pixel cache and sprite draw order). Compositing is left out: it
 * stays in the FPGA in the ARM-helper design. Output must equal tools/armdec/hdref_decisions.py
 * byte for byte (same order, same format); that is the correctness check. Then it is timed on
 * the DE10-Nano's Cortex-A9.
 *
 * usage: hdarm PACK.hdp REC.bin WATCH.bin [--delta D] [--iters N] [--out FILE] [--quiet]
 *   --out   write the decision dump of the first iteration
 *   --iters repeat the whole decision pass N times (fresh state each time) and report timing
 */
#define _POSIX_C_SOURCE 200809L
/* 64-bit stat on 32-bit ARM: without it fstat fails with EOVERFLOW on filesystems whose inode
   numbers do not fit 32 bits (Docker bind mounts did; 2026-10-21) */
#define _FILE_OFFSET_BITS 64
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#if defined(__ARM_NEON)
#include <arm_neon.h>
#endif
#ifndef _WIN32
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#define NOTILE (-1LL)

/* ---------------------------------------------------------------- byte access (little-endian) */
static const uint8_t *D;   /* the .hdp */
/* little-endian loads (x86 and the A9 are both little-endian; memcpy compiles to one load) */
static inline uint32_t rd32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }
static inline uint64_t rd64(const uint8_t *p) { uint64_t v; memcpy(&v, p, 8); return v; }

/* Software prefetch INSIDE the streaming loops, one per 32-byte cache line, PFD bytes ahead.
   Measured on the A9 (2026-10-22, scratchpad stream_bench.c, pid_build's pattern over a 240 KB
   record after a 4 MB flush): no prefetch 1.74 ms, inline prefetch 64 B - 2 KB ahead 0.75 ms
   (core 0; core 1 3.6 -> 0.66 ms). A dependent DDR3 miss is ~250 ns there and the hardware does
   not prefetch these streams. Issuing a whole row of prefetches at the start of each row gave
   NOTHING (1.72 ms) - the A9's few line-fill buffers fill and the burst stalls - so prefetches
   are spread through the loop. They never change a result. -DPFD=0 turns them off. */
#ifndef PFD
#define PFD 256
#endif
#if PFD
#define PFI(p) __builtin_prefetch((const void *)(p))
#else
#define PFI(p) ((void)0)
#endif

/* ---------------------------------------------------------------- CRC-32 (MSB first, init 0) */
static uint32_t T32[256];
static void crc_init(void) {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t r = i << 24;
        for (int k = 0; k < 8; k++) r = (r & 0x80000000u) ? (r << 1) ^ 0x04C11DB7u : r << 1;
        T32[i] = r;
    }
}
static uint32_t crc32b(const uint8_t *d, int n) {
    uint32_t r = 0;
    for (int i = 0; i < n; i++) r = (r << 8) ^ T32[((r >> 24) ^ d[i]) & 0xFF];
    return r;
}

/* ---------------------------------------------------------------- pack header */
static uint32_t H_flags, H_options, nbuckets, max_probe, nconds, nwatched, nadd, nfallback;
static int32_t overscan[4];
static uint32_t SEC[18];
static int legacy, game_ram, cache_enabled;

/* ---------------------------------------------------------------- per-pixel records */
typedef struct {
    uint64_t fp;
    int32_t idx;
    uint32_t pal;
    uint32_t gid;     /* hdarm4: identity of the drawing entry (capture sprite, addition, BG span) */
    uint8_t fpv, offx, offy, hm, vm, bgp, sci, scolor;
    uint8_t rew;      /* hdarm4: index rewritten by a fallback walk this frame (set_idx) */
} Rec;
/* No per-pixel record array. A pixel's TILE record is built on demand from the capture's
   4-byte pixel entry (span slot, fine x) and a per-line table of span records; the only
   per-pixel state kept is (a) the sprite-block number, for pixels that have sprites, and
   (b) an overlay for the fallback rewrite of a tile's index, which the original makes
   persistent in its record. Pixel (0,0) is special: the additions pass copies each line's
   first pixel over it (hdref quirk), so it shows line P0L's first pixel. */
typedef struct { Rec r[4]; int64_t hd[4]; uint8_t n; } SBlk;   /* hd: decision, -3 = not drawn */
static uint8_t LPAL[240][32];     /* hdarm4: each line's palette (6-bit NES colours) */
static uint8_t LGE[240];          /* hdarm4: line grayscale (bit 0) + emphasis (bits 1-3) */

#ifdef CSTATS
long CST[16][2];
#endif
static const uint8_t *RECB;       /* the capture */
static Rec SPAN[240][35];         /* [y][slot]; slot 34 = no tile */
static uint8_t NSPAN[240], LFX[240];
static uint16_t LT[240];
static uint16_t *SBI;             /* per pixel: sprite block, 0 = none */
static SBlk *SB; static uint32_t SBn;
static uint8_t MUTB[61440 / 8]; static int32_t MUTI[61440];
static int ANY_MUT;               /* any overlay write this frame (MUTB is not all zero) */
static int P0L;
static int CUR_PIX = -1;          /* pixel whose tile record get_matching is working on */
#define SPANF (&SPAN[0][0])
/* per pixel: which SPAN record its tile is, as an index into SPANF (line * 35 + slot, the slot
   clamped to 34 = no tile as tile_rec does). Built once per frame after the records; pixel
   (0,0) follows P0L and is refreshed whenever P0L moves (pid_fix0). */
static uint16_t PID[61440 + 16];   /* +16: run_scan reads up to 8 entries past a line's end */
static void pid_fix0(void) {
    int s = RECB[P0L * 1024] & 63;
    PID[0] = (uint16_t)(P0L * 35 + (s >= NSPAN[P0L] ? 34 : s));
}
/* game-RAM packs: a record's CONTENT identity (see dep_id), interned to a small number so equal
   tiles on different spans and lines share it. Everything a walk reads from a tile record:
   palette, fingerprint (fp, fpv), index, and the flags conditions test (hm, vm, bgp). The
   position fields (offx, offy) are not identity - position conditions note them separately.
   build_records gives every span record fpv = 1 and hm = vm = bgp = 0, and slot 34 (no tile)
   the same all-zero record with idx NOTILE on every line, so the identity is (pal, idx, fp)
   for the spans and one fixed number (0) for slot 34. */
static uint32_t CIDG[240 * 35];
typedef struct { uint64_t fp; int32_t idx; uint32_t pal, id, gen; } CidE;
static CidE CIDH[16384];
static uint32_t CID_GEN;
static void cid_build(void) {
    uint32_t next = 1;                       /* 0 = the no-tile record */
    CID_GEN++;
    for (int y = 0; y < 240; y++) {
        const Rec *row = SPAN[y], *up = y ? SPAN[y - 1] : NULL;
        uint32_t *cid = CIDG + y * 35;
        int ns = NSPAN[y], nsu = y ? NSPAN[y - 1] : 0;
        cid[34] = 0;
        for (int s = 0; s < ns; s++) {
            const Rec *r = &row[s];
            /* the line above, same slot: the same tile on most lines of a tile row */
            if (s < nsu && r->fp == up[s].fp && r->idx == up[s].idx && r->pal == up[s].pal) {
                cid[s] = cid[s - 35];
                continue;
            }
            uint32_t lo = (uint32_t)r->fp, hi = (uint32_t)(r->fp >> 32);
            uint32_t h = (lo * 0x9E3779B1u) ^ (hi * 0x85EBCA6Bu) ^ (r->pal * 0xC2B2AE35u)
                       ^ ((uint32_t)r->idx * 0x27D4EB2Fu);
            h ^= h >> 15;
            for (h &= 16383;; h = (h + 1) & 16383) {
                CidE *e = &CIDH[h];
                if (e->gen != CID_GEN) {
                    e->gen = CID_GEN; e->fp = r->fp; e->idx = r->idx; e->pal = r->pal; e->id = next++;
                    cid[s] = e->id;
                    break;
                }
                if (e->fp == r->fp && e->idx == r->idx && e->pal == r->pal) { cid[s] = e->id; break; }
            }
        }
    }
}
static void pid_build(void) {
    for (int y = 0; y < 240; y++) {
        const uint8_t *b = RECB + y * 1024;
        int ns = NSPAN[y], base = y * 35;
        uint16_t *o = PID + y * 256;
        uint16_t tab[64];                 /* span slot -> record index, slot 34 = no tile */
        for (int s = 0; s < 64; s++) tab[s] = (uint16_t)(base + (s >= ns ? 34 : s));
        for (int x = 0; x < 256; x += 8) {   /* 8 pixels = one 32-byte line of the record */
            PFI(b + x * 4 + PFD);             /* past the record's last row: harmless */
            for (int k = 0; k < 8; k++) o[x + k] = tab[b[(x + k) * 4] & 63];
        }
    }
    pid_fix0();
    if (game_ram) cid_build();
}

static void tile_rec(int i, Rec *t) {
    int y = i >> 8, x = i & 255;
    if (i == 0) y = P0L;
    *t = SPANF[PID[i]];
    t->offx = (RECB[y * 1024 + x * 4 + 1] >> 2) & 7;
    if ((MUTB[i >> 3] >> (i & 7)) & 1) t->idx = MUTI[i];
}
static uint16_t line_t(int y) { return LT[y == 0 ? P0L : y]; }
static int nspr_of(int i) { return SBI[i] ? SB[SBI[i]].n : 0; }
static Rec *spr_of(int i, int k) { return &SB[SBI[i]].r[k]; }
static int16_t SPX0[240], SPX1[240];   /* hdarm4: per line, x range holding sprite pixels */
static void spx_note(int i) {
    int y = i >> 8, x = i & 255;
    if (x < SPX0[y]) SPX0[y] = (int16_t)x;
    if (x > SPX1[y]) SPX1[y] = (int16_t)x;
}
static Rec *spr_push(int i) {
    spx_note(i);
    if (!SBI[i]) { SBI[i] = (uint16_t)++SBn; SB[SBn].n = 0; }
    SBlk *b = &SB[SBI[i]];
    return &b->r[b->n++];
}
/* a change to the index of the tile record get_matching is working on, written through */
static int64_t TC_I = -1;        /* tile_at_matches' last-read cache key (see there) */
static void set_idx(Rec *r, int32_t v) {
    r->idx = v; r->rew = 1;
#ifndef MUT_TCSTALE   /* negative control: neighbour cache not cleared on a tile rewrite */
    TC_I = -1;
#endif
#ifdef MUT_NOWT   /* negative control: tile-index rewrite not written through to the overlay */
    return;
#endif
    if (CUR_PIX >= 0) { MUTB[CUR_PIX >> 3] |= (uint8_t)(1 << (CUR_PIX & 7)); MUTI[CUR_PIX] = v; ANY_MUT = 1; }
}
static uint32_t FRAME;
/* Feature-usage study (OPEN_ITEMS item 11): env HD_FEATOFF switches pack features off so the
   decision dump can be diffed against the exact one. An unsupported condition FAILS (its rule
   cannot match), which is what a renderer without the feature would do.
     1 tileAtPosition/spriteAtPosition (3,4)   2 tileNearby/spriteNearby (5,6)
     4 position conditions (7-10)              8 hmirror/vmirror/bgpriority (11-13)
    16 additions                              32 backgrounds
    64 memory / frame-range conditions (0-2, frame-level, the ARM's job either way)
   0 (the default) leaves the program exact; parity.sh runs with it unset. */
static unsigned FEATOFF;
static unsigned feat_bit(int t) {
    return t <= 2 ? 64 : t <= 4 ? 1 : t <= 6 ? 2 : t <= 10 ? 4 : 8;
}
static uint8_t SNAP[256];
static int8_t *MEMO;      /* frame-condition memo, -1 = not evaluated */
static long st_rules, st_conds, st_fb, st_add;

static void build_records(const uint8_t *rec) {
    RECB = rec;
    memset(SBI, 0, 61440 * sizeof(uint16_t));
    memset(MUTB, 0, sizeof MUTB); ANY_MUT = 0;
    for (int y = 0; y < 240; y++) { SPX0[y] = 256; SPX1[y] = -1; }
    TC_I = -1;
    P0L = 0;
    for (int y = 0; y < 240; y++) {
        const uint8_t *h = rec + 0x40000 + y * 2048;
        /* (prefetching the next line's header and SPAN row here was tried 2026-10-22: records
           -0.1 ms, below the A9's build-to-build layout noise. Not kept.) */
        uint64_t q0 = rd64(h);
        int nspan = (int)((q0 >> 32) & 0xFF); if (nspan > 34) nspan = 34;
        int nspr = (int)((q0 >> 48) & 0xFF); if (nspr > 64) nspr = 64;
        uint8_t pal[32];
        for (int n = 0; n < 32; n++) {
            int bit = 6 * n, by = bit >> 3, sh = bit & 7;
            uint32_t v = h[8 + by] | (by + 1 < 24 ? h[9 + by] << 8 : 0);
            pal[n] = (v >> sh) & 63;
        }
        memcpy(LPAL[y], pal, 32);
        uint32_t sp_lin[34], sp_psel[34]; uint64_t sp_fp[34];
        for (int s = 0; s < nspan; s++) {
            uint64_t qa = rd64(h + 64 + s * 16), qb = rd64(h + 72 + s * 16);
            sp_lin[s] = qa & 0x3FFFFF; sp_psel[s] = (qa >> 32) & 3; sp_fp[s] = qb;
        }
        int ns = 0;
        uint8_t o_attr[64], o_sx[64], o_p1[64], o_p2[64]; uint32_t o_lin[64]; uint64_t o_fp[64];
        for (int i = 0; i < nspr; i++) {
            uint64_t qa = rd64(h + 640 + i * 16), qb = rd64(h + 648 + i * 16);
            if (!((qa >> 54) & 1)) continue;
            o_attr[ns] = (qa >> 46) & 0xFF; o_sx[ns] = (qa >> 38) & 0xFF;
            o_lin[ns] = (qa >> 16) & 0x3FFFFF; o_p1[ns] = (qa >> 8) & 0xFF; o_p2[ns] = qa & 0xFF;
            o_fp[ns] = qb; ns++;
        }
        LT[y] = q0 & 0x7FFF; LFX[y] = (q0 >> 15) & 7;
        LGE[y] = (uint8_t)((q0 >> 18) & 15);
        NSPAN[y] = (uint8_t)nspan;
        /* slots 0..nspan-1 hold the line's spans; tile_rec maps every other slot value
           (63 = no span, or one past the line's spans) to slot 34, the no-tile record */
        for (int slot = 0; slot < nspan; slot++) {
            Rec *tl = &SPAN[y][slot];
            uint32_t lin = sp_lin[slot]; int pb = sp_psel[slot] * 4;
            tl->offx = 0; tl->hm = tl->vm = tl->bgp = 0; tl->sci = 0; tl->rew = 0;
            tl->idx = (lin & 0xFFFFF) >> 4; tl->offy = lin & 7;
            tl->pal = pal[pb + 3] | pal[pb + 2] << 8 | pal[pb + 1] << 16;
            if (!legacy) tl->pal |= (uint32_t)pal[0] << 24;
            tl->fpv = 1; tl->fp = game_ram ? sp_fp[slot] : 0;
            tl->gid = 0x40000000u + (uint32_t)(y * 64 + slot); tl->scolor = 0;
        }
        {
            Rec *tl = &SPAN[y][34];
            memset(tl, 0, sizeof *tl);
            tl->idx = NOTILE;
            tl->gid = 0x40000000u + (uint32_t)(y * 64 + 34);
        }
        if (!ns) continue;
        /* sprite i covers x = sx..sx+7; walking sprites in index order pushes each pixel's
           sprites in index order, as a per-pixel scan over all sprites would */
        for (int i = 0; i < ns; i++) {
            for (int c = 0; c < 8; c++) {
                int x = o_sx[i] + c;
                if (x > 255) break;
                const uint8_t *b = rec + y * 1024 + x * 4;
                int pi = y * 256 + x;
                if (!(b[0] & 0x40)) continue;
                if (nspr_of(pi) == 4) continue;
                {
                    Rec *s = spr_push(pi);
                    uint8_t a = o_attr[i];
                    s->idx = (o_lin[i] & 0xFFFFF) >> 4; s->offy = o_lin[i] & 7; s->offx = c;
                    s->hm = (a & 0x40) != 0; s->vm = (a & 0x80) != 0; s->bgp = (a & 0x20) != 0;
                    int q = 0x10 + 4 * (a & 3);
                    s->pal = pal[q + 3] | pal[q + 2] << 8 | pal[q + 1] << 16;
                    if (!legacy) s->pal |= 0xFF000000u;
                    s->fpv = 1; s->fp = game_ram ? o_fp[i] : 0;
                    s->sci = ((o_p1[i] >> c) & 1) | (((o_p2[i] >> c) & 1) << 1);
                    s->scolor = s->sci ? pal[q + s->sci] : b[2];
                    s->gid = 1u + (uint32_t)(y * 64 + i); s->rew = 0;
                }
            }
        }
    }
}

/* ---------------------------------------------------------------- pack lookups */
static int64_t fallback(int64_t idx) {
    if (idx >= 0 && idx < (int64_t)nfallback) return (int32_t)rd32(D + SEC[10] + 4 * idx);
    return -1;
}

static int64_t find_key(uint32_t pal, int kind, uint32_t lo, uint32_t hi) {
    uint8_t k[12];
    for (int i = 0; i < 4; i++) { k[i] = pal >> 8 * i; k[4 + i] = lo >> 8 * i; k[8 + i] = hi >> 8 * i; }
    uint32_t h = crc32b(k, 12) & (nbuckets - 1);
    for (uint32_t probe = 0; probe <= max_probe; probe++) {
        const uint8_t *b = D + SEC[0] + (uint64_t)((h + probe) & (nbuckets - 1)) * 64;
        for (int j = 0; j < 4; j++) {
            uint64_t q0 = rd64(b + j * 16), q1 = rd64(b + j * 16 + 8);
            if (!(q1 >> 63)) return -1;
            if ((uint32_t)q0 == pal && (uint32_t)(q0 >> 32) == lo && (uint32_t)q1 == hi
                && (int)((q1 >> 62) & 1) == kind)
                return (int64_t)((q1 >> 32) & 0x3FFFFFFF) * 8;
        }
    }
    return -1;
}

/* ---------------------------------------------------------------- per-frame memos */
static uint32_t G;        /* generation: bumped every iteration, so nothing carries between them */
static long st_fm_hit, st_gm_hit, st_gm_static, st_gm_dynamic;

/* 32-bit only: the A9 is a 32-bit core, and a 64x64 multiply costs several instructions */
static uint32_t mix(uint32_t pal, uint32_t lo, uint32_t hi, uint32_t k) {
    uint32_t h = lo * 0x9E3779B1u ^ hi * 0x85EBCA77u ^ pal * 0xC2B2AE3Du ^ k * 0x27D4EB2Fu;
    h ^= h >> 15; h *= 0x2C1B3C6Du; h ^= h >> 12;
    return h;
}

/* find_key memo: a pure function of the pack */
#define FM_N 16384
typedef struct { uint32_t pal, lo, hi, gen; int64_t off; uint8_t kind; } FM;
static FM *FMT; static uint32_t FMcount;
static int64_t find_key_m(uint32_t pal, int kind, uint32_t lo, uint32_t hi) {
    uint32_t h = mix(pal, lo, hi, kind);
    for (int i = 0; i < 32; i++) {
        FM *e = &FMT[(h + i) & (FM_N - 1)];
        if (e->gen != G) {
            int64_t off = find_key(pal, kind, lo, hi);
            if (FMcount < FM_N * 3 / 4) { e->gen = G; e->pal = pal; e->lo = lo; e->hi = hi; e->kind = kind; e->off = off; FMcount++; }
            return off;
        }
        if (e->pal == pal && e->lo == lo && e->hi == hi && e->kind == kind) { st_fm_hit++; return e->off; }
    }
    return find_key(pal, kind, lo, hi);
}

static int64_t find(uint32_t pal, const Rec *r) {
    if (game_ram) {
        if (!r->fpv) return -1;
        return find_key_m(pal, 1, (uint32_t)r->fp, (uint32_t)(r->fp >> 32));
    }
    if (r->idx == NOTILE) return -1;
    return find_key_m(pal, 0, (uint32_t)r->idx, 0);
}

/* is a rule list static? Pack property: kept across iterations */
#define CL_N 65536
typedef struct { uint32_t off; uint8_t used, flag; } CL;
static CL *CLT; static uint32_t CLcount;
static int clist_static(uint32_t off) {
    if (off == 0) return 1;
    const uint8_t *p = D + SEC[3] + off;
    for (;;) {
        uint32_t w = rd32(p); p += 4;
        uint64_t q0 = rd64(D + SEC[2] + 32 * (uint64_t)(w & 0xFFFFFF));
        if (!(((q0 >> 11) & 1) || (q0 & 0xFF) <= 2)) return 0;
        if (w >> 31) return 1;
    }
}
static int list_static(int64_t off) {
#ifdef MUT_MEMOALL   /* negative control: treat every list as static */
    (void)off; return 1;
#endif
    uint32_t h = (uint32_t)((uint64_t)off * 0x9E3779B97F4A7C15ull >> 40);
    for (int i = 0; i < 64; i++) {
        CL *e = &CLT[(h + i) & (CL_N - 1)];
        if (e->used && e->off == (uint32_t)off) return e->flag;
        if (!e->used) {
            const uint8_t *L = D + SEC[1] + off;
            uint32_t n = rd32(L); int s = 1;
            for (uint32_t k = 0; k < n && s; k++) {
                const uint8_t *w = L + 8 + 16 * k;
                s = clist_static(rd32(w + 4)) && clist_static(rd32(w + 8));
            }
            if (CLcount < CL_N * 3 / 4) { e->used = 1; e->off = (uint32_t)off; e->flag = s; CLcount++; }
            return s;
        }
    }
    return 0;
}

/* per-frame REDUCED rule lists. A condition of type 0-2 (memory, frame counter) has the same
   value at every pixel, so it can be evaluated once per frame, in any order. A rule that has
   such a condition evaluating false can never pass this frame and is dropped -- unless the rule
   also holds a frame-memoised condition of another type (bit 11, type > 2): that one takes its
   value from the FIRST pixel that evaluates it, so skipping the rule could move that first
   evaluation and change the memo. Dropped rules still OR their disableCache bit into the
   result, exactly as walking them would (dis_before for rules ahead of a kept one, tail for
   rules after the last). A kept rule with nothing pixel-dependent always passes, and ends the
   list. */
static int eval_cond(uint32_t ci, int x, int y, const Rec *rec);
/* run: index + 1 into RUNA when this entry starts a dispatch run (see dispatch_build), else 0;
   dacc: OR of the disableCache bits a walk collects from the run's first entry through this one */
typedef struct { uint32_t rule, run; uint8_t disb, always, ok, dacc; } RE;
typedef struct { uint32_t off, gen, start, n; uint8_t tail; } RL;
#define RL_N 16384
static RL *RLT; static RE *REA; static uint32_t REn, REcap; static long st_rl_kept, st_rl_dropped;

/* Per-frame dispatch over runs of "is tile K nearby" rules. A pack's rule list can hold long
   stretches of rules whose only condition that varies by pixel is one type-5 test of the SAME
   tile key K = (tile T, palette P, ignore-palette flag) at DIFFERENT neighbouring positions -
   "is T at +8, else at +16, else at +24 ..." (Super Mario Bros. 3's list of tile 329 runs 50
   positions deep). The walk tests them in order and stops at the first pass, so a run is still
   walked in order, but each test shrinks from a whole eval_cond + tile_at_matches to a lookup:
   whether K matches a tile record is a property of the record alone,
       (ignore || rec.pal == P) && rec.idx != NOTILE && (rec.idx == T || fallback(rec.idx) == T)
   so it is computed once per frame per key for every record (mk_flags) and read through PID.
   A pixel whose tile index was rewritten in the overlay (MUTB) is tested on MUTI instead, as
   tile_at_matches does. Every position walked still notes its dependency exactly as eval_cond
   does, and the disableCache bits are collected as the sequential walk would (dacc).
   Built in reduce(), so per frame; an entry qualifies only when its rule was kept with no
   failing frame condition (ok), it is not 'always', and its condition lists hold exactly one
   non-pure condition, that condition being type 5 with no negate, data or frame-memo bit.
   FEATOFF falls back to the sequential walk. */
#define NREC (240 * 35)
typedef struct { int64_t tile; uint32_t pal; uint8_t ign; uint32_t gen; } MKEY;
#define MK_MAX 64
static MKEY MK[MK_MAX]; static uint32_t MKn; static uint8_t *MKF;   /* MK_MAX x NREC flags */
typedef struct { uint32_t n, doff, key; uint8_t dall; } RUN;
static int cond_pure(uint64_t q0);
static RUN *RUNA; static uint32_t RUNn, RUNcap;
static int32_t *RDA; static uint32_t RDn, RDcap;   /* each run's position offsets, Y*256 + X */
static long st_disp, st_disp_q;
#define DISP_MIN 4      /* shorter runs are not worth it */
/* key k's per-record match flags for this frame, filled on first use of each record:
   0 = not yet known, 1 = no match, 2 = match (a full pass over all records cost more than it
   saved on packs that walk a run only a few times a frame) */
static uint8_t *mk_flags(uint32_t k) {
    MKEY *m = &MK[k];
    uint8_t *f = MKF + (size_t)k * NREC;
    if (m->gen != G) { m->gen = G; memset(f, 0, NREC); }
    return f;
}
static int mk_match(const MKEY *m, const Rec *s, int32_t idx) {
    return (m->ign || s->pal == m->pal) && idx != NOTILE && (idx == m->tile || fallback(idx) == m->tile);
}
/* the run-relevant shape of a kept rule: 1 when it qualifies (see above), with its test */
static int rule_t5(const uint8_t *w, int32_t *X, int32_t *Y, int64_t *tile, uint32_t *pal, int *ign) {
    int found = 0;
    for (int l = 0; l < 2; l++) {
        uint32_t off = rd32(w + 4 + 4 * l);
        if (off == 0) continue;
        const uint8_t *p = D + SEC[3] + off;
        for (;;) {
            uint32_t cw = rd32(p); p += 4;
            const uint8_t *c = D + SEC[2] + 32 * (uint64_t)(cw & 0xFFFFFF);
            uint64_t q0 = rd64(c);
            if (!cond_pure(q0)) {
                if (found || (q0 & 0xFF) != 5 || ((q0 >> 8) & 1) || ((q0 >> 10) & 1) || ((q0 >> 11) & 1))
                    return 0;
                uint64_t q1 = rd64(c + 8);
                *X = (int32_t)(uint32_t)q1; *Y = (int32_t)(uint32_t)(q1 >> 32);
                *tile = (int64_t)rd64(c + 16); *pal = (uint32_t)rd64(c + 24); *ign = (q0 >> 9) & 1;
                if (*ign) *pal = 0;
                found = 1;
            }
            if (cw >> 31) break;
        }
    }
    return found;
}
static void dispatch_build(const uint8_t *L, RL *slot) {
    uint32_t n = slot->n, i = 0;
    while (i < n) {
        RE *e0 = &REA[slot->start + i];
        int32_t X, Y; int64_t t0, tile; uint32_t p0, pal; int i0, ign;
        if (!e0->ok || e0->always || !rule_t5(L + 8 + 16 * e0->rule, &X, &Y, &t0, &p0, &i0)) { i++; continue; }
        st_disp_q++;
        uint32_t j = i + 1;
        while (j < n) {
            RE *e = &REA[slot->start + j];
            if (!e->ok || e->always || !rule_t5(L + 8 + 16 * e->rule, &X, &Y, &tile, &pal, &ign)
                || tile != t0 || pal != p0 || ign != i0) break;
            j++;
        }
        uint32_t m = j - i, k;
        for (k = 0; k < MKn; k++) if (MK[k].tile == t0 && MK[k].pal == p0 && MK[k].ign == i0) break;
        if (k == MKn && MKn < MK_MAX) {
            MK[MKn].tile = t0; MK[MKn].pal = p0; MK[MKn].ign = (uint8_t)i0; MK[MKn].gen = 0; MKn++;
        }
        if (m >= DISP_MIN && k < MKn) {
            if (RDn + m > RDcap) { RDcap = (RDn + m) * 2 + 4096; RDA = realloc(RDA, RDcap * sizeof(int32_t)); }
            if (RUNn == RUNcap) { RUNcap = RUNcap * 2 + 256; RUNA = realloc(RUNA, RUNcap * sizeof(RUN)); }
            RUN *R = &RUNA[RUNn];
            R->n = m; R->doff = RDn; R->key = k;
            uint8_t acc = 0;
            for (uint32_t q = 0; q < m; q++) {
                RE *e = &REA[slot->start + i + q];
                const uint8_t *w = L + 8 + 16 * e->rule;
                rule_t5(w, &X, &Y, &tile, &pal, &ign);
#ifdef MUT_NEARBY   /* negative control: nearby offset off by one pixel (as in eval_cond) */
                X += 1;
#endif
                RDA[RDn++] = Y * 256 + X;
                acc |= e->disb | ((rd32(w + 12) >> 24) & 1);
                e->dacc = acc;
            }
            R->dall = acc;
            REA[slot->start + i].run = ++RUNn;
        }
        i = j;
    }
}

static int cond_pure(uint64_t q0) { return (q0 & 0xFF) <= 2; }
/* 0 = clist fails on a pure condition, 1 = no pure failure; *dyn / *fdyn report what it holds */
static int clist_scan(uint32_t off, int *dyn, int *fdyn) {
    if (off == 0) return 1;
    const uint8_t *p = D + SEC[3] + off;
    int ok = 1;
    for (;;) {
        uint32_t w = rd32(p); p += 4;
        uint32_t ci = w & 0xFFFFFF;
        uint64_t q0 = rd64(D + SEC[2] + 32 * (uint64_t)ci);
        if (cond_pure(q0)) { if (ok && !eval_cond(ci, 0, 0, NULL)) ok = 0; }
        else { *dyn = 1; if ((q0 >> 11) & 1) *fdyn = 1; }
        if (w >> 31) return ok;
    }
}

static const RL *reduce(int64_t off) {
    uint32_t h = (uint32_t)((uint64_t)off * 0x9E3779B97F4A7C15ull >> 40);
    RL *slot = NULL;
    for (int i = 0; i < 64; i++) {
        RL *e = &RLT[(h + i) & (RL_N - 1)];
        if (e->gen != G) { slot = e; break; }
        if (e->off == (uint32_t)off) return e;
    }
    if (!slot) return NULL;
    slot->gen = G; slot->off = (uint32_t)off; slot->start = REn; slot->n = 0; slot->tail = 0;
    const uint8_t *L = D + SEC[1] + off;
    uint32_t n = rd32(L);
    uint8_t pend = 0;
    for (uint32_t k = 0; k < n; k++) {
        const uint8_t *w = L + 8 + 16 * k;
        uint8_t db = (rd32(w + 12) >> 24) & 1;
        int dyn = 0, fdyn = 0;
        int ok = clist_scan(rd32(w + 4), &dyn, &fdyn);
        ok = clist_scan(rd32(w + 8), &dyn, &fdyn) && ok;
        if (!ok && !fdyn) { pend |= db; st_rl_dropped++; continue; }
        if (REn == REcap) { REcap = REcap * 2 + 4096; REA = realloc(REA, REcap * sizeof(RE)); }
        REA[REn].rule = k; REA[REn].disb = pend; REA[REn].always = ok && !dyn;
        REA[REn].ok = (uint8_t)ok; REA[REn].run = 0; REA[REn].dacc = 0;
        REn++; slot->n++; pend = 0; st_rl_kept++;
        if (ok && !dyn) break;      /* passes at every pixel: later rules are never reached */
    }
    slot->tail = pend;
#ifndef NO_DISPATCH
    dispatch_build(L, slot);
#endif
    return slot;
}

/* get_matching memo: only entries whose list is static (or that found no list) */
#define GM_N 16384
typedef struct { uint32_t pal, lo, hi, gen; int64_t res; int32_t newidx; uint8_t kind, fpv, dis, mut; } GM;
static GM *GMT; static uint32_t GMcount;

/* ---------------------------------------------------------------- conditions */
static int op_eval(int op, uint64_t a, uint64_t b) {
    switch (op) {
    case 0: return a == b; case 1: return a != b; case 2: return a > b;
    case 3: return a < b; case 4: return a <= b; default: return a >= b;
    }
}

static inline int tmatch_f(uint64_t q0, uint64_t q2, uint64_t q3, uint32_t tpal, int32_t tidx,
                           uint64_t tfp, uint8_t tfpv) {
    uint32_t pal = (uint32_t)q3;
    int ignore = (q0 >> 9) & 1, data = (q0 >> 10) & 1;
    if (!data) {
        int64_t tile = (int64_t)q2;
        if (!(ignore || tpal == pal)) return 0;
        return tidx != NOTILE && (tidx == tile || fallback(tidx) == tile);
    }
    if (!tfpv || tidx == NOTILE) return 0;
    if (ignore) return tfp == q2;
    return tpal == pal && tfp == q2;
}
static int tmatch(uint64_t q0, uint64_t q2, uint64_t q3, const Rec *t) {
    return tmatch_f(q0, q2, q3, t->pal, t->idx, t->fp, t->fpv);
}

static int in_screen(int64_t i) { return i >= 0 && i < 61440; }

/* ---- dependency memo (hdarm4, speed only; results identical).
   A get_matching walk on a background pixel reads: the pixel's own tile record, the frame's
   memory/frame conditions, and the tile records at the pixels its tileAtPosition/tileNearby
   conditions point at. When the next pixel has the same tile record and every pointed-at
   pixel (shifted by one for nearby, fixed for absolute) has the same tile record, the walk
   would evaluate the same conditions to the same values and return the same answer. Sprite
   conditions are dependencies too (bit 1 of the kind, see spr_dep_id), and position
   conditions (x, y, fine x/y) are kept apart and re-evaluated for the asking pixel when what
   they read changed (DEP_P*, pos_hold). (A frame-wide table keyed by record CONTENT was tried
   2026-10-22 and was slower: neighbour combinations vary too much, see OPEN_ITEMS.) */
#define DEP_MAX 24
static int DEP_ON, DEP_N, DEP_BAD;
static int32_t DEP_T[DEP_MAX]; static uint8_t DEP_ABS[DEP_MAX]; static uint64_t DEP_ID[DEP_MAX];
/* position conditions (types 7-10) a walk read, kept apart so the common case - a walk that
   read none - costs nothing at reuse: the condition index, its kind (8 + type - 7) and the
   result. Within one walk x, y and the record are fixed, so one entry per condition. */
#define DEP_PMAX 8
static int DEP_PN;
static uint32_t DEP_PCI[DEP_PMAX]; static uint8_t DEP_PKD[DEP_PMAX], DEP_PV[DEP_PMAX];
/* a memo's copy of them, with the walked record's context (see rec_ctx) */
typedef struct { int n; uint32_t ctx; uint32_t ci[DEP_PMAX]; uint8_t kd[DEP_PMAX], v[DEP_PMAX]; } PosDeps;
static inline void pos_save(PosDeps *p, uint32_t ctx) {
    p->n = DEP_PN;
    if (DEP_PN) {
        p->ctx = ctx;
        memcpy(p->ci, DEP_PCI, sizeof(uint32_t) * DEP_PN);
        memcpy(p->kd, DEP_PKD, DEP_PN);
        memcpy(p->v, DEP_PV, DEP_PN);
    }
}
/* identity of the tile record tile_at_matches would read at linear pixel T: the record's index
   in SPAN (line * 35 + span slot, slot 34 = no tile), plus the overlay value when the pixel's
   index was rewritten. PID (built with the records, see pid_build) holds the first part. */
/* Without game RAM the identity is the record's CONTENT instead: everything tile_at_matches
   and a background walk read from a tile record is its palette and index (fp is 0, fpv is
   1 except on the no-tile record, whose index alone decides), so (palette, index) - the
   overlay's index when rewritten - is exact, and equal tiles on different spans and lines
   share it. Palettes are four 6-bit values, so no identity equals the off-screen ~0. */
static inline uint64_t tile_cid(int64_t T, int32_t idx) {
    return (uint64_t)SPANF[PID[T]].pal << 32 | (uint32_t)idx;
}
static inline __attribute__((always_inline)) uint64_t dep_id(int64_t T) {
    if (!in_screen(T)) return ~0ull;
    int m = ANY_MUT && ((MUTB[T >> 3] >> (T & 7)) & 1);
    if (!game_ram) return tile_cid(T, m ? MUTI[T] : SPANF[PID[T]].idx);
    if (!m) return CIDG[PID[T]];
    return PID[T] | 0x80000000ull | (uint64_t)(uint32_t)MUTI[T] << 32;
}
/* identity of the sprite records sprite_at_matches would read at linear pixel T. A record's
   match fields (pal, idx, fp, fpv) are fixed by its drawing entry for the whole frame - a
   capture sprite's are the same on every pixel of its row, an addition's come from the
   addition alone - so the entries' ids, in block order, identify them. Additions are complete
   before lookups; the only change during lookups is a fallback rewrite of a record's index
   (set_idx, flagged in rew), and a pixel holding one gets an id no other pixel shares. */
static inline uint64_t spr_dep_id(int64_t T) {
    if (!in_screen(T)) return ~0ull;
    uint16_t sb = SBI[T];
    if (!sb) return 0;
    const SBlk *b = &SB[sb];
    uint64_t id = 0;
    for (int k = 0; k < b->n; k++) {
        uint32_t g = b->r[k].gid;
        uint32_t c = g < 0x4000u ? g : (g & 0x80000000u) ? 0x4000u + (g & 0x7FFFFFFFu) : 0x10000u;
        if (b->r[k].rew || c >= 0x10000u) return (1ull << 63) | (uint64_t)T;
        id |= (uint64_t)c << (16 * k);
    }
    return id;
}
/* a noted dependency: bit 0 = absolute position (not shifted with the pixel),
   bit 1 = the sprites at the position (else its tile record) */
static inline uint64_t dep_id_k(int64_t T, int kind) { return (kind & 2) ? spr_dep_id(T) : dep_id(T); }
/* do a walk's noted dependencies still read the same at a pixel sh to the right? */
/* w = the pixel the walk was made at, w + sh the pixel asking. A pixel's records change only
   while that pixel is processed (its tile overlay in its background walk, its sprite records
   in its sprite walks), and pixels are processed in order, so a FIXED-position dependency
   outside w..w+sh reads now exactly what it read at the walk; only inside is it re-read. */
static inline __attribute__((always_inline)) int deps_hold(int n, const int32_t *T, const uint8_t *K, const uint64_t *ID,
                                                           int32_t w, int32_t sh, const int gr) {
#ifdef MUT_DEPLOOSE   /* negative control: reuse on the source record alone, ignoring deps */
    (void)n; (void)T; (void)K; (void)ID; (void)w; (void)sh; (void)gr; return 1;
#else
    int32_t r = w + sh;
    if (!ANY_MUT) {
        /* no tile index rewritten yet this frame: a tile id is its PID alone (and the ids
           noted at the walk were too, the overlay only ever grows) */
        for (int k = 0; k < n; k++) {
            int kd = K[k];
            int64_t t = T[k];
            if (kd & 1) {
                if (kd == 1 || t < w || t > r) continue;   /* fixed, and unchanged */
            } else t += sh;
            uint64_t id = (kd & 2) ? spr_dep_id(t)
                        : (uint64_t)t >= 61440 ? ~0ull
                        : gr ? CIDG[PID[t]] : tile_cid(t, SPANF[PID[t]].idx);
            if (id != ID[k]) return 0;
        }
        return 1;
    }
    for (int k = 0; k < n; k++) {
        int kd = K[k];
        int64_t t = T[k];
        if (kd & 1) { if (t < w || t > r) continue; } else t += sh;
        if (dep_id_k(t, kd) != ID[k]) return 0;
    }
    return 1;
#endif
}
/* The position conditions a walk read (DEP_P*), checked for the pixel r: one is re-evaluated
   only when what it reads differs between the walk (at w) and r - x (type 7), y (8), x / fine x
   / h-mirror (9), y / fine y / v-mirror (10). A record's context packs fine x | fine y << 8 |
   hm << 16 | vm << 24; ctx is the asking pixel's, wctx the walked record's. */
static inline __attribute__((always_inline)) uint32_t rec_ctx(int ox, int oy, int hm, int vm) {
    return (uint32_t)ox | (uint32_t)oy << 8 | (uint32_t)hm << 16 | (uint32_t)vm << 24;
}
static inline int cond_pos_ci(uint32_t ci, int x, int y, int offx, int offy, int hm, int vm);
static int pos_hold(int n, const uint32_t *ci, const uint8_t *kd, const uint8_t *v,
                    int32_t w, int32_t r, uint32_t ctx, uint32_t wctx) {
#ifdef MUT_DEPLOOSE
    (void)n; (void)ci; (void)kd; (void)v; (void)w; (void)r; (void)ctx; (void)wctx; return 1;
#else
    uint32_t dp = (uint32_t)(w ^ r), dc = ctx ^ wctx;
    for (int k = 0; k < n; k++) {
        int need;
        switch (kd[k]) {
        case 8:  need = (dp & 255) != 0; break;
        case 9:  need = (dp >> 8) != 0; break;
        case 10: need = (dp & 255) != 0 || (dc & 0x00FF00FFu) != 0; break;
        default: need = (dp >> 8) != 0 || (dc & 0xFF00FF00u) != 0; break;
        }
        if (need && cond_pos_ci(ci[k], r & 255, r >> 8, ctx & 255, (ctx >> 8) & 255,
                                (ctx >> 16) & 255, ctx >> 24) != v[k]) return 0;
    }
    return 1;
#endif
}
/* duplicate test for dep_note: a 64-slot table stamped with the walk's generation (DH_GEN,
   bumped where a walk starts), so a note costs a hash probe instead of a scan of DEP_T */
static uint32_t DH_GEN = 1;
static struct { uint32_t gen; int32_t t; uint8_t abs; } DH[64];
/* a new dependency's slot in DEP_*, or -1 when not noting, already noted, or out of room */
static inline __attribute__((always_inline)) int dep_slot(int64_t T, int kind) {
    if (!DEP_ON || DEP_BAD) return -1;   /* a walk that cannot be reused needs no more notes */
    if (T < INT32_MIN || T > INT32_MAX) { DEP_BAD = 1; return -1; }
    uint32_t h = ((uint32_t)T * 2654435761u ^ (uint32_t)kind * 0x9E3779B9u) >> 26;
    for (;; h = (h + 1) & 63) {
        if (DH[h].gen != DH_GEN) break;
        if (DH[h].t == (int32_t)T && DH[h].abs == kind) return -1;
    }
    if (DEP_N == DEP_MAX) { DEP_BAD = 1; return -1; }
    DH[h].gen = DH_GEN; DH[h].t = (int32_t)T; DH[h].abs = (uint8_t)kind;
    DEP_T[DEP_N] = (int32_t)T; DEP_ABS[DEP_N] = (uint8_t)kind;
    return DEP_N++;
}
static void dep_note(int64_t T, int kind) {
    int i = dep_slot(T, kind);
    if (i >= 0) DEP_ID[i] = dep_id_k(T, kind);
}
/* a position condition (index ci, kind 8 + type - 7) and the result it gave */
static void dep_note_pos(uint32_t ci, int kind, int r) {
    if (!DEP_ON || DEP_BAD) return;
    for (int i = 0; i < DEP_PN; i++) if (DEP_PCI[i] == ci) return;
    if (DEP_PN == DEP_PMAX) { DEP_BAD = 1; return; }
    DEP_PCI[DEP_PN] = ci; DEP_PKD[DEP_PN] = (uint8_t)kind; DEP_PV[DEP_PN] = (uint8_t)r; DEP_PN++;
}
/* a position condition (types 7-10) at pixel x, y for a record with fine x / fine y and
   mirroring (offx < 0: no record), negate bit and the feature switch applied */
static inline int cond_pos(uint64_t q0, uint64_t q1, int x, int y, int offx, int offy, int hm, int vm) {
    int t = q0 & 0xFF, op = (q0 >> 12) & 0xF;
    int64_t val;
    if (t == 7) val = x;
    else if (t == 8) val = y;
    else if (offx < 0) val = 0;
    else if (t == 9) val = hm ? x - (7 - offx) : x - offx;
    else val = vm ? y - (7 - offy) : y - offy;
    int r = op_eval(op, (uint32_t)val, (uint32_t)q1);
#ifndef MUT_NEGATE   /* negative control: ignore the negate bit */
    if ((q0 >> 8) & 1) r = !r;
#endif
    if (FEATOFF && (FEATOFF & feat_bit(t))) r = 0;
    return r;
}
static inline int cond_pos_ci(uint32_t ci, int x, int y, int offx, int offy, int hm, int vm) {
    const uint8_t *c = D + SEC[2] + 32 * (uint64_t)ci;
    return cond_pos(rd64(c), rd64(c + 8), x, y, offx, offy, hm, vm);
}
/* tile_rec's fields read in place (no record copy): the tile at pixel i matches? */
/* last tile read: a rule list usually tests one neighbour against many tiles. Valid while
   the frame's records and overlay are unchanged: cleared per frame (TC_GEN) and on every
   overlay write (set_idx). */
static const Rec *TC_S; static int32_t TC_IDX; static int64_t TC_FB; static int TC_FBOK;
static int tile_at_matches(uint64_t q0, uint64_t q2, uint64_t q3, int64_t i) {
    if (!in_screen(i)) return 0;
    if (i != TC_I) {
        TC_S = SPANF + PID[i];
        TC_IDX = (ANY_MUT && ((MUTB[i >> 3] >> (i & 7)) & 1)) ? MUTI[i] : TC_S->idx;
        TC_I = i; TC_FBOK = 0;
    }
    const Rec *s = TC_S;
    int32_t idx = TC_IDX;
    uint32_t pal = (uint32_t)q3;
    int ignore = (q0 >> 9) & 1, data = (q0 >> 10) & 1;
    if (!data) {
        int64_t tile = (int64_t)q2;
        if (!(ignore || s->pal == pal)) return 0;
        if (idx == NOTILE) return 0;
        if (idx == tile) return 1;
        if (!TC_FBOK) { TC_FB = fallback(idx); TC_FBOK = 1; }
        return TC_FB == tile;
    }
    return tmatch_f(q0, q2, q3, s->pal, idx, s->fp, s->fpv);
}
/* the first passing rule of a dispatch run at pixel x, y (run position), or -1. Walks the
   run's positions in rule order like the sequential walk, noting each position's dependency as
   eval_cond does, and tests the tile there as tile_at_matches does (records through the
   per-frame flags, an overlay-rewritten pixel on MUTI). */
/* the dependency kind dispatch_find notes (0, as eval_cond does for type 5). A variable rather
   than a literal: a literal 0 made GCC clone dep_note for it, and that clone moved the inlining
   and register allocation of the per-pixel loop by ~0.3M ARM instructions a frame */
static int disp_dep_kind;
/* (kept out of line: inlined into the per-pixel loop it cost that loop more than it saved) */
static __attribute__((noinline)) int dispatch_find(const RUN *R, int x, int y) {
    uint8_t *f = mk_flags(R->key);
    const MKEY *m = &MK[R->key];
    const int32_t *d = RDA + R->doff;
    int64_t base = (int64_t)y * 256 + x;
    for (uint32_t k = 0; k < R->n; k++) {
        int64_t i = base + d[k];
#ifndef MUT_DISPNODEP   /* negative control: the dispatch notes no dependencies */
        dep_note(i, disp_dep_kind);
#endif
        if (!in_screen(i)) continue;
        int hit;
        uint16_t pid = PID[i];
        if (ANY_MUT && ((MUTB[i >> 3] >> (i & 7)) & 1))
            hit = mk_match(m, SPANF + pid, MUTI[i]);
        else {
            if (!f[pid]) f[pid] = 1 + (uint8_t)mk_match(m, SPANF + pid, SPANF[pid].idx);
            hit = f[pid] == 2;
        }
        if (hit) return (int)k;
    }
    return -1;
}
static int sprite_at_matches(uint64_t q0, uint64_t q2, uint64_t q3, int64_t i) {
    if (!in_screen(i)) return 0;
    int n = nspr_of((int)i);
    for (int k = 0; k < n; k++) if (tmatch(q0, q2, q3, spr_of((int)i, k))) return 1;
    return 0;
}

static int eval_cond(uint32_t ci, int x, int y, const Rec *rec) {
    const uint8_t *c = D + SEC[2] + 32 * (uint64_t)ci;
    uint64_t q0 = rd64(c), q1 = rd64(c + 8), q2 = rd64(c + 16), q3 = rd64(c + 24);
    int frame = (q0 >> 11) & 1;
    st_conds++;
#ifdef CSTATS   /* per-type evaluation counts: [type][frame-memoised hit?] */
    { extern long CST[16][2]; CST[q0 & 15][frame && MEMO[ci] >= 0]++; }
#endif
    if (frame && MEMO[ci] >= 0) return MEMO[ci];
    int t = q0 & 0xFF, op = (q0 >> 12) & 0xF, r = 0;
    uint32_t mask = (q0 >> 16) & 0xFF;
    switch (t) {
    case 0: r = op_eval(op, SNAP[q1 & 0xFF] & mask, SNAP[(q1 >> 8) & 0xFF] & mask); break;
    case 1: r = op_eval(op, SNAP[q1 & 0xFF] & mask, (q1 >> 8) & 0xFF); break;
    case 2: { uint32_t a = q1 & 0xFFFF, b = (q1 >> 16) & 0xFFFF; r = a != 0 && (FRAME % a) >= b; break; }
    case 3: dep_note((int64_t)(q3 >> 32), 1); r = tile_at_matches(q0, q2, q3, (int64_t)(q3 >> 32)); break;
    case 4:   /* fixed position, but a sprite walk there may rewrite its record mid-frame */
        dep_note((int64_t)(q3 >> 32), 3); r = sprite_at_matches(q0, q2, q3, (int64_t)(q3 >> 32)); break;
    case 5: case 6: {
        int64_t X = (int32_t)(uint32_t)q1, Y = (int32_t)(uint32_t)(q1 >> 32);
#ifdef MUT_NEARBY   /* negative control: nearby offset off by one pixel */
        X += 1;
#endif
        if (t == 5) {
            dep_note((y + Y) * 256 + x + X, 0);
            r = tile_at_matches(q0, q2, q3, (y + Y) * 256 + x + X);
        } else {
            int64_t xs = (rec && rec->hm) ? -1 : 1, ys = (rec && rec->vm) ? -1 : 1;
            /* a mirrored record flips the offset, not the motion: the position still moves
               one pixel with the walking pixel, and a reuse only ever pairs records of one
               drawing entry, so with the same mirroring */
            dep_note((y + Y * ys) * 256 + x + X * xs, 2);
            r = sprite_at_matches(q0, q2, q3, (y + Y * ys) * 256 + x + X * xs);
        }
        break;
    }
    case 7: case 8: case 9: case 10:
        /* position conditions: the memos keep the RESULT and re-evaluate the condition at a
           pixel they would answer for, so the walk is reused only where it reads the same */
        r = rec ? cond_pos(q0, q1, x, y, rec->offx, rec->offy, rec->hm, rec->vm)
                : cond_pos(q0, q1, x, y, -1, -1, 0, 0);
        if (!frame) dep_note_pos(ci, 8 + (t - 7), r);
        if (frame) MEMO[ci] = (int8_t)r;
        return r;
    case 11: r = rec && rec->hm; break;
    case 12: r = rec && rec->vm; break;
    case 13: r = rec && rec->bgp; break;
    default: fprintf(stderr, "condition type %d\n", t); exit(2);
    }
#ifndef MUT_NEGATE   /* negative control: ignore the negate bit */
    if ((q0 >> 8) & 1) r = !r;
#endif
    if (FEATOFF && (FEATOFF & feat_bit(t))) r = 0;
    if (frame) MEMO[ci] = (int8_t)r;
    return r;
}

/* walk a condition list (offset into section 3); 1 if all pass */
static int clist_pass(uint32_t off, int x, int y, const Rec *rec) {
    if (off == 0) return 1;
    const uint8_t *p = D + SEC[3] + off;
    for (;;) {
        uint32_t w = rd32(p); p += 4;
        if (!eval_cond(w & 0xFFFFFF, x, y, rec)) return 0;
        if (w >> 31) return 1;
    }
}

static int64_t get_matching_walk(int x, int y, Rec *r, int *dis, int *is_static);

/* returns payload, or -1 for no HD tile; *dis = disableCache */
static int64_t get_matching(int x, int y, Rec *r, int *dis) {
    uint32_t lo, hi; uint8_t kind = (uint8_t)game_ram;
    if (game_ram) { lo = (uint32_t)r->fp; hi = (uint32_t)(r->fp >> 32); }
    else { lo = (uint32_t)r->idx; hi = 0; }
    /* neighbouring pixels of one tile ask the same question: check the last memo hit first */
    static GM *last;
    if (last && last->gen == G && last->pal == r->pal && last->lo == lo && last->hi == hi
        && last->kind == kind && last->fpv == r->fpv) {
        st_gm_hit++;
        if (last->mut) set_idx(r, last->newidx);
        *dis = last->dis;
        return last->res;
    }
    uint32_t h = mix(r->pal, lo, hi, kind | (uint32_t)r->fpv << 1);
    GM *slot = NULL;
    for (int i = 0; i < 32; i++) {
        GM *e = &GMT[(h + i) & (GM_N - 1)];
        if (e->gen != G) { slot = e; break; }
        if (e->pal == r->pal && e->lo == lo && e->hi == hi && e->kind == kind && e->fpv == r->fpv) {
            st_gm_hit++;
            last = e;
            if (e->mut) set_idx(r, e->newidx);
            *dis = e->dis;
            return e->res;
        }
    }
    int32_t org = r->idx; int st;
    int64_t res = get_matching_walk(x, y, r, dis, &st);
    if (st) st_gm_static++; else st_gm_dynamic++;
    if (st && slot && GMcount < GM_N * 3 / 4) {
        slot->gen = G; slot->pal = r->pal; slot->lo = lo; slot->hi = hi; slot->kind = kind; slot->fpv = r->fpv;
        slot->res = res; slot->dis = (uint8_t)*dis; slot->mut = r->idx != org; slot->newidx = r->idx;
        GMcount++;
        last = slot;
    }
    return res;
}

/* the original get_matching body; *is_static = 1 if the answer is the same at every pixel */
static int64_t get_matching_walk(int x, int y, Rec *r, int *dis, int *is_static) {
    *is_static = 1;
    int64_t off = find(r->pal, r);
    if (off < 0) {
        int64_t f = game_ram ? -1 : fallback(r->idx);
        if (f >= 0) {
            int64_t org = r->idx;
            set_idx(r, (int32_t)f);
            off = find(r->pal, r);
            if (off < 0) off = find(0xFFFFFFFFu, r);
            if (off < 0) set_idx(r, (int32_t)org); else st_fb++;
        }
        if (off < 0) off = find(0xFFFFFFFFu, r);
    }
    *dis = 0;
    if (off < 0) return -1;
    *is_static = list_static(off);
    const uint8_t *L = D + SEC[1] + off;
#ifndef NO_REDUCE
    const RL *rl = reduce(off);
    if (rl) {
        if (rl->n == 0 || REA[rl->start].always) *is_static = 1;
        for (uint32_t i = 0; i < rl->n; i++) {
            const RE *e = &REA[rl->start + i];
#ifndef NO_DISPATCH
            if (e->run && !FEATOFF) {   /* a dispatch run: one lookup for the whole stretch */
                const RUN *R = &RUNA[e->run - 1];
                int p = dispatch_find(R, x, y);
                st_disp++;
                if (p >= 0) {
                    const RE *h = &REA[rl->start + i + (uint32_t)p];
                    if (h->dacc) *dis = 1;
                    return rd32(L + 8 + 16 * h->rule);
                }
                if (R->dall) *dis = 1;
                i += R->n - 1;
                continue;
            }
#endif
            const uint8_t *w = L + 8 + 16 * e->rule;
            st_rules++;
            if (e->disb) *dis = 1;
            if ((rd32(w + 12) >> 24) & 1) *dis = 1;
            if (e->always || (clist_pass(rd32(w + 4), x, y, r) && clist_pass(rd32(w + 8), x, y, r)))
                return rd32(w);
        }
        if (rl->tail) *dis = 1;
        return -1;
    }
#endif
    uint32_t n = rd32(L);
    for (uint32_t i = 0; i < n; i++) {
        const uint8_t *w = L + 8 + 16 * i;
        st_rules++;
        if ((rd32(w + 12) >> 24) & 1) *dis = 1;
        if (clist_pass(rd32(w + 4), x, y, r) && clist_pass(rd32(w + 8), x, y, r))
            return rd32(w);
    }
    return -1;
}

/* ---------------------------------------------------------------- backgrounds */
static int cmp_u32(const void *a, const void *b) {
    uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b; return x < y ? -1 : x > y;
}
static int bg_candidates(uint32_t ioff, uint32_t *out, int max) {
    const uint8_t *base = D + SEC[13] + ioff;
    uint32_t nsig = rd32(base); int n = 0;
    for (uint32_t k = 0; k < nsig; k++) {
        const uint8_t *p = base + 4 + 48 * k;
        int nterms = p[0];
        uint8_t vals[16] = {0};
        for (int j = 0; j < nterms; j++) vals[j] = SNAP[p[2 + 2 * j]] & p[3 + 2 * j];
        uint32_t toff = rd32(p + 34), nb = rd32(p + 38), maxp = rd32(p + 42);
        uint32_t h = crc32b(vals, 16) & (nb - 1);
        for (uint32_t probe = 0; probe <= maxp; probe++) {
            const uint8_t *e = D + SEC[13] + toff + 32 * (uint64_t)((h + probe) & (nb - 1));
            uint32_t lst = rd32(e + 16), cnt = rd32(e + 20), valid = rd32(e + 24);
            if (!valid) break;
            if (!memcmp(e, vals, 16)) {
                for (uint32_t i = 0; i < cnt && n < max; i++) out[n++] = rd32(D + SEC[13] + lst + 4 * i);
                break;
            }
        }
    }
    qsort(out, n, 4, cmp_u32);
    int u = 0;
    for (int i = 0; i < n; i++) if (!u || out[u - 1] != out[i]) out[u++] = out[i];
    return u;
}

/* ---------------------------------------------------------------- output */
static char *OB; static size_t OBn, OBcap;
#if defined(HDARM_LIB) || defined(HD_NOOUT)
#define OUT_ON 0                  /* the library never dumps decisions: the tests compile away
                                     (HD_NOOUT: the same for a timing build of the tool) */
#else
static int OUT_ON;
#endif
#include <stdarg.h>
static void outf_(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
/* the decision dump costs a call per pixel even when off; test the flag at the call site */
#define outf(...) do { if (OUT_ON) outf_(__VA_ARGS__); } while (0)
static void outf_(const char *fmt, ...) {
    if (OBcap - OBn < 512) { OBcap = OBcap * 2 + 65536; OB = realloc(OB, OBcap); }
    va_list ap; va_start(ap, fmt);
    OBn += vsnprintf(OB + OBn, OBcap - OBn, fmt, ap);
    va_end(ap);
}

static uint32_t LAY[4][10]; static int NLAY[4];   /* hdarm4: chosen background records per layer */
static int select_levels(void) {
    static uint32_t cand[65536];
    int hits = 0;
    NLAY[0] = NLAY[1] = NLAY[2] = NLAY[3] = 0;
    if (FEATOFF & 32) return 0;
    for (int lvl = 0; lvl < 40; lvl++) {
        uint32_t roff = rd32(D + SEC[5] + lvl * 8), ioff = rd32(D + SEC[5] + lvl * 8 + 4);
        if (roff == 0) continue;
        const uint8_t *L = D + SEC[1] + roff;
        uint32_t n = rd32(L);
        int nc = ioff ? bg_candidates(ioff, cand, 65536) : (int)n;
        for (int i = 0; i < nc; i++) {
            uint32_t e = ioff ? cand[i] : (uint32_t)i;
            const uint8_t *w = L + 8 + 16 * e;
            if (clist_pass(rd32(w + 4), 0, 0, NULL)) {
                outf("L %d %x\n", lvl, rd32(w));
                LAY[lvl / 10][NLAY[lvl / 10]++] = rd32(w);
                hits++;
                break;
            }
        }
    }
    return hits;
}

/* ---------------------------------------------------------------- additions */
typedef struct { uint32_t opal, apal, ignore; int64_t oidx, aidx; uint64_t ofp, afp; int32_t ox, oy; } Add;
static Add *ADDS;
static int has_fb;
/* additions_for cache: open addressing on (pal, key, keytype) */
typedef struct { uint32_t pal, gen; uint8_t isnone; uint64_t key; int start, n; } AC;
static AC *ACT; static int ACTn; static int *ACL; static int ACLn, ACLcap;

static void add_cache_reset(void) {
    if (!ACT) { ACTn = 1 << 16; ACT = calloc(ACTn, sizeof(AC)); }
    ACLn = 0;   /* entries are live only when gen == G */
}

/* additions sorted by original body (tile index, or fingerprint for CHR-RAM games), then by
   index: every addition additions_for can take has body == the record's body or, through a
   fallback, == fallback(record's tile) */
typedef struct { uint64_t body; uint32_t a; } AX;
static AX *AXT;
static int cmp_ax(const void *p, const void *q) {
    const AX *a = p, *b = q;
    if (a->body != b->body) return a->body < b->body ? -1 : 1;
    return a->a < b->a ? -1 : a->a > b->a;
}
static void ax_range(uint64_t body, uint32_t *lo, uint32_t *hi) {
    uint32_t l = 0, h = nadd;
    while (l < h) { uint32_t m = (l + h) / 2; if (AXT[m].body < body) l = m + 1; else h = m; }
    *lo = l; h = nadd;
    while (l < h) { uint32_t m = (l + h) / 2; if (AXT[m].body <= body) l = m + 1; else h = m; }
    *hi = l;
}

static int key_eq(const Rec *r, uint32_t pal, int64_t idx, uint64_t fp) {
    if (game_ram) return r->fpv && r->pal == pal && r->fp == fp;
    return r->idx == idx && r->pal == pal;
}
static int body_eq(const Rec *r, int64_t idx, uint64_t fp) {
    if (game_ram) return r->fpv && r->fp == fp;
    return r->idx == idx;
}

static const int *additions_for(const Rec *r, int *cnt) {
    uint64_t key = game_ram ? r->fp : (uint64_t)(int64_t)r->idx;
    uint8_t isnone = game_ram && !r->fpv;
    /* (answering repeat keys from the last entry found was tried 2026-10-22: additions -0.1 ms,
       below the A9's build-to-build layout noise. Not kept.) */
    uint32_t h = (uint32_t)(key * 0x9E3779B97F4A7C15ull >> 40) ^ r->pal * 2654435761u;
    for (int i = 0;; i++) {
        AC *e = &ACT[(h + i) & (ACTn - 1)];
        if (e->gen != G) {
            e->gen = G; e->pal = r->pal; e->key = key; e->isnone = isnone; e->start = ACLn; e->n = 0;
            int check_fb = !game_ram && has_fb;
            /* candidates: body == key, plus body == fallback(key); merged in index order */
            uint32_t l1, h1, l2 = 0, h2 = 0;
            ax_range(key, &l1, &h1);
            if (check_fb && r->idx != NOTILE) {
                int64_t f = fallback(r->idx);
                if ((uint64_t)f != key) ax_range((uint64_t)f, &l2, &h2);
            }
            while (l1 < h1 || l2 < h2) {
                uint32_t a;
                if (l2 >= h2 || (l1 < h1 && AXT[l1].a < AXT[l2].a)) a = AXT[l1++].a; else a = AXT[l2++].a;
                const Add *A = &ADDS[a];
                int take = 0;
                if (!A->ignore && key_eq(r, A->opal, A->oidx, A->ofp)) take = 1;
                else if (A->ignore && body_eq(r, A->oidx, A->ofp)) take = 1;
                else if (check_fb && (A->ignore || r->pal == A->opal) && r->idx != NOTILE
                         && fallback(r->idx) == A->oidx) take = 1;
                if (take) {
                    if (ACLn == ACLcap) { ACLcap = ACLcap * 2 + 1024; ACL = realloc(ACL, ACLcap * sizeof(int)); }
                    ACL[ACLn++] = a; e->n++;
                }
            }
            *cnt = e->n; return ACL + e->start;
        }
        if (e->pal == r->pal && e->key == key && e->isnone == isnone) { *cnt = e->n; return ACL + e->start; }
        if (i >= ACTn) { fprintf(stderr, "additions cache full\n"); exit(2); }
    }
}

/* hdarm4: an addition entry's identity = (parent entry, addition index), so the entries one
   parent row places on neighbouring pixels form one span in the display list */
#define AG_N 65536
typedef struct { uint32_t parent, a, gen, id; } AG;
static AG *AGT; static uint32_t AGnext;
static uint32_t add_gid(uint32_t parent, uint32_t a) {
    uint32_t h = (parent * 0x9E3779B1u) ^ (a * 0x85EBCA77u);
    h ^= h >> 15;
    for (uint32_t i = 0; i < AG_N; i++) {
        AG *e = &AGT[(h + i) & (AG_N - 1)];
        if (e->gen != G) { e->gen = G; e->parent = parent; e->a = a; e->id = 0x80000000u + AGnext++; return e->id; }
        if (e->parent == parent && e->a == a) return e->id;
    }
    fprintf(stderr, "addition id table full\n"); exit(2);
}

static int draw_additions(int x, int y, const Rec *r) {
    int n; const int *lst = additions_for(r, &n);
    for (int i = 0; i < n; i++) {
        const Add *A = &ADDS[lst[i]];
        int tx = x + (r->hm ? -A->ox : A->ox), ty = y + (r->vm ? -A->oy : A->oy);
        if (tx >= 0 && tx < 256 && ty >= 0 && ty < 240) {
            int pi = (ty << 8) | tx;
            if (nspr_of(pi) < 4) {
                Rec nr = *r;
                nr.sci = 0; nr.idx = (int32_t)A->aidx; nr.pal = A->apal; nr.fp = A->afp; nr.fpv = 1;
                nr.gid = add_gid(r->gid, (uint32_t)lst[i]); nr.rew = 0;
                *spr_push(pi) = nr;
                st_add++;
            }
        }
    }
    return n > 0;
}

static int scroll_y(uint16_t t) { return (((t & 0x3E0) >> 2) | ((t & 0x7000) >> 12)) + ((t & 0x800) ? 240 : 0); }

static void process_additions(void) {
#ifdef MUT_NOADD   /* negative control: no additions */
    return;
#endif
    if (!nadd || (FEATOFF & 16)) return;
    int yscroll = scroll_y(line_t(0)); uint16_t tmpv = line_t(0); int process_bg = 1;
    for (int y = 0; y < 240; y++) {
        if (y) {
            /* the original copies the whole pixel over pixel (0,0), sprites included: point
               (0,0) at line y, and give it its own copy of that pixel's sprite block, so a later
               addition to (0,0) cannot change line y's first pixel through an alias. No tile
               rewrite can exist yet (those happen in lookups), so the overlay needs no copy. */
#ifndef MUT_NOP0   /* negative control: (0,0) keeps showing line 0 */
            P0L = y;
            pid_fix0();
#endif
            TC_I = -1;
            int src = y << 8;
            SBI[0] = 0;
            if (SBI[src]) { SBI[0] = (uint16_t)++SBn; SB[SBn] = SB[SBI[src]]; spx_note(0); }
        }
        uint16_t lt = line_t(0);
        if (((yscroll + y) & 7) == 0 || tmpv != lt) { yscroll = scroll_y(lt); tmpv = lt; process_bg = 1; }
        int dobg = process_bg;
        if (process_bg) process_bg = 0;
        /* a background pixel whose tile record is the same SPAN record as the previous pixel's
           has the same additions list (no index is rewritten before lookups), so when that
           list was empty the pixel adds nothing and is skipped */
        int lpid = -1, lnone = 0;
        const uint16_t *sbl = SBI + (y << 8);
        for (int x = 0; x < 256; x++) {
            int pi = (y << 8) | x;
            if (!dobg) {
                /* no background pass on this line: only pixels holding sprites matter. Four
                   empty slots are skipped at once; nothing between the read and the skip can
                   add a sprite (draw_additions only runs on a pixel that has one). */
                if (!(x & 3)) { uint64_t w; memcpy(&w, sbl + x, 8); if (!w) { x += 3; continue; } }
                if (!sbl[x]) continue;
            }
            for (int i = 0; i < nspr_of(pi); i++) { Rec r = *spr_of(pi, i); draw_additions(x, y, &r); }
            if (dobg) {
                int pid = PID[pi];
                if (pid == lpid && lnone) continue;
                Rec t; tile_rec(pi, &t);
                int any = draw_additions(x, y, &t);
                if (any) process_bg = 1;
                lpid = pid; lnone = !any;
            }
        }
    }
}

/* ---------------------------------------------------------------- the decision pass */
static void outv(int64_t v) { if (v < 0) outf(" -"); else outf(" %llx", (unsigned long long)v); }

/* hdarm4: the CURRENT line's backdrop and BG-tile spans, merged into runs while lookups makes
   the line's decisions (each pixel's BG decision is final once made: later rewrites only
   change later pixels). gen_line, called right after the line, emits them in the display
   list's order. A backdrop run: one colour. A BG run: a BLIT (k 0, v = block) while the block
   and tile row match and offx steps by one, or a FILL (k 1, v = rgb, the original tile colour
   when the decision falls back) while the colour matches - the same merge rule gen_line used
   when it swept the line a second time. */
typedef struct { uint16_t x, len; uint32_t v; } BDRun;
typedef struct { uint16_t x, len; uint8_t k, ofy, ofx; uint32_t v; } BGRun;
static BDRun BDR[257]; static int nBDR;
static BGRun BGR[257]; static int nBGR;
static void gen_line(int y, int L, int R);
static uint32_t PRGB[64];         /* (tentative; defined with the display list below) */
static int render_original;

#ifdef TPROF   /* cycle profile of lookups(): [0] tile_rec, [1] BG get_matching, [2] sprites, [3] BG calls, [4] BG cached */
#include <x86intrin.h>
long long TP[8];
#define TPS(v) unsigned long long v = __rdtsc()
#define TPA(i, v) TP[i] += (long long)(__rdtsc() - v)
#else
#define TPS(v)
#define TPA(i, v)
#endif

static int R_ok, R_n, R_dis, R_mut; static int32_t R_pi, R_newidx; static int64_t R_hd; static PosDeps R_P;
static int R_hc; static uint32_t R_hv;   /* R_hd's class and value (hd_class), kept so a reuse needs no 64-bit work */
static uint64_t R_src; static int32_t R_T[DEP_MAX]; static uint8_t R_ABS[DEP_MAX]; static uint64_t R_ID[DEP_MAX];
/* the common shape, a single tile dependency that moves with the pixel and no position
   conditions: deps_hold reduces to one identity read at pi + R_off1 (r_shape sets it) */
static int R_one; static int32_t R_off1; static uint32_t R_id1;   /* R_id1: R_ID[0] as 32 bits (game-RAM ids fit) */
static inline void r_shape(void) {
    R_one = R_n == 1 && R_ABS[0] == 0 && R_P.n == 0;
    R_off1 = R_one ? R_T[0] - R_pi : 0;
    R_id1 = (uint32_t)R_ID[0];
}
static long st_dep_hit __attribute__((unused)), st_dep_rej __attribute__((unused)),
            st_spr_hit __attribute__((unused)), st_vm_hit __attribute__((unused));
/* the per-pixel memo counters are for the x86 tools' report (memo_parity.sh); the daemon and the
   timing build leave them out of the per-pixel loop, where each one cost a global load + store
   per pixel and moved the loop's register allocation (2026-10-22: ~0.17M ARM instructions on
   Castlevania frame 60 appeared from one new branch beside them) */
#if defined(HDARM_LIB) || defined(HD_NOOUT)
#define PIXSTAT(x) ((void)0)
#else
#define PIXSTAT(x) (x)
#endif
/* -DPATHSTAT: which path each pixel of the lookups loop takes (study for per-tile decisions,
   OPEN_ITEMS item 11). Off in every shipped build. PS[] indices: see main's report. */
#ifdef PATHSTAT
static long PS[24];
#define PST(i) (PS[i]++)
#define PS_ADD(i, n) (PS[i] += (n))
#else
#define PST(i) ((void)0)
#define PS_ADD(i, n) ((void)0)
#endif
#ifdef MEMO_VERIFY
static long memo_checked, memo_failed;
static void memo_verify(int bad, const char *what, int x, int y) {
    memo_checked++;
    if (bad && memo_failed++ < 20) fprintf(stderr, "MEMO MISMATCH %s at (%d,%d)\n", what, x, y);
}
#endif

/* the same memo for sprite-record walks. Every pixel of one drawing entry's row (a capture
   sprite, or an addition) carries a record with the same match fields, mirroring and fine y;
   only offx and the sprite colour differ, and a condition on offx or x is re-evaluated for
   the asking pixel (dependency kind 4).
   Keyed by the entry id, a few entries at a time since sprites overlap; reset per line. */
typedef struct {
    uint32_t gid; int32_t pi, newidx, y; int ok, n, dis, mut; int64_t hd;
    int32_t T[DEP_MAX]; uint8_t K[DEP_MAX]; uint64_t ID[DEP_MAX]; PosDeps P;
} SMemo;
static SMemo SM[16];

/* the VERTICAL memo: per screen column, the last background walk there. A tile's lines
   below usually hold the same tile; with identities by content (palette and index, see
   dep_id) the walk one line up answers for this one when its dependencies, shifted down
   with the pixel, read the same. Position conditions (x, y, fine x/y) are re-evaluated for
   the asking pixel (see pos_hold). Both pack kinds: game-RAM packs identify tiles by content
   too (cid_build). */
typedef struct {
    int ok, n, dis, mut, hc; int32_t pi, newidx; uint32_t hv; int64_t hd; uint64_t src;
    int32_t T[DEP_MAX]; uint8_t K[DEP_MAX]; uint64_t ID[DEP_MAX]; PosDeps P;
} VMemo;
static VMemo VM[256];
static int64_t spr_walk(int pi, int x, int y, Rec *s, int *dis) {
    SMemo *m = &SM[s->gid & 15];
    if (m->ok && m->gid == s->gid && m->y == y && deps_hold(m->n, m->T, m->K, m->ID, m->pi, pi - m->pi, game_ram)
        && (!m->P.n || pos_hold(m->P.n, m->P.ci, m->P.kd, m->P.v, m->pi, pi,
                                rec_ctx(s->offx, s->offy, s->hm, s->vm), m->P.ctx))) {
#ifdef MEMO_VERIFY
        {
            Rec vs = *s; int vdis; int32_t vorg = vs.idx;
            int64_t vhd = get_matching(x, y, &vs, &vdis);
            memo_verify(vhd != m->hd || vdis != m->dis || (vs.idx != vorg) != m->mut
                        || (m->mut && vs.idx != m->newidx), "SPR", x, y);
        }
#endif
        *dis = m->dis;
        if (m->mut) set_idx(s, m->newidx);   /* CUR_PIX is -1 here: a sprite record */
        PIXSTAT(st_spr_hit++);
        return m->hd;
    }
    int32_t org = s->idx;
    DEP_ON = 1; DEP_N = 0; DEP_PN = 0; DEP_BAD = 0; DH_GEN++;
    int64_t v = get_matching(x, y, s, dis);
    DEP_ON = 0;
    m->ok = !DEP_BAD;
    if (m->ok) {
        m->gid = s->gid; m->pi = pi; m->y = y; m->hd = v; m->dis = *dis;
        pos_save(&m->P, rec_ctx(s->offx, s->offy, s->hm, s->vm));
        m->mut = s->idx != org; m->newidx = s->idx; m->n = DEP_N;
        memcpy(m->T, DEP_T, sizeof(int32_t) * DEP_N);
        memcpy(m->K, DEP_ABS, DEP_N);
        memcpy(m->ID, DEP_ID, sizeof(uint64_t) * DEP_N);
    }
    return v;
}
static uint8_t DEP_WANT[32] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                               1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

/* the background pixel's overlay write when a memo answers for a walk that rewrote the index
   (the walk would rewrite it the same way; the record copy is not kept, only the write) */
static __attribute__((noinline)) void bg_mut(int pi, const Rec *sp) {
    Rec t = *sp; CUR_PIX = pi; set_idx(&t, R_newidx); CUR_PIX = -1;
}
#ifdef MEMO_VERIFY   /* check build: do the walk anyway and compare with what the memo gives */
static void verify_bg(int pi, int x, int y, const Rec *sp, const uint8_t *b, int mut) {
    Rec vt = *sp; int vdis;
    vt.offx = (b[1] >> 2) & 7;
    if (mut) vt.idx = MUTI[pi];
    int32_t vorg = vt.idx;
    CUR_PIX = pi;
    int64_t vhd = get_matching(x, y, &vt, &vdis);
    CUR_PIX = -1;
    memo_verify(vhd != R_hd || vdis != R_dis || (vt.idx != vorg) != R_mut
                || (R_mut && vt.idx != R_newidx), "BG", x, y);
}
#endif
/* a decision's class for the display list: 1 = an HD block (hd >= 0, not 0xFFFFFFFF, the "no
   HD image"), 2 = a fallback decision (negative, not -2 = no tile) showing the original tile
   colour when render_original, 0 = nothing; *hv = the block. Computed once per walk and kept
   with the memos, so a reused pixel copies it. */
static inline int hd_class(int64_t hd, int rorig, uint32_t *hv) {
    *hv = (uint32_t)hd;
    return hd >= 0 ? hd != 0xFFFFFFFFLL : (hd != -2 && rorig) ? 2 : 0;
}
/* a background pixel the line memo did not answer: the column memo (vmem), else a walk.
   Its class and value come back in *hc / *hv (see hd_class). */
static inline __attribute__((always_inline)) int64_t bg_slow(int pi, int x, int y, const Rec *sp, const uint8_t *b,
                                                 int mut, uint64_t sid, int vmem, int cen, int rorig, const int gr,
                                                 int *dis, int *hc, uint32_t *hv) {
#if !defined(NO_VMEMO) && !defined(NO_DEPMEMO)
    if (vmem) {   /* the walk at this column on an earlier line */
        VMemo *v = &VM[x];
        if (v->ok && v->src == sid
            && deps_hold(v->n, v->T, v->K, v->ID, v->pi, pi - v->pi, gr)
            && (!v->P.n || pos_hold(v->P.n, v->P.ci, v->P.kd, v->P.v, v->pi, pi,
                                    rec_ctx((b[1] >> 2) & 7, sp->offy, sp->hm, sp->vm), v->P.ctx))) {
            R_ok = 1; R_src = sid; R_pi = v->pi; R_hd = v->hd; R_dis = v->dis; R_P = v->P;
            R_hc = *hc = v->hc; R_hv = *hv = v->hv;
            R_mut = v->mut; R_newidx = v->newidx; R_n = v->n;
            memcpy(R_T, v->T, sizeof(int32_t) * v->n);
            memcpy(R_ABS, v->K, v->n);
            memcpy(R_ID, v->ID, sizeof(uint64_t) * v->n);
            if (gr) r_shape();   /* only game-RAM packs take the fast path (and the call measured costly elsewhere) */
            PIXSTAT(st_vm_hit++);
            PST(5);
#ifdef MEMO_VERIFY
            verify_bg(pi, x, y, sp, b, mut);
#endif
            *dis = R_dis;
            if (R_mut) bg_mut(pi, sp);
            PIXSTAT(st_dep_hit++);
            return R_hd;
        }
    }
#endif
    Rec t = *sp;
    t.offx = (b[1] >> 2) & 7;
    if (mut) t.idx = MUTI[pi];
    int32_t org = t.idx;
    CUR_PIX = pi;
    /* noting dependencies only pays when the tile's other pixels are not taken by the 8-pixel
       cache, i.e. the walk disables it; guess that from the last walk in the same 8-pixel column
       (the tile above, as a rule) - no notes = no reuse, never a wrong answer. The column memo
       always notes. */
    int track = vmem || DEP_WANT[x >> 3] || !cen;
    DEP_ON = track; DEP_N = 0; DEP_PN = 0; DEP_BAD = 0; DH_GEN++;
    int64_t hd = get_matching(x, y, &t, dis);
    DEP_ON = 0;
    CUR_PIX = -1;
    PST(6); if (*dis) PST(9);
    DEP_WANT[x >> 3] = (uint8_t)*dis;
    *hc = hd_class(hd, rorig, hv);
    R_ok = track && !DEP_BAD;
    if (R_ok) {
        R_src = sid; R_pi = pi; R_hd = hd; R_dis = *dis; R_hc = *hc; R_hv = *hv;
        pos_save(&R_P, rec_ctx((b[1] >> 2) & 7, sp->offy, sp->hm, sp->vm));
        R_mut = t.idx != org; R_newidx = t.idx; R_n = DEP_N;
        memcpy(R_T, DEP_T, sizeof(int32_t) * DEP_N);
        memcpy(R_ABS, DEP_ABS, DEP_N);
        memcpy(R_ID, DEP_ID, sizeof(uint64_t) * DEP_N);
        if (gr) r_shape();
#ifndef NO_VMEMO
        if (vmem) {
            VMemo *v = &VM[x];
            v->ok = 1; v->src = sid; v->pi = pi; v->hd = hd; v->dis = *dis; v->P = R_P;
            v->hc = *hc; v->hv = *hv;
            v->mut = R_mut; v->newidx = R_newidx; v->n = DEP_N;
            memcpy(v->T, DEP_T, sizeof(int32_t) * DEP_N);
            memcpy(v->K, DEP_ABS, DEP_N);
            memcpy(v->ID, DEP_ID, sizeof(uint64_t) * DEP_N);
        }
#endif
    }
    return hd;
}
/* a pixel's sprite records (block sbi): their decisions, in the order the reference walks them.
   Out of line, like bg_slow. any = the pixel's decision line is already open. */
static inline __attribute__((always_inline)) void spr_pixel(int pi, int x, int y, uint16_t sbi, int any) {
    int lowest = 999, dis;
    TPS(t2);
    int ns = SB[sbi].n;
    for (int k = 0; k < ns; k++) SB[sbi].hd[k] = -3;   /* not drawn unless walked below */
    for (int k = ns - 1; k >= 0; k--) {
        Rec *s = spr_of(pi, k);
        if (s->bgp) {
            if (s->sci) lowest = k;
            int64_t v = spr_walk(pi, x, y, s, &dis);
            if (!any) { outf("P %d %d", y, x); any = 1; }
            outf(" S%d", k); outv(v);
            SB[sbi].hd[k] = v;
        }
    }
    for (int k = ns - 1; k >= 0; k--) {
        Rec *s = spr_of(pi, k);
        if (!s->bgp && lowest > k) {
            int64_t v = spr_walk(pi, x, y, s, &dis);
            if (!any) { outf("P %d %d", y, x); any = 1; }
            outf(" S%d", k); outv(v);
            SB[sbi].hd[k] = v;
        }
    }
    TPA(2, t2);
    if (any) outf("\n");
}
/* The fast run (lookups_gr) one tile at a time: how many of the pixels x+1.. (at most maxn,
   which is <= 7 and stays inside the tile) continue a cached decision, with sb/pd/bp pointing
   at pixel x+1's sprite block, PID and record bytes. Every pixel must have no sprite, the same
   PID and the same backdrop palette index; and then, by mode:
     0  nothing else (no BG run is open: a "nothing" tile, or a transparent one)
     1  transparent (original tile colour with no open run: an opaque pixel would open one)
     2  opaque with palette index ref (the open colour run; an index change ends the scan even
        where the colours are equal, which only hands that pixel back to the loop body)
     3  none (the loop body must take the next pixel)
     4  HD block: the tile's offx counts up from ref (the open block run's next offx)
   Reads up to 8 entries/records past x+1 regardless of maxn (PID, SBI and the line-0 row copy
   are padded for that). The scalar loop is the definition; NEON only on the A9. */
static inline int run_scan(const uint16_t *sb, const uint16_t *pd, const uint8_t *bp, int maxn,
                           uint16_t pid0, int bdli, int mode, int ref) {
    if (mode == 3 || maxn <= 0) return 0;
#if defined(__ARM_NEON) && !defined(NO_NEONSCAN)
    uint8x8x4_t r = vld4_u8(bp);   /* lanes: record byte 0..3 of 8 pixels */
    uint16x8_t okw = vandq_u16(vceqq_u16(vld1q_u16(sb), vdupq_n_u16(0)), vceqq_u16(vld1q_u16(pd), vdupq_n_u16(pid0)));
    uint8x8_t ok = vand_u8(vmovn_u16(okw), vceq_u8(vand_u8(r.val[2], vdup_n_u8(63)), vdup_n_u8((uint8_t)bdli)));
    if (mode == 1) ok = vand_u8(ok, vceq_u8(vand_u8(r.val[1], vdup_n_u8(3)), vdup_n_u8(0)));
    else if (mode == 2) ok = vand_u8(ok, vand_u8(vtst_u8(r.val[1], vdup_n_u8(3)),
                                                 vceq_u8(vand_u8(r.val[3], vdup_n_u8(63)), vdup_n_u8((uint8_t)ref))));
    else if (mode == 4) {
        static const uint8_t iota[8] = {0, 1, 2, 3, 4, 5, 6, 7};
        ok = vand_u8(ok, vceq_u8(vand_u8(vshr_n_u8(r.val[1], 2), vdup_n_u8(7)),
                                 vadd_u8(vld1_u8(iota), vdup_n_u8((uint8_t)ref))));
    }
    uint64_t m = vget_lane_u64(vreinterpret_u64_u8(ok), 0);
    int n = ~m ? __builtin_ctzll(~m) >> 3 : 8;
    return n < maxn ? n : maxn;
#else
    int n = 0;
    for (; n < maxn; n++) {
        const uint8_t *q = bp + 4 * n;
        if (sb[n] || pd[n] != pid0 || (q[2] & 63) != bdli) break;
        if (mode == 1 && (q[1] & 3)) break;
        if (mode == 2 && (!(q[1] & 3) || (q[3] & 63) != ref)) break;
        if (mode == 4 && ((q[1] >> 2) & 7) != ref + n) break;
    }
    return n;
#endif
}

/* the per-pixel loop, specialized for game-RAM packs (GR = 1) and the rest, so each copy
   carries only its own identity and memo paths (see lookups) */
static inline __attribute__((always_inline)) void lookups_gr(const int GR) {
    int top = overscan[0] > 0 ? overscan[0] : 0, right = overscan[1] > 0 ? overscan[1] : 0;
    int bottom = overscan[2] > 0 ? overscan[2] : 0, left = overscan[3] > 0 ? overscan[3] : 0;
    const int rorig = render_original, cen = cache_enabled;   /* fixed for the frame */
#ifndef NO_VMEMO
    const int vmem = 1;   /* both pack kinds: tiles are identified by content (dep_id) */
    for (int i = 0; i < 256; i++) VM[i].ok = 0;
#else
    const int vmem = 0;
#endif
    for (int y = top; y < 240 - bottom; y++) {
        int fl = y == 0 ? P0L : y;        /* (0,0) may show another line's first pixel */
        uint16_t ft = LT[fl];
        int scroll_x = ((ft & 0x1F) << 3) | LFX[fl] | ((ft & 0x400) ? 256 : 0);
        int use_cached = 0; int64_t cached = -1;
        int dep_lim = 999;   /* last x a dependency-checked cache entry covers (999 = whole tile) */
        int cached_c = 0; uint32_t cached_v = 0;   /* its class and value, see below */
        R_ok = 0;                     /* the memo never crosses lines: y-conditions */
        for (int i = 0; i < 16; i++) SM[i].ok = 0;   /* nor frames: entry ids repeat */
        const uint8_t *brow = RECB + y * 1024;
        if (y == 0) {   /* pixel (0,0) shows line P0L's first pixel: a patched copy of the row */
            static uint8_t row0[1024 + 64];   /* +64: run_scan reads past the row's end */
            memcpy(row0, brow, 1024); memcpy(row0, RECB + P0L * 1024, 4);
            brow = row0;
        }
        int bd_sx = -1, bd_li = -1; uint32_t bd_c = 0;                     /* backdrop run */
        int g_sx = -1, g_k = 0, g_ofy = 0, g_ofx = 0, g_len = 0; uint32_t g_v = 0;   /* BG run */
        nBDR = nBGR = 0;
        const uint8_t *bp = brow + left * 4;
        const uint16_t *pidr = PID + (y << 8), *sbir = SBI + (y << 8);
        /* (two passes per line - decisions, then runs - was tried 2026-10-22 to cut the live
           state on the A9 and cost ~17 more ARM instructions per pixel) */
        int pfx = left;                   /* next x at which the streams are prefetched (see PFI) */
        const Rec *span_next = y + 1 < 240 ? SPAN[y + 1] : SPAN[y];
        for (int x = left; x < 256 - right; x++) {
            int pi = (y << 8) | x;
            int any = 0, dis;
            const uint8_t *b = bp; bp += 4;   /* the pixel's own record bytes */
            PST(0);
            if (x >= pfx) {   /* once per 8 pixels (the fast run below can skip x past pfx) */
                PFI(b + PFD); PFI(pidr + x + PFD / 2); PFI(sbir + x + PFD / 2);
                if ((x >> 3) < 35) PFI(span_next + (x >> 3));   /* next line's span records */
                pfx = x + 8;
            }
            {   /* backdrop: the palette index first (equal index, equal colour), the RGB only
                   when it changes, since two indices can share a colour */
                int ci = b[2] & 63;
                if (ci != bd_li) {
                    bd_li = ci;
                    uint32_t col = PRGB[ci];
                    if (bd_sx < 0 || col != bd_c) {
                        if (bd_sx >= 0) { BDR[nBDR].x = (uint16_t)bd_sx; BDR[nBDR].len = (uint16_t)(x - bd_sx); BDR[nBDR++].v = bd_c; }
                        bd_sx = x; bd_c = col;
                    }
                }
            }
            int gk = -1, gofy = 0, gofx = 0; uint32_t gv = 0;   /* this pixel's BG span part */
            TPS(t0);
            /* the pixel's tile record, read in place; copied into t only when a walk needs it */
            const Rec *sp = SPANF + pidr[x];
            TPA(0, t0);
#ifdef PATHSTAT
            if (sp->idx == NOTILE) PST(1);
            if (sbir[x]) PST(8);
#endif
            if (sp->idx != NOTILE) {
                int64_t hd;          /* the decision itself: only the decision dump reads it */
                int hc; uint32_t hv; /* its class and value (hd_class), what the display list uses */
                if (((scroll_x + x) & 7) == 0) use_cached = 0;
                if (x > dep_lim) use_cached = 0;   /* a per-tile dependency run ended (see below) */
                if (use_cached) { hd = cached; hc = cached_c; hv = cached_v; PST(2);
#ifdef MEMO_VERIFY   /* a pixel the per-tile dependency cache answers: do its walk anyway */
                    if (dep_lim != 999) verify_bg(pi, x, y, sp, b, 0);
#endif
#ifdef TPROF
                    TP[4]++;
#endif
                }
                else {
                    TPS(t1);
                    /* game-RAM packs never rewrite a tile index (get_matching_walk has no
                       fallback for them), so their overlay stays empty: no mut, no MUTI */
#ifdef MEMO_VERIFY
                    if (GR && ANY_MUT) memo_verify(1, "GR-overlay", x, y);
#endif
                    int mut = !GR && ANY_MUT && ((MUTB[pi >> 3] >> (pi & 7)) & 1);
                    /* the tile's identity, overlay included: dep_id(pi), inlined (pi is on screen).
                       A game-RAM pack's is a cid_build number, so it fits 32 bits. */
                    uint64_t sid;
                    if (GR) sid = CIDG[pidr[x]];
                    else sid = (uint64_t)sp->pal << 32 | (uint32_t)(mut ? MUTI[pi] : sp->idx);
                    int reuse = 0;
                    if (R_ok && (GR ? (uint32_t)sid == (uint32_t)R_src : sid == R_src)) {
                        /* (a precomputed reuse window over runs of equal ids was tried
                           2026-10-22 and cost more than these checks: runs are one tile) */
                        if (GR && R_one) {   /* deps_hold's !ANY_MUT loop for that one entry
                                                (game-RAM packs; the others measured slower) */
#ifdef MUT_DEPLOOSE
                            reuse = 1;
#else
                            /* an off-screen id is ~0, and no cid_build number is 0xFFFFFFFF */
                            int32_t t = pi + R_off1;
                            reuse = ((uint32_t)t >= 61440 ? 0xFFFFFFFFu : CIDG[PID[t]]) == R_id1;
#endif
                        } else
                        reuse = deps_hold(R_n, R_T, R_ABS, R_ID, R_pi, pi - R_pi, GR)
                            && (!R_P.n || pos_hold(R_P.n, R_P.ci, R_P.kd, R_P.v, R_pi, pi, rec_ctx((b[1] >> 2) & 7, sp->offy, sp->hm, sp->vm), R_P.ctx));
                        if (!reuse) PIXSTAT(st_dep_rej++);
                    }
#ifndef NO_DEPMEMO
                    if (reuse) {
#ifdef MEMO_VERIFY
                        verify_bg(pi, x, y, sp, b, mut);
#endif
                        hd = R_hd; dis = R_dis; hc = R_hc; hv = R_hv;
                        PST(3); if (dis) PST(10);
                        if (!GR && R_mut) bg_mut(pi, sp);
                        PIXSTAT(st_dep_hit++);
                    } else
#endif
                    hd = bg_slow(pi, x, y, sp, b, mut, sid, vmem, cen, rorig, GR, &dis, &hc, &hv);
                    TPA(1, t1);
#ifdef TPROF
                    TP[3]++;
#endif
                    if (!dis && cen) { cached = hd; cached_c = hc; cached_v = hv; use_cached = 1; dep_lim = 999; }
#ifndef NO_TILEDEP
                    else if (GR && dis && R_ok && (uint32_t)R_src == (uint32_t)sid && !ANY_MUT && !R_P.n) {
                        /* A per-pixel ("disableCache") decision is still the same for the rest of
                           this tile's pixels when what it depends on is: the line memo would
                           reuse it at the next pixel exactly when every noted tile dependency,
                           moved along with the pixel, reads the same id (deps_hold). Find how
                           many of the tile's remaining pixels that holds for, comparing PIDs
                           (equal PID = equal id; unequal just ends the run early), and treat
                           those pixels as cached. Position conditions (R_P) or sprite/fixed
                           dependencies end it: those need the per-pixel check. */
                        int rem = 7 - ((scroll_x + x) & 7), ok = rem;
                        for (int k = 0; k < R_n && ok > 0; k++) {
                            if (R_ABS[k]) { ok = 0; break; }
                            int32_t t0 = pi + (R_T[k] - R_pi);
                            if ((uint32_t)t0 >= 61440) { ok = 0; break; }
                            uint16_t p0 = PID[t0];
                            int j = 1;
                            for (; j <= ok; j++) {
                                uint32_t t = (uint32_t)(t0 + j);
                                if (t >= 61440 || PID[t] != p0) break;
                            }
                            ok = j - 1;
                        }
                        if (ok > 0) {
                            cached = hd; cached_c = hc; cached_v = hv; use_cached = 1;
                            dep_lim = x + ok;
                            PST(17); PS_ADD(18, ok);
                        }
                    }
#endif
                }
                if (OUT_ON) { outf("P %d %d B", y, x); any = 1; outv(hd); }
                if (hc == 1) { gk = 0; gv = hv; gofy = sp->offy; gofx = (b[1] >> 2) & 7; }
                else if (hc == 2 && (b[1] & 3)) { gk = 1; gv = PRGB[b[3] & 63]; }
            }
            /* (packing the run state into one key compared per pixel was tried 2026-10-22
               and measured 0.06-0.5M ARM instructions WORSE per frame on all four frames) */
            if (gk >= 0 && g_sx >= 0 && gk == g_k && gv == g_v && (gk == 1 || (gofy == g_ofy && gofx == g_ofx + g_len))) g_len++;
            else {
                if (g_sx >= 0) {
                    BGRun *r = &BGR[nBGR++];
                    r->x = (uint16_t)g_sx; r->len = (uint16_t)g_len; r->k = (uint8_t)g_k; r->v = g_v;
                    r->ofy = (uint8_t)g_ofy; r->ofx = (uint8_t)g_ofx;
                }
                if (gk >= 0) { g_sx = x; g_k = gk; g_v = gv; g_ofy = gofy; g_ofx = gofx; g_len = 1; } else g_sx = -1;
            }
            uint16_t sbi = sbir[x];
            if (sbi) spr_pixel(pi, x, y, sbi, any);
            else if (any) outf("\n");
#if !defined(NO_FASTRUN) && !defined(TPROF)
            /* The rest of this tile's pixels, while each of them would do nothing but extend the
               HD-block BG run: the decision is the cached one (same tile, so the same record and
               the same class/value), no sprite, the same backdrop palette index, and the next
               fine x. That is exactly what the loop body above would do for such a pixel, in a
               loop with almost nothing live. The decision dump (OUT_ON) needs every pixel. */
            if (use_cached && !OUT_ON) {
                const uint16_t pid0 = pidr[x];
                const int xe = dep_lim + 1 < 256 - right ? dep_lim + 1 : 256 - right;
#ifdef PATHSTAT
                const int fx0 = x;
#endif
#ifndef NO_TILESCAN
                /* the rest of the tile in one step (run_scan; the scalar loops below define it) */
                int maxn = 7 - ((scroll_x + x) & 7);
                if (maxn > xe - 1 - x) maxn = xe - 1 - x;
                if (cached_c == 1) {
                    if (gk == 0) {
                        int n = run_scan(sbir + x + 1, pidr + x + 1, bp, maxn, pid0, bd_li, 4, g_ofx + g_len);
                        g_len += n; x += n; bp += 4 * n;
                    }
                } else {
                    int mode = cached_c == 2 ? (g_sx >= 0 ? (g_k == 1 ? 2 : 3) : 1) : (g_sx >= 0 ? 3 : 0);
                    int n = run_scan(sbir + x + 1, pidr + x + 1, bp, maxn, pid0, bd_li, mode, b[3] & 63);
                    if (mode == 2) g_len += n;
                    x += n; bp += 4 * n;
                }
#else
                if (cached_c == 1) {   /* HD block: same block and row, the next fine x */
                    if (gk == 0)
                        while (x + 1 < xe && ((scroll_x + x + 1) & 7) && !sbir[x + 1] && pidr[x + 1] == pid0
                               && (bp[2] & 63) == bd_li && ((bp[1] >> 2) & 7) == g_ofx + g_len) {
                            g_len++; x++; bp += 4;
                        }
                } else {   /* original tile (class 2) or nothing: an opaque pixel extends a
                              colour run of the same colour, a transparent one extends no run */
                    /* (closing and reopening runs here on a colour change, instead of handing
                       the pixel back to the body, was tried 2026-10-22: Castlevania -0.2/-0.8 ms,
                       the other three games +0.35/+0.5 ms on the A9. Not kept.) */
                    while (x + 1 < xe && ((scroll_x + x + 1) & 7) && !sbir[x + 1] && pidr[x + 1] == pid0
                           && (bp[2] & 63) == bd_li) {
                        if (cached_c == 2 && (bp[1] & 3)) {
                            if (g_sx < 0 || g_k != 1 || PRGB[bp[3] & 63] != g_v) break;
                            g_len++;
                        } else if (g_sx >= 0) break;
                        x++; bp += 4;
                    }
                }
#endif
#ifdef PATHSTAT
                PS[7] += x - fx0;   /* pixels the fast run took; then why it stopped at x + 1 */
                if (cached_c == 1) PS[23] += x - fx0;   /* of those, HD-block pixels */
                if (x + 1 >= xe) PST(11);
                else if (!((scroll_x + x + 1) & 7)) PST(12);
                else if (sbir[x + 1]) PST(13);
                else if (pidr[x + 1] != pid0) PST(14);
                else if ((bp[2] & 63) != bd_li) PST(15);
                else {
                    PST(16);
                    if (cached_c == 1) PST(19);   /* HD block: offx not the next one (flip?) */
                    else if (cached_c == 2 && (bp[1] & 3) && g_sx >= 0 && g_k == 1) PST(20);   /* colour change */
                    else if (cached_c == 2 && (bp[1] & 3)) PST(21);   /* opaque after no run / other run */
                    else PST(22);   /* transparent after an open run */
                }
#endif
            }
#endif
        }
        {   /* close the line's open runs */
            int R = 256 - right;
            if (bd_sx >= 0) { BDR[nBDR].x = (uint16_t)bd_sx; BDR[nBDR].len = (uint16_t)(R - bd_sx); BDR[nBDR++].v = bd_c; }
            if (g_sx >= 0) {
                BGRun *r = &BGR[nBGR++];
                r->x = (uint16_t)g_sx; r->len = (uint16_t)g_len; r->k = (uint8_t)g_k; r->v = g_v;
                r->ofy = (uint8_t)g_ofy; r->ofx = (uint8_t)g_ofx;
            }
        }
        gen_line(y, left, 256 - right);   /* this line's decisions are final: emit its list */
    }
}
static void lookups(void) { if (game_ram) lookups_gr(1); else lookups_gr(0); }

/* ================================================================ hdarm4: display list + painter
 *
 * The ARM turns its decisions into a per-line DISPLAY LIST of 64-bit paint commands, in the order
 * Mesen composites a pixel (hdref.render): backdrop, background layer 0, behind-BG sprites (lowest
 * priority first), layer 1, the BG tile (or its fallback colour), layer 2, front sprites, layer 3,
 * then the line's grayscale / emphasis. Each command paints one horizontal span of one NES line,
 * i.e. 2 HD rows x 2*len HD pixels. The FPGA only executes the list (paint()); it decides nothing.
 *
 * Command (u64):  [1:0] op  0 BLIT  1 FILL  2 FX  3 EOL
 *   [9:2] x0   [17:10] len-1   [19:18] blend mode (0 alpha, 1 add, 2 subtract)
 *   BLIT: [22:20] tile row offy  [23] vm  [24] hm  [27:25] offx of the first pixel  [51:28] block
 *   FILL: [43:20] RGB (opaque)
 *   FX:   [2] gray  [5:3] emphasis
 * A BLIT never crosses its tile (offx0 + len <= 8). Order is the painter's only rule: a later
 * command paints over an earlier one where they overlap.
 */
static uint64_t *DLB; static size_t DLn, DLcap;
static void dl_put(uint64_t c) {
    if (DLn == DLcap) { DLcap = DLcap * 2 + 65536; DLB = realloc(DLB, DLcap * sizeof(uint64_t)); }
    DLB[DLn++] = c;
}
static void dl_blit(int x0, int len, int mode, uint32_t block, int offy, int vm, int hm, int offx0) {
    dl_put(0 | (uint64_t)x0 << 2 | (uint64_t)(len - 1) << 10 | (uint64_t)mode << 18 | (uint64_t)offy << 20
           | (uint64_t)vm << 23 | (uint64_t)hm << 24 | (uint64_t)offx0 << 25 | (uint64_t)(block & 0xFFFFFF) << 28);
}
static void dl_fill(int x0, int len, uint32_t rgb) {
    dl_put(1 | (uint64_t)x0 << 2 | (uint64_t)(len - 1) << 10 | (uint64_t)(rgb & 0xFFFFFF) << 20);
}
static uint32_t PRGB[64];
static const uint32_t MESEN_2C02[64] = {
    0x666666, 0x002A88, 0x1412A7, 0x3B00A4, 0x5C007E, 0x6E0040, 0x6C0600, 0x561D00,
    0x333500, 0x0B4800, 0x005200, 0x004F08, 0x00404D, 0x000000, 0x000000, 0x000000,
    0xADADAD, 0x155FD9, 0x4240FF, 0x7527FE, 0xA01ACC, 0xB71E7B, 0xB53120, 0x994E00,
    0x6B6D00, 0x388700, 0x0C9300, 0x008F32, 0x007C8D, 0x000000, 0x000000, 0x000000,
    0xFFFEFF, 0x64B0FF, 0x9290FF, 0xC676FF, 0xF36AFF, 0xFE6ECC, 0xFE8170, 0xEA9E22,
    0xBCBE00, 0x88D800, 0x5CE430, 0x45E082, 0x48CDDE, 0x4F4F4F, 0x000000, 0x000000,
    0xFFFEFF, 0xC0DFFF, 0xD3D2FF, 0xE8C8FF, 0xFBC2FF, 0xFEC4EA, 0xFECCC5, 0xF7D8A5,
    0xE4E594, 0xCFEF96, 0xBDF4AB, 0xB3F3CC, 0xB5EBF2, 0xB8B8B8, 0x000000, 0x000000,
};
static int render_original;
static long st_cmds, st_cyc_lines;

/* one paint item of one pixel */
typedef struct { uint32_t gid; uint8_t kind, offy, vm, hm, offx; uint32_t blk; uint32_t rgb; } Item;

/* emit one phase's items for a line: items[x][0..cnt[x]) in per-pixel draw order */
#define MAXG 512
static void emit_phase(int L, int R, Item (*it)[8], const int *cnt) {
    /* nodes = distinct gids; edges = consecutive items of a pixel (draw order) */
    static uint32_t node[MAXG]; static int indeg[MAXG], ehead[MAXG], enext[2048], eto[2048];
    static int nmin[MAXG], nmax[MAXG];   /* x extent of each gid's items: the sweep covers only that */
    int nn = 0, ne = 0;
    for (int x = L; x < R; x++) {
        int prev = -1;
        for (int i = 0; i < cnt[x]; i++) {
            int k;
            for (k = 0; k < nn; k++) if (node[k] == it[x][i].gid) break;
            if (k == nn) { if (nn == MAXG) goto perpixel; node[nn] = it[x][i].gid; indeg[nn] = 0; ehead[nn] = -1; nmin[nn] = x; nn++; }
            nmax[k] = x;
            if (prev >= 0 && prev != k) {
                if (ne == 2048) goto perpixel;
                eto[ne] = k; enext[ne] = ehead[prev]; ehead[prev] = ne; ne++; indeg[k]++;
            }
            prev = k;
        }
    }
    {
        /* Kahn; a cycle means no single span order serves every pixel */
        static int order[MAXG], q[MAXG];
        int qh = 0, qt = 0, no = 0;
        for (int k = 0; k < nn; k++) if (!indeg[k]) q[qt++] = k;
        while (qh < qt) {
            int k = q[qh++]; order[no++] = k;
            for (int e = ehead[k]; e >= 0; e = enext[e]) if (--indeg[eto[e]] == 0) q[qt++] = eto[e];
        }
        if (no != nn) goto perpixel;
        for (int oi = 0; oi < no; oi++) {
            int k = order[oi]; uint32_t g = node[k];
            /* sweep this gid's x extent, merging neighbours into spans */
            int sx = -1; Item cur = {0}; int len = 0;
            for (int x = nmin[k]; x <= nmax[k] + 1; x++) {
                const Item *p = NULL;
                if (x <= nmax[k]) for (int i = 0; i < cnt[x]; i++) if (it[x][i].gid == g) { p = &it[x][i]; break; }
                int ext = 0;
                if (p && sx >= 0 && p->kind == cur.kind) {
                    if (p->kind == 1) ext = p->rgb == cur.rgb;
                    else ext = p->blk == cur.blk && p->offy == cur.offy && p->vm == cur.vm && p->hm == cur.hm
                               && p->offx == cur.offx + len;
                }
                if (ext) { len++; continue; }
                if (sx >= 0) {
                    if (cur.kind == 1) dl_fill(sx, len, cur.rgb);
                    else dl_blit(sx, len, 0, cur.blk, cur.offy, cur.vm, cur.hm, cur.offx);
                    st_cmds++;
                }
                if (p) { sx = x; cur = *p; len = 1; } else sx = -1;
            }
        }
        return;
    }
perpixel:
    st_cyc_lines++;
    for (int x = L; x < R; x++)
        for (int i = 0; i < cnt[x]; i++) {
            const Item *p = &it[x][i];
            if (p->kind == 1) dl_fill(x, 1, p->rgb);
            else dl_blit(x, 1, 0, p->blk, p->offy, p->vm, p->hm, p->offx);
            st_cmds++;
        }
}

static void emit_layer(int y, const int *cfgv, uint32_t recno, int L, int R) {
    const uint8_t *rc = D + SEC[6] + 32 * (uint64_t)recno;
    uint32_t map = rd32(rc), w = rd32(rc + 4), left = rd32(rc + 12), top = rd32(rc + 16);
    int mode = rc[24];
    int sx = cfgv[0], sy = cfgv[1], mn = cfgv[2], mx = cfgv[3];
    int x0 = L > mn ? L : mn, x1 = (R - 1) < mx ? (R - 1) : mx;
    int cols = (int)((w + 7) >> 3);
    int py = (int)top + y + sy;
#ifndef NO_LAYERFAST
    /* The same commands dl_blit would build, one per 8-pixel step of the layer image: space
       reserved once, the map row read in order, the fixed fields (mode, tile row) built once.
       2026-10-22: a layer is ~33 commands a line and Zelda II draws ~9 layers a line. */
    if (x0 > x1) return;
    if (DLn + (size_t)((x1 - x0) >> 3) + 2 > DLcap) {
        DLcap = (DLn + (size_t)((x1 - x0) >> 3) + 2) * 2 + 65536;
        DLB = realloc(DLB, DLcap * sizeof(uint64_t));
    }
    uint64_t *o = DLB + DLn;
    const uint64_t base = (uint64_t)mode << 18 | (uint64_t)(py & 7) << 20;
    const uint8_t *row = D + SEC[7] + map + 4 * (uint64_t)((py >> 3) * cols);
    int px = (int)left + x0 + sx;
    for (int x = x0; x <= x1;) {
        int n = 8 - (px & 7); if (x + n - 1 > x1) n = x1 - x + 1;
        uint32_t bi = rd32(row + 4 * (uint64_t)(px >> 3));
        if (bi != 0xFFFFFFFFu)
            *o++ = base | (uint64_t)x << 2 | (uint64_t)(n - 1) << 10 | (uint64_t)(px & 7) << 25
                 | (uint64_t)(bi & 0xFFFFFF) << 28;
        x += n; px += n;
    }
    st_cmds += (long)(o - (DLB + DLn));
    DLn = (size_t)(o - DLB);
#else
    for (int x = x0; x <= x1;) {
        int px = (int)left + x + sx;
        int n = 8 - (px & 7); if (x + n - 1 > x1) n = x1 - x + 1;
        uint32_t bi = rd32(D + SEC[7] + map + 4 * (uint64_t)((py >> 3) * cols + (px >> 3)));
        if (bi != 0xFFFFFFFFu) { dl_blit(x, n, mode, bi, py & 7, 0, 0, px & 7); st_cmds++; }
        x += n;
    }
#endif
}

static int ratio(uint32_t i, int s) { return (int32_t)rd32(D + SEC[8] + i * 2048 + (uint32_t)s * 4); }

/* one line of the display list; lookups() calls it as soon as the line's decisions are made */
static void gen_line(int y, int L, int R) {
    static Item it[256][8]; static int cnt[256];
    {
        int fl = y == 0 ? P0L : y;
        uint16_t ft = LT[fl];
        int scroll_x = ((ft & 0x1F) << 3) | LFX[fl] | ((ft & 0x400) ? 256 : 0);
        int scroll_yv = scroll_y(ft);
        /* per-layer line configs (hdref.bg_line) */
        int cfg[4][10][4]; int cok[4][10];
        for (int l = 0; l < 4; l++)
            for (int j = 0; j < NLAY[l]; j++) {
                const uint8_t *rc = D + SEC[6] + 32 * (uint64_t)LAY[l][j];
                int w = (int)rd32(rc + 4), h = (int)rd32(rc + 8), lf = (int)rd32(rc + 12), tp = (int)rd32(rc + 16);
                uint16_t hr = rc[20] | rc[21] << 8, vr = rc[22] | rc[23] << 8;
                int sx = ratio(hr, scroll_x), sy = ratio(vr, scroll_yv);
                cok[l][j] = y >= -sy && y + tp + sy + 1 <= h;
                cfg[l][j][0] = sx; cfg[l][j][1] = sy; cfg[l][j][2] = -sx; cfg[l][j][3] = w - lf - sx - 1;
            }
        /* backdrop: the runs lookups merged */
        for (int i = 0; i < nBDR; i++) { dl_fill(BDR[i].x, BDR[i].len, BDR[i].v); st_cmds++; }
        for (int j = 0; j < NLAY[0]; j++) if (cok[0][j]) emit_layer(y, cfg[0][j], LAY[0][j], L, R);
        /* sprite phases cover only the line's sprite-pixel range (emit_phase gives the same
           commands for any range that holds every item) */
        int SL = SPX0[y] > L ? SPX0[y] : L, SR = SPX1[y] + 1 < R ? SPX1[y] + 1 : R;
        /* behind-BG sprites, lowest priority (highest k) first */
        if (SL < SR) for (int x = SL; x < SR; x++) {
            int pi = (y << 8) | x; cnt[x] = 0;
            int ns = nspr_of(pi);
            for (int k = ns - 1; k >= 0; k--) {
                const Rec *s = spr_of(pi, k); int64_t hd = SB[SBI[pi]].hd[k];
                if (!s->bgp || hd == -3) continue;
                Item *p = &it[x][cnt[x]];
                if (hd >= 0) { if (hd == 0xFFFFFFFFLL) continue; p->kind = 0; p->blk = (uint32_t)hd; }
                else if (s->sci) { p->kind = 1; p->rgb = PRGB[s->scolor & 63]; }
                else continue;
                p->gid = s->gid; p->offy = s->offy; p->vm = s->vm; p->hm = s->hm; p->offx = s->offx;
                cnt[x]++;
            }
        }
        if (SL < SR) emit_phase(SL, SR, it, cnt);
        for (int j = 0; j < NLAY[1]; j++) if (cok[1][j]) emit_layer(y, cfg[1][j], LAY[1][j], L, R);
        /* the BG tile: one item per pixel, one gid, so emit_phase reduces to merging runs,
           which lookups did as it decided the pixels (BGR) */
        for (int i = 0; i < nBGR; i++) {
            const BGRun *r = &BGR[i];
            if (r->k == 1) dl_fill(r->x, r->len, r->v);
            else dl_blit(r->x, r->len, 0, r->v, r->ofy, 0, 0, r->ofx);
            st_cmds++;
        }
        for (int j = 0; j < NLAY[2]; j++) if (cok[2][j]) emit_layer(y, cfg[2][j], LAY[2][j], L, R);
        /* front sprites */
        if (SL < SR) for (int x = SL; x < SR; x++) {
            int pi = (y << 8) | x; cnt[x] = 0;
            int ns = nspr_of(pi);
            for (int k = ns - 1; k >= 0; k--) {
                const Rec *s = spr_of(pi, k); int64_t hd = SB[SBI[pi]].hd[k];
                if (s->bgp || hd == -3) continue;
                Item *p = &it[x][cnt[x]];
                if (hd >= 0) { if (hd == 0xFFFFFFFFLL) continue; p->kind = 0; p->blk = (uint32_t)hd; }
                else if (s->sci) { p->kind = 1; p->rgb = PRGB[s->scolor & 63]; }
                else continue;
                p->gid = s->gid; p->offy = s->offy; p->vm = s->vm; p->hm = s->hm; p->offx = s->offx;
                cnt[x]++;
            }
        }
        if (SL < SR) emit_phase(SL, SR, it, cnt);
        for (int j = 0; j < NLAY[3]; j++) if (cok[3][j]) emit_layer(y, cfg[3][j], LAY[3][j], L, R);
        if (LGE[fl]) dl_put(2 | (uint64_t)(LGE[fl] & 1) << 2 | (uint64_t)(LGE[fl] >> 1) << 3);
        dl_put(3);
    }
}

#ifndef HDARM_LIB   /* the C painter, image writer and file loader serve main() only */
/* ---- the painter: what the FPGA does, from the display list alone ---- */
static uint8_t *IMG;              /* 480 x 512 x 3 */
static void blend(uint8_t *o, const uint8_t *s, int mode) {
    int pr = s[0], pg = s[1], pb = s[2], a = s[3];
    if (mode == 0) {
        if (a == 255) { o[0] = pr; o[1] = pg; o[2] = pb; }
        else if (a) {
            int inv = (256 - a) & 0xFF;
            o[0] = (uint8_t)(pr + ((inv * o[0]) >> 8 & 0xFF));
            o[1] = (uint8_t)(pg + ((inv * o[1]) >> 8 & 0xFF));
            o[2] = (uint8_t)(pb + ((inv * o[2]) >> 8 & 0xFF));
        }
    } else if (mode == 1) {
        int v;
        v = pr + o[0]; o[0] = v > 255 ? 255 : v; v = pg + o[1]; o[1] = v > 255 ? 255 : v;
        v = pb + o[2]; o[2] = v > 255 ? 255 : v;
    } else {
        int v;
        v = o[0] - pr; o[0] = v < 0 ? 0 : v; v = o[1] - pg; o[1] = v < 0 ? 0 : v;
        v = o[2] - pb; o[2] = v < 0 ? 0 : v;
    }
}
#include <math.h>
static void paint(void) {
    int top = overscan[0] > 0 ? overscan[0] : 0, right = overscan[1] > 0 ? overscan[1] : 0;
    int bottom = overscan[2] > 0 ? overscan[2] : 0, left = overscan[3] > 0 ? overscan[3] : 0;
    if (!IMG) IMG = malloc(480 * 512 * 3);
    memset(IMG, 0, 480 * 512 * 3);
    size_t pc = 0;
    for (int y = top; y < 240 - bottom; y++) {
        for (;;) {
            uint64_t c = DLB[pc++];
            int op = c & 3;
            if (op == 3) break;
            int x0 = (c >> 2) & 0xFF, len = (int)((c >> 10) & 0xFF) + 1;
            if (op == 1) {
                uint32_t rgb = (c >> 20) & 0xFFFFFF;
                for (int r = 0; r < 2; r++)
                    for (int j = 0; j < 2 * len; j++) {
                        uint8_t *o = IMG + ((2 * y + r) * 512 + 2 * x0 + j) * 3;
                        o[0] = rgb >> 16; o[1] = rgb >> 8; o[2] = rgb;
                    }
            } else if (op == 0) {
                int mode = (c >> 18) & 3, offy = (c >> 20) & 7, vm = (c >> 23) & 1, hm = (c >> 24) & 1;
                int offx0 = (c >> 25) & 7; uint32_t blk = (uint32_t)(c >> 28) & 0xFFFFFF;
                const uint8_t *B = D + SEC[4] + (uint64_t)blk * 1024;
                for (int j = 0; j < len; j++) {
                    int offx = offx0 + j, col = hm ? 7 - offx : offx;
                    for (int r = 0; r < 2; r++)
                        for (int cc = 0; cc < 2; cc++) {
                            int sr = offy * 2 + (vm ? 1 - r : r), sc = col * 2 + (hm ? 1 - cc : cc);
                            blend(IMG + ((2 * y + r) * 512 + 2 * (x0 + j) + cc) * 3, B + (sr * 16 + sc) * 4, mode);
                        }
                }
            } else {
                int gray = (c >> 2) & 1, emph = (c >> 3) & 7;
                for (int r = 0; r < 2; r++)
                    for (int xx = left * 2; xx < (256 - right) * 2; xx++) {
                        uint8_t *o = IMG + ((2 * y + r) * 512 + xx) * 3;
                        if (gray) { int avg = (o[0] + o[1] + o[2]) / 3; o[0] = o[1] = o[2] = (uint8_t)avg; }
                        if (emph) {
                            double rf = 1.0, gf = 1.0, bf = 1.0;
                            if (emph & 1) { rf *= 1.1; gf *= 0.9; bf *= 0.9; }
                            if (emph & 2) { gf *= 1.1; rf *= 0.9; bf *= 0.9; }
                            if (emph & 4) { bf *= 1.1; rf *= 0.9; gf *= 0.9; }
                            double v;
                            v = floor(o[0] * rf); o[0] = (uint8_t)(v > 255 ? 255 : v);
                            v = floor(o[1] * gf); o[1] = (uint8_t)(v > 255 ? 255 : v);
                            v = floor(o[2] * bf); o[2] = (uint8_t)(v > 255 ? 255 : v);
                        }
                    }
            }
        }
    }
}
static void write_img(const char *path) {
    int top = overscan[0] > 0 ? overscan[0] : 0, right = overscan[1] > 0 ? overscan[1] : 0;
    int bottom = overscan[2] > 0 ? overscan[2] : 0, left = overscan[3] > 0 ? overscan[3] : 0;
    int W = (256 - left - right) * 2, H = (240 - top - bottom) * 2;
    FILE *f = fopen(path, "wb"); if (!f) { perror(path); exit(1); }
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    for (int yy = top * 2; yy < top * 2 + H; yy++) fwrite(IMG + (yy * 512 + left * 2) * 3, 1, (size_t)W * 3, f);
    fclose(f);
}

/* ---------------------------------------------------------------- files */
static uint8_t *load(const char *path, size_t *len) {
#ifdef _WIN32
    FILE *f = fopen(path, "rb"); if (!f) { perror(path); exit(1); }
    fseek(f, 0, SEEK_END); *len = ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *b = malloc(*len); if (fread(b, 1, *len, f) != *len) { perror(path); exit(1); }
    fclose(f); return b;
#else
    /* plain read: simplest, and loading is outside the timed region. (An earlier mmap version
       "failed" only because fstat had failed and left the size 0; see _FILE_OFFSET_BITS.) */
    int fd = open(path, O_RDONLY); if (fd < 0) { perror(path); exit(1); }
    struct stat st;
    if (fstat(fd, &st) != 0) { perror(path); exit(1); }
    *len = (size_t)st.st_size;
    uint8_t *b = malloc(*len ? *len : 1);
    if (!b) { perror("malloc"); exit(1); }
    size_t got = 0;
    while (got < *len) {
        ssize_t k = read(fd, b + got, *len - got);
        if (k <= 0) { perror(path); exit(1); }
        got += (size_t)k;
    }
    close(fd); return b;
#endif
}

static int cmp_d(const void *a, const void *b) {
    double x = *(const double *)a, y = *(const double *)b; return x < y ? -1 : x > y;
}
#endif /* !HDARM_LIB */

static double now(void) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/* ---- pack setup and the per-frame work, shared by main() and the library entry points ---- */
static int hd_setup(size_t dl) {
    crc_init();
    if (memcmp(D, "NHDP", 4) || (D[4] | D[5] << 8) != 3) {
        fprintf(stderr, "not a format-3 .hdp (%zu bytes, starts %02x %02x %02x %02x %02x %02x)\n",
                dl, D[0], D[1], D[2], D[3], D[4], D[5]);
        return 1;
    }
    H_flags = D[7]; H_options = rd32(D + 12);
    for (int i = 0; i < 4; i++) overscan[i] = (int32_t)rd32(D + 0x10 + 4 * i);
    nbuckets = rd32(D + 0x24); max_probe = rd32(D + 0x28); nconds = rd32(D + 0x2C);
    nwatched = rd32(D + 0x34); nadd = rd32(D + 0x40); nfallback = rd32(D + 0x44);
    for (int i = 0; i < 18; i++) SEC[i] = rd32(D + 0x80 + 8 * i);
    legacy = H_flags & 1;
    if (!(H_flags & 0x20)) { fprintf(stderr, "pack does not say whether the game is CHR-RAM\n"); return 1; }
    game_ram = (H_flags & 0x10) != 0;
    cache_enabled = (H_options & 8) == 0;
    render_original = (H_options & 16) == 0;
    for (int i = 0; i < 64; i++) PRGB[i] = (H_flags & 2) ? (rd32(D + SEC[12] + 4 * i) & 0xFFFFFF) : MESEN_2C02[i];
    AGT = calloc(AG_N, sizeof(AG));
    has_fb = 0;
    for (uint32_t i = 0; i < nfallback; i++) if ((int32_t)rd32(D + SEC[10] + 4 * i) >= 0) { has_fb = 1; break; }
    ADDS = calloc(nadd ? nadd : 1, sizeof(Add));
    for (uint32_t i = 0; i < nadd; i++) {
        const uint8_t *a = D + SEC[11] + 64 * (uint64_t)i;
        uint32_t okind = rd32(a + 4), akind = rd32(a + 20);
        uint64_t obody = rd64(a + 8), abody = rd64(a + 24);
        ADDS[i].opal = rd32(a); ADDS[i].oidx = okind == 0 ? (int64_t)obody : NOTILE; ADDS[i].ofp = okind == 1 ? obody : 0;
        ADDS[i].apal = rd32(a + 16); ADDS[i].aidx = akind == 0 ? (int64_t)abody : NOTILE; ADDS[i].afp = akind == 1 ? abody : 0;
        ADDS[i].ox = (int32_t)rd32(a + 32); ADDS[i].oy = (int32_t)rd32(a + 36); ADDS[i].ignore = rd32(a + 40) & 1;
        if (akind == 0 && (ADDS[i].aidx < INT32_MIN || ADDS[i].aidx > INT32_MAX)) {
            fprintf(stderr, "addition %u: tile index does not fit 32 bits\n", i); return 1;
        }
    }
    AXT = calloc(nadd ? nadd : 1, sizeof(AX));
    for (uint32_t i = 0; i < nadd; i++) {
        AXT[i].body = game_ram ? ADDS[i].ofp : (uint64_t)ADDS[i].oidx; AXT[i].a = i;
    }
    qsort(AXT, nadd, sizeof(AX), cmp_ax);
    SBI = calloc(61440 + 16, sizeof(uint16_t));   /* +16: see PID */
    SB = malloc(sizeof(SBlk) * (61440 + 256 + 2));   /* every pixel, plus one copy per line */
    FMT = calloc(FM_N, sizeof(FM)); GMT = calloc(GM_N, sizeof(GM)); CLT = calloc(CL_N, sizeof(CL));
    RLT = calloc(RL_N, sizeof(RL));
    MKF = malloc((size_t)MK_MAX * NREC);
    MEMO = malloc(nconds ? nconds : 1);
    return 0;
}

#ifdef SUBT
static double SUBT_A[4]; static long SUBT_N;
#endif
/* One frame: every decision, then the display list into DLB/DLn. ts (6 doubles, may be NULL)
   gets the phase boundaries for the timing report. */
static void hd_frame(const uint8_t *rec, int delta, double *ts) {
    OBn = 0;
    st_rules = st_conds = st_fb = st_add = 0;
    st_fm_hit = st_gm_hit = st_gm_static = st_gm_dynamic = 0;
    if (ts) ts[0] = now();
    G++; FMcount = GMcount = 0; SBn = 0; REn = 0; RUNn = 0; RDn = 0; MKn = 0; st_rl_kept = st_rl_dropped = 0; AGnext = 0;
    st_cmds = st_cyc_lines = 0;
    FRAME = (uint32_t)(rd64(rec + 0xF0000) & 0xFFFFFFFF) + delta;
#ifdef SUBT   /* timing study: the records phase split into its parts (SUBT_A, see main) */
    double s0 = now();
#endif
    memset(MEMO, 0xFF, nconds ? nconds : 1);
    add_cache_reset();
#ifdef SUBT
    double s1 = now();
#endif
    build_records(rec);
#ifdef SUBT
    double s2 = now();
#endif
    pid_build();
#ifdef SUBT
    double s3 = now();
    SUBT_A[0] += s1 - s0; SUBT_A[1] += s2 - s1; SUBT_A[2] += s3 - s2; SUBT_N++;
#endif
    if (ts) ts[1] = now();
    outf("F %u\n", FRAME);
    select_levels();
    if (ts) ts[2] = now();
    process_additions();
    if (ts) ts[3] = now();
    DLn = 0;
    lookups();                    /* decisions AND the display list, line by line */
    if (ts) ts[4] = ts[5] = now();
}

#ifdef HDARM_LIB
/* ---- library entry points: the ARM program and the full-core simulation ----
   hdarm_init: the pack image (the whole .hdp, or at least everything but the block section's
   pixels), kept by reference. hdarm_frame: one frame record (the 1 MB bank, as hd_capture wrote
   it) and the watched-address values (one byte per pack slot); returns the display list. */
int hdarm_init(const uint8_t *pack, size_t len) {
    D = pack;
    return hd_setup(len);
}
uint32_t hdarm_nwatched(void) { return nwatched; }
int hdarm_frame(const uint8_t *rec, const uint8_t *vals, uint32_t nvals,
                const uint64_t **list, size_t *nq) {
    if (nwatched) {
        if (nvals != nwatched) return 1;
        memcpy(SNAP, vals, nvals > 256 ? 256 : nvals);
    }
    hd_frame(rec, 0, NULL);
    *list = DLB; *nq = DLn;
    return 0;
}
#else
int main(int argc, char **argv) {
    if (argc < 4) { fprintf(stderr, "usage: hdarm PACK.hdp REC.bin WATCH.bin [--delta D] [--iters N] [--out F]\n"); return 2; }
    { const char *e = getenv("HD_FEATOFF"); FEATOFF = e ? (unsigned)strtoul(e, NULL, 0) : 0; }
    int delta = 0, iters = 1; const char *outp = NULL, *imgp = NULL, *dlp = NULL;
    for (int i = 4; i < argc; i++) {
        if (!strcmp(argv[i], "--img") && i + 1 < argc) imgp = argv[++i];
        else if (!strcmp(argv[i], "--dl") && i + 1 < argc) dlp = argv[++i];
        else if (!strcmp(argv[i], "--delta") && i + 1 < argc) delta = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--iters") && i + 1 < argc) iters = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--out") && i + 1 < argc) outp = argv[++i];
        else { fprintf(stderr, "bad arg %s\n", argv[i]); return 2; }
    }
    size_t dl, rl, wl;
    D = load(argv[1], &dl);
    const uint8_t *rec = load(argv[2], &rl);
    const uint8_t *wat = load(argv[3], &wl);
    if (hd_setup(dl)) return 1;
    if (rl < 0xF0008) { fprintf(stderr, "short frame record\n"); return 1; }
    /* watch snapshot, indexed by the pack's slot numbers */
    if (memcmp(wat, "NWAT", 4)) { fprintf(stderr, "watch file: bad magic\n"); return 1; }
    uint32_t wn = rd32(wat + 4);
    if (nwatched) {
        if (wn != nwatched) { fprintf(stderr, "watch list differs\n"); return 1; }
        for (uint32_t i = 0; i < wn; i++)
            if (rd32(wat + 8 + 4 * i) != rd32(D + SEC[9] + 4 * i)) { fprintf(stderr, "watch list differs\n"); return 1; }
        memcpy(SNAP, wat + 8 + 4 * wn, wn > 256 ? 256 : wn);
    }

    double *tb = calloc(iters, sizeof(double)), *tl = calloc(iters, sizeof(double)),
           *ta = calloc(iters, sizeof(double)), *tk = calloc(iters, sizeof(double)),
           *tt = calloc(iters, sizeof(double)), *tg = calloc(iters, sizeof(double));
    long r_rules = 0, r_conds = 0, r_fb = 0, r_add = 0;
    for (int it = 0; it < iters; it++) {
#ifndef HD_NOOUT
        OUT_ON = (it == 0 && outp != NULL);
#else
        if (outp) { fprintf(stderr, "--out needs a build without HD_NOOUT\n"); return 2; }
#endif
        double ts[6];
        hd_frame(rec, delta, ts);
#ifdef SUBT
        if (it == 0) { memset(SUBT_A, 0, sizeof SUBT_A); SUBT_N = 0; }   /* warm iterations only */
#endif
        tb[it] = ts[1] - ts[0]; tl[it] = ts[2] - ts[1]; ta[it] = ts[3] - ts[2]; tk[it] = ts[4] - ts[3];
        tg[it] = ts[5] - ts[4]; tt[it] = ts[5] - ts[0];
        if (it == 0 && imgp) { paint(); write_img(imgp); }
        if (it == 0 && dlp) {
            /* the display list as the FPGA reads it: header {blocks section offset; word count,
               right, left, bottom, top}, then the commands, little-endian u64 */
            FILE *f = fopen(dlp, "wb"); if (!f) { perror(dlp); return 1; }
#define OVS(i) ((uint64_t)(overscan[i] > 0 ? overscan[i] : 0))
            uint64_t hdr[2] = { SEC[4], (uint64_t)DLn << 32 | OVS(1) << 24 | OVS(3) << 16 | OVS(2) << 8 | OVS(0) };
#undef OVS
            fwrite(hdr, 8, 2, f); fwrite(DLB, 8, DLn, f); fclose(f);
        }
        if (it == 0) {
            fprintf(stderr, "display list: %ld commands (%zu words), %ld lines needed per-pixel order\n",
                    st_cmds, DLn, st_cyc_lines);
            /* painter load per NES line: commands, and NES pixels painted (BLIT / FILL) */
            int mc = 0, mb = 0, mf = 0, n = 0, b = 0, fl2 = 0; long tb2 = 0;
            for (size_t i = 0; i < DLn; i++) {
                uint64_t c = DLB[i]; int op = c & 3, len = (int)((c >> 10) & 0xFF) + 1;
                if (op == 3) { if (n > mc) mc = n; if (b > mb) mb = b; if (fl2 > mf) mf = fl2; n = b = fl2 = 0; continue; }
                n++;
                if (op == 0) { b += len; tb2 += len; } else if (op == 1) fl2 += len;
            }
            fprintf(stderr, "per line max: %d commands, %d blit px, %d fill px; blit px per frame %ld\n", mc, mb, mf, tb2);
            /* block-row fetches (128 B each: the 2 HD rows of one tile row) with a direct-mapped cache */
            static const int csz[4] = {0, 64, 256, 1024};
            for (int ci = 0; ci < 4; ci++) {
                static uint32_t tag[1024]; long miss = 0, nb = 0; int lm = 0, mlm = 0;
                for (int k = 0; k < 1024; k++) tag[k] = 0xFFFFFFFFu;
                for (size_t i = 0; i < DLn; i++) {
                    uint64_t c = DLB[i];
                    if ((c & 3) == 3) { if (lm > mlm) mlm = lm; lm = 0; continue; }
                    if ((c & 3) != 0) continue;
                    uint32_t key = (uint32_t)((c >> 28) & 0xFFFFFF) << 3 | (uint32_t)((c >> 20) & 7);
                    nb++;
                    if (!csz[ci]) { miss++; lm++; continue; }
                    uint32_t s = (key ^ (key >> 7)) & (csz[ci] - 1);
                    if (tag[s] != key) { tag[s] = key; miss++; lm++; }
                }
                fprintf(stderr, "  block-row cache %4d entries: %ld fetches of %ld blits (%.2f MB/s at 60 fps), max %d in one line\n",
                        csz[ci], miss, nb, miss * 128.0 * 60 / 1e6, mlm);
            }
        }
        if (it == 0) { r_rules = st_rules; r_conds = st_conds; r_fb = st_fb; r_add = st_add; }
        if (it == iters - 1)
            fprintf(stderr, "memo find_hits %ld match_hits %ld walked_static %ld walked_dynamic %ld sprite_blocks %u"
                    " reduced_kept %ld reduced_dropped %ld dep_reuse %ld dep_rejected %ld index_rewrites %d spr_reuse %ld vert_reuse %ld\n",
                    st_fm_hit, st_gm_hit, st_gm_static, st_gm_dynamic, SBn, st_rl_kept, st_rl_dropped, st_dep_hit, st_dep_rej,
                    ANY_MUT, st_spr_hit, st_vm_hit);
#ifdef MEMO_VERIFY
        if (it == iters - 1) fprintf(stderr, "MEMO_VERIFY checked %ld failed %ld\n", memo_checked, memo_failed);
#endif
        if (OUT_ON) {
            FILE *f = fopen(outp, "wb"); if (!f) { perror(outp); return 1; }
            fwrite(OB, 1, OBn, f); fclose(f);
        }
    }
    fprintf(stderr, "rules_walked %ld conds_evaluated %ld fallback_hits %ld additions %ld\n",
            r_rules, r_conds, r_fb, r_add);
    fprintf(stderr, "dispatch: qualifying entries %ld, runs %u (last frame), lookups %ld (all frames)\n",
            st_disp_q, RUNn, st_disp);
#ifdef TPROF
    fprintf(stderr, "TPROF per iteration (Mcycles): tile_rec %.3f  BG get_matching %.3f (%lld calls, %lld cached px)  sprites %.3f\n",
            TP[0] / 1e6 / iters, TP[1] / 1e6 / iters, TP[3] / iters, TP[4] / iters, TP[2] / 1e6 / iters);
#endif
#ifdef CSTATS
    { extern long CST[16][2];
      for (int t = 0; t < 16; t++) if (CST[t][0] || CST[t][1])
          fprintf(stderr, "cond type %2d evaluated %ld memo-hit %ld\n", t, CST[t][0], CST[t][1]); }
#endif
#ifdef PATHSTAT
    {   /* per frame (the counters ran on every iteration) */
        double f = iters;
        long walked = PS[0] + PS[7];
        fprintf(stderr, "PATHSTAT per frame: pixels %.0f = loop body %.0f + fast run %.0f\n", walked / f, PS[0] / f, PS[7] / f);
        fprintf(stderr, "  body: no tile %.0f  cached in body %.0f  line-memo reuse %.0f (dis %.0f)  column-memo %.0f  walk %.0f (dis %.0f)  sprite px %.0f\n",
                PS[1] / f, PS[2] / f, PS[3] / f, PS[10] / f, PS[5] / f, PS[6] / f, PS[9] / f, PS[8] / f);
        fprintf(stderr, "  fast run stops: line end %.0f  tile boundary %.0f  sprite %.0f  span change %.0f  backdrop %.0f  run/offx %.0f\n",
                PS[11] / f, PS[12] / f, PS[13] / f, PS[14] / f, PS[15] / f, PS[16] / f);
        fprintf(stderr, "  per-tile dependency cache: enabled %.0f times, covering up to %.0f pixels\n", PS[17] / f, PS[18] / f);
        fprintf(stderr, "  fast-run pixels in HD blocks %.0f, in original/none tiles %.0f\n", PS[23] / f, (PS[7] - PS[23]) / f);
        fprintf(stderr, "  run/offx stops: HD offx %.0f  colour change %.0f  opaque-no-run %.0f  transparent-after-run %.0f\n",
                PS[19] / f, PS[20] / f, PS[21] / f, PS[22] / f);
    }
#endif
#ifdef SUBT
    if (SUBT_N)
        fprintf(stderr, "SUBT mean of %ld warm frames (ms): memo_clear %.3f  build_records %.3f  pid+cid %.3f  (nconds %u, game_ram %d)\n",
                SUBT_N, SUBT_A[0] * 1e3 / SUBT_N, SUBT_A[1] * 1e3 / SUBT_N, SUBT_A[2] * 1e3 / SUBT_N, nconds, game_ram);
#endif
    if (iters > 1) {
        double *v[6] = {tb, tl, ta, tk, tg, tt};
        /* the display list is generated inside lookups, line by line: "lookups" holds both */
        const char *nm[6] = {"records", "levels", "additions", "lookups+dl", "(in lookups)", "TOTAL"};
        for (int k = 0; k < 6; k++) {
            qsort(v[k], iters, sizeof(double), cmp_d);
            fprintf(stderr, "%-10s min %7.3f ms  median %7.3f ms  max %7.3f ms\n", nm[k],
                    v[k][0] * 1e3, v[k][iters / 2] * 1e3, v[k][iters - 1] * 1e3);
        }
    }
    return 0;
}
#endif /* HDARM_LIB */
