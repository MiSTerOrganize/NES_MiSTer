/* aud_arm.c - the ARM half of stage-6 HD audio (see aud_arm.h). The device is hdaudio.Device
 * (tools/audio/hdaudio.py) line for line: the register semantics of Mesen's HdAudioDevice, its
 * voices, and OggMixer's order; the ADPCM decoder is the one the FPGA used to run
 * (hd3_audio.sv until 2026-10-28; test_ima.py checks hdaudio's against the encoder's).
 *
 * The one thing a restore needs that the log does not carry is the BGM's ADPCM state at its
 * saved frame. The state at a frame is the state after decoding the stream from frame 0 up to it
 * (a loop restarts with the converter's loop state, which is the state the first pass had at
 * the loop frame), so it is decoded here, from the nearest checkpoint: every AUD_CK frames of an
 * entry, the state is kept the first time a voice passes it.
 *
 * The section is NOT copied into ARM memory (one pack's can be over 100 MB): a pack reset reads
 * its header and entry table, and each voice reads its stream AUD_BLK bytes at a time from the
 * pack in DDR3 through io->fetch.
 */
#include <stdlib.h>
#include <string.h>
#include "aud_arm.h"

static const int STEP[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45, 50, 55, 60,
    66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371,
    408, 449, 494, 544, 598, 658, 724, 796, 876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878,
    2066, 2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845,
    8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086,
    29794, 32767};
static const int IDX[8] = {-1, -1, -1, -1, 2, 4, 6, 8};

static int clamp16(int v) { return v < -32768 ? -32768 : v > 32767 ? 32767 : v; }

static void ima(int *pred, int *idx, int n) {
    int step = STEP[*idx], diff = step >> 3;
    if (n & 4) diff += step;
    if (n & 2) diff += step >> 1;
    if (n & 1) diff += step >> 2;
    *pred = clamp16(n & 8 ? *pred - diff : *pred + diff);
    int i = *idx + IDX[n & 7];
    *idx = i < 0 ? 0 : i > 88 ? 88 : i;
}

/* Mesen: (int32_t)(sample * volume / 255), the division truncating toward zero */
static int vol(int x, int v) {
    int p = x * v, q = (p < 0 ? -p : p) / 255;
    return p < 0 ? -q : q;
}

static uint32_t le32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }
static uint64_t le64(const uint8_t *p) { return le32(p) | (uint64_t)le32(p + 4) << 32; }

static void unpack(uint64_t s, aud_voice *v) {
    v->pl = (int16_t)(s & 0xFFFF); v->il = (s >> 16) & 0x7F;
    v->pr = (int16_t)((s >> 32) & 0xFFFF); v->ir = (s >> 48) & 0x7F;
}
static uint64_t pack(const aud_voice *v) {
    return (uint64_t)(v->pl & 0xFFFF) | (uint64_t)v->il << 16 |
           (uint64_t)(v->pr & 0xFFFF) << 32 | (uint64_t)v->ir << 48;
}

/* section byte off, through the voice's block (fetched from the pack when off leaves it);
   0 past the section, and 0 for a block that cannot be fetched (counted) */
static int sec_byte(aud_arm *a, aud_voice *v, uint32_t off) {
    if (off >= a->sec_len) return 0;
    if (v->blk_at == UINT32_MAX || off - v->blk_at >= AUD_BLK) {
        uint32_t at = off & ~(AUD_BLK - 1), n = a->sec_len - at < AUD_BLK ? a->sec_len - at : AUD_BLK;
        if (!a->io || a->io->fetch(a->io->ctx, a->sec_off + at, n, v->blk)) {
            memset(v->blk, 0, AUD_BLK);
            a->fetch_fail++;
            v->bad = 1;
        }
        v->blk_at = at;
    }
    return v->blk[off - v->blk_at];
}

/* decode the frame at v->pos, advance, keep a checkpoint at a multiple of AUD_CK */
static void decode_one(aud_arm *a, aud_voice *v) {
    aud_ent *e = &a->ent[v->e];
    int b = sec_byte(a, v, e->start + v->pos);
    ima(&v->pl, &v->il, b & 15);
    ima(&v->pr, &v->ir, b >> 4);
    v->pos++;
    if (v->pos % AUD_CK == 0 && !v->bad) {
        uint32_t k = v->pos / AUD_CK;
        if (k < e->nck && !e->ckv[k]) { e->ck[k] = pack(v); e->ckv[k] = 1; }
    }
}

/* hdaudio.Voice.next: 0 after the last frame of a stream that does not loop */
static int voice_next(aud_arm *a, aud_voice *v, int *l, int *r) {
    aud_ent *e = &a->ent[v->e];
    if (v->pos >= e->frames) {
        if (!v->loop) return 0;
        v->pos = e->loop;
        unpack(e->sl, v);
    }
    decode_one(a, v);
    *l = v->pl; *r = v->pr;
    return 1;
}

static void voice_start(aud_arm *a, aud_voice *v, int e, int loop) {
    v->on = 1; v->e = e; v->loop = loop; v->pos = 0;
    v->blk_at = UINT32_MAX;
    v->bad = 0;
    unpack(a->ent[e].s0, v);
}

static void power_on(aud_arm *a) {
    a->album = 0; a->options = 0; a->paused = 0; a->bvol = 128; a->svol = 128;
    a->bgm.on = 0; a->n_sfx = 0;
}

static void free_section(aud_arm *a) {
    for (int i = 0; i < a->n_ent; i++) { free(a->ent[i].ck); free(a->ent[i].ckv); }
    a->sec_off = a->sec_len = 0; a->n_ent = 0;
    a->ckb_e = 0; a->ckb.on = 0; a->ck_loaded = 0;
}

void aud_init(aud_arm *a) { memset(a, 0, sizeof *a); power_on(a); }
void aud_free(aud_arm *a) { free_section(a); }

/* The section's checkpoint table (hdaudio.build_section, since 2026-10-29): at section byte
   ck_off {u32 spacing, u32 count, count x u64}, every entry's states before frame k * spacing in
   table order. With it a restore at any frame decodes at most AUD_CK - 1 frames from the moment
   the pack loads; without it (ck_off 0: a section built earlier) the background builder makes
   them. A table that does not fit this section exactly - another spacing, a count that is not
   the entries' sum, past the end, a first state that is not the entry's state 0, an index past
   88 - is not used at all (a wrong state would play noise; the builder is always right). */
static void load_checkpoints(aud_arm *a, const aud_io *io, uint32_t ck_off) {
    uint8_t h[8];
    if (!ck_off || ck_off > a->sec_len || a->sec_len - ck_off < 8 ||
        io->fetch(io->ctx, a->sec_off + ck_off, 8, h))
        return;
    uint32_t spacing = le32(h), count = le32(h + 4), want = 0;
    for (int i = 0; i < a->n_ent; i++) want += a->ent[i].nck;
    if (spacing != AUD_CK || count != want || (a->sec_len - ck_off - 8) / 8 < count) return;
    uint8_t *t = malloc((size_t)count * 8);
    if (!t) return;
    if (io->fetch(io->ctx, a->sec_off + ck_off + 8, count * 8, t)) { free(t); return; }
    uint32_t at = 0;
    for (int i = 0; i < a->n_ent; i++) {
        if (le64(t + 8 * at) != a->ent[i].s0) { free(t); return; }
        for (uint32_t k = 0; k < a->ent[i].nck; k++) {
            uint64_t s = le64(t + 8 * (at + k));
            if (((s >> 16) & 0xFFFF) > 88 || ((s >> 48) & 0xFFFF) > 88) { free(t); return; }
        }
        at += a->ent[i].nck;
    }
    at = 0;
    for (int i = 0; i < a->n_ent; i++) {
        aud_ent *e = &a->ent[i];
        for (uint32_t k = 0; k < e->nck; k++) { e->ck[k] = le64(t + 8 * (at + k)); e->ckv[k] = 1; }
        at += e->nck;
    }
    free(t);
    a->ckb_e = a->n_ent;                 /* nothing left for the builder */
    a->ck_loaded = 1;
}

/* the section at pack file bytes [off, off + len): its header and entry table now, the streams
   as they play; 0 = valid */
int aud_load_section(aud_arm *a, const aud_io *io, uint32_t off, uint32_t len) {
    free_section(a);
    power_on(a);
    a->io = io;
    uint8_t h[16];
    if (len < 16 || io->fetch(io->ctx, off, 16, h) || memcmp(h, "HDAU", 4)) return 1;
    uint32_t n = le32(h + 4);
    if (n == 0 || n > AUD_MAXENT || 16 + 32ull * n > len) return 1;
    uint8_t *tab = malloc(32 * n);
    if (!tab) return 1;
    if (io->fetch(io->ctx, off + 16, 32 * n, tab)) { free(tab); return 1; }
    a->sec_off = off;
    a->sec_len = len;
    for (uint32_t i = 0; i < n; i++) {
        const uint8_t *p = tab + 32 * i;
        aud_ent *e = &a->ent[i];
        e->key = le32(p); e->start = le32(p + 4); e->frames = le32(p + 8); e->loop = le32(p + 12);
        e->s0 = le64(p + 16); e->sl = le64(p + 24);
        /* one byte per frame: a stream must lie inside the section (a table that says
           otherwise is not one the converter wrote - refuse the section, do not play garbage) */
        if (e->start > len || e->frames > len - e->start) {
            a->n_ent = (int)i; free_section(a); free(tab); return 1;
        }
        e->nck = e->frames / AUD_CK + 1;
        e->ck = calloc(e->nck, sizeof *e->ck);
        e->ckv = calloc(e->nck, 1);
        if (!e->ck || !e->ckv) { a->n_ent = (int)i + 1; free_section(a); free(tab); return 1; }
        e->ck[0] = e->s0; e->ckv[0] = 1;
        a->n_ent = (int)i + 1;
    }
    free(tab);
    load_checkpoints(a, io, le32(h + 12));
    return 0;
}

/* the entry with this key, or -1 (the keys are sorted: hdaudio.build_section) */
static int find(const aud_arm *a, uint32_t key) {
    int lo = 0, hi = a->n_ent;
    while (lo < hi) {
        int m = (lo + hi) >> 1;
        if (a->ent[m].key == key) return m;
        if (a->ent[m].key < key) lo = m + 1; else hi = m;
    }
    return -1;
}

void aud_write(aud_arm *a, int reg, int v) {
    switch (reg) {
    case 0: a->options = v; if (a->bgm.on) a->bgm.loop = v & 1; break;
    case 1:
        a->paused = v & 1;
        if (v & 2) a->bgm.on = 0;
        if (v & 4) a->n_sfx = 0;
        break;
    case 2: a->bvol = v; break;
    case 3: a->svol = v; break;
    case 4: a->album = v; break;
    case 5: {
        int e = find(a, (uint32_t)(a->album * 256 + v));
        if (e >= 0 && a->ent[e].frames > 0) voice_start(a, &a->bgm, e, a->options & 1);
        break;
    }
    case 6: {
        int e = find(a, 0x10000u | (uint32_t)(a->album * 256 + v));
        if (e >= 0 && a->ent[e].frames > 0) {
            if (a->n_sfx >= AUD_SFX) {               /* replace the oldest */
                memmove(&a->sfx[0], &a->sfx[1], sizeof a->sfx[0] * (AUD_SFX - 1));
                a->n_sfx--;
            }
            voice_start(a, &a->sfx[a->n_sfx++], e, 0);
        }
        break;
    }
    default: break;
    }
}

/* hdaudio.Device.load: the registers, and the BGM at its saved frame or none */
void aud_restore(aud_arm *a, uint64_t w) {
    a->album = (w >> 56) & 0xFF;
    a->bvol = (w >> 48) & 0xFF;
    a->svol = (w >> 40) & 0xFF;
    a->options = (w >> 39) & 1;
    int idx = (w >> 29) & 0x1FF;
    if (((w >> 38) & 1) && idx < a->n_ent) {
        aud_voice *v = &a->bgm;
        aud_ent *e = &a->ent[idx];
        uint32_t pos = (uint32_t)(w & 0x1FFFFFFF), p = pos < e->frames ? pos : e->frames;
        uint32_t k = p / AUD_CK;
        while (k > 0 && !e->ckv[k]) k--;
        v->on = 1; v->e = idx; v->loop = a->options & 1; v->pos = k * AUD_CK;
        v->blk_at = UINT32_MAX;
        v->bad = 0;
        unpack(e->ck[k], v);
        while (v->pos < p) decode_one(a, v);
        v->pos = pos;
    } else {
        a->bgm.on = 0;
    }
}

/* The checkpoint builder: each stream decoded once from frame 0, so a checkpoint exists every
   AUD_CK frames of every stream, not only where a voice has already passed. Without it a
   restore far into a stream not yet played this session decoded from frame 0 (a five-minute
   BGM is ~14 million frames - far past the tick deadline). It changes no output: a checkpoint
   is the stream's own state at that frame, wherever the decoding started. */
int aud_build_checkpoints(aud_arm *a, uint32_t budget) {
    aud_voice *v = &a->ckb;
    while (budget && a->ckb_e < a->n_ent) {
        aud_ent *e = &a->ent[a->ckb_e];
        if (!v->on || v->e != a->ckb_e) {
            /* start from the highest checkpoint a voice has already left behind */
            uint32_t k = e->nck - 1;
            while (k > 0 && !e->ckv[k]) k--;
            v->on = 1; v->e = a->ckb_e; v->loop = 0; v->pos = k * AUD_CK;
            v->blk_at = UINT32_MAX; v->bad = 0;
            unpack(e->ck[k], v);
        }
        /* the last checkpoint this stream can have is at (nck - 1) * AUD_CK */
        uint32_t end = (e->nck - 1) * AUD_CK;
        if (v->pos >= end || v->bad) {
            a->ckb_e++; v->on = 0;
            continue;
        }
        decode_one(a, v);
        budget--;
    }
    return a->ckb_e < a->n_ent;
}

void aud_tick(aud_arm *a, uint32_t out[AUD_NV]) {
    int l, r;
    memset(out, 0, sizeof(uint32_t) * AUD_NV);
    if (a->bgm.on && !a->paused) {
        if (!voice_next(a, &a->bgm, &l, &r)) {
            a->bgm.on = 0;
        } else {
            out[0] = (uint32_t)(vol(r, a->bvol) & 0xFFFF) << 16 | (vol(l, a->bvol) & 0xFFFF);
            if (a->bgm.pos >= a->ent[a->bgm.e].frames && !a->bgm.loop) a->bgm.on = 0;
        }
    }
    /* in place (a voice carries its stream block, so it is copied only when one ends) */
    int keep = 0;
    for (int i = 0; i < a->n_sfx; i++) {
        aud_voice *v = &a->sfx[i];
        if (!voice_next(a, v, &l, &r)) continue;
        out[1 + i] = (uint32_t)(vol(r, a->svol) & 0xFFFF) << 16 | (vol(l, a->svol) & 0xFFFF);
        if (v->pos < a->ent[v->e].frames) {
            if (keep != i) a->sfx[keep] = *v;
            keep++;
        }
    }
    a->n_sfx = keep;
}

/* ------------------------------------------------------------------- the log */
static void apply(aud_arm *a, const aud_io *io, const aud_event *ev) {
    switch (ev->kind) {
    case 1: aud_write(a, (int)(ev->payload >> 8) & 7, (int)(ev->payload & 0xFF)); break;
    case 2: aud_restore(a, ev->payload); break;
    case 3: {
        uint32_t off = (uint32_t)(ev->payload & 0x1FFFFFFF), len = (uint32_t)(ev->payload >> 32);
        a->resets++;
        if (len && aud_load_section(a, io, off, len)) { free_section(a); power_on(a); a->fetch_fail++; }
        else if (!len) { free_section(a); power_on(a); }
        break;
    }
    case 4:
        /* the device's state at this tick: what this side already holds unless it started
           late, at this event (aud_service) */
        if (a->want_sync) {
            aud_restore(a, ev->payload);
            a->paused = ev->flags & 1;
            a->want_sync = 0;
        }
        break;
    default: a->bad_events++; break;
    }
    a->events++;
}

/* Started late (or the first event is not a fresh session's pack reset): the section the pack
   in DDR3 names (its header, section 15 at 0xF8/0xFC) is loaded now, so the device plays the
   writes that follow even when no pack reset is among the events still in the log. */
static void late_load(aud_arm *a, const aud_io *io) {
    uint8_t h[8];
    if (io->fetch(io->ctx, 0, 4, h) || memcmp(h, "NHDP", 4)) return;
    if (io->fetch(io->ctx, 0xF8, 8, h)) return;
    uint32_t off = le32(h), len = le32(h + 4);
    if (len && aud_load_section(a, io, off & 0x1FFFFFFF, len)) { free_section(a); power_on(a); a->fetch_fail++; }
}

/* the status qword, read until two reads agree (ARMv7 does not promise a 64-bit read is
   single-copy atomic, and a torn {T, E} would render past events not yet read) */
static uint64_t read_status(const aud_io *io) {
    uint64_t s = io->rd(io->ctx, AUD_ST_Q);
    for (int i = 0; i < 4; i++) {
        uint64_t s2 = io->rd(io->ctx, AUD_ST_Q);
        if (s2 == s) break;
        s = s2;
    }
    return s;
}

uint32_t aud_service(aud_arm *a, const aud_io *io) {
    a->io = io;
    uint64_t s = read_status(io);
    if ((s >> 56) != 0xA7) return 0;
    uint32_t T = (uint32_t)s;
    uint16_t E = (uint16_t)(s >> 32);
    /* the FPGA's counters went back (or jumped further than any stall): a new core session
       (the DDR3 still held the last one's status when this side started, or the core was
       reloaded) - start over. T moves ~100 ticks between passes; a jump of 2^30 is ~6 hours. */
    if (a->started && ((uint32_t)(T - a->last_T) > 0x40000000u ||
                       (uint16_t)(E - a->last_E) > 0x8000)) {
        a->started = 0;
        a->nq = 0;
        a->restarts++;
    }
    if (!a->started) {
        int fresh = 0;
        if (E == 0) return 0;
        if (E <= AUD_NEV) {
            /* every event since the core started may still be in the log: if the first is a
               pack reset stamped close to now, begin there so no tick is skipped. (E counts
               modulo 65536, so a small E alone does not mean a young session: the first
               event must be a pack reset, and recent.) */
            uint64_t h = io->rd(io->ctx, AUD_EV_Q);
            if ((h >> 56) != 0xE1 || (uint16_t)(h >> 40) != 0) return 0;     /* not in yet */
            if (((h >> 36) & 0xF) == 3 && (uint32_t)(T - (uint32_t)h) < AUD_NSLOT / 2) {
                a->next_t = (uint32_t)h;
                a->ev_next = 0;
                fresh = 1;
            }
        }
        a->want_sync = 0;
        if (!fresh) {
            /* started late: begin at the latest state event still in the log (the FPGA logs
               one every 2^SYNC_LOG2 ticks), so the BGM playing then resumes at its frame and the
               events after it play in order; the ticks from there to now are rendered as a
               catch-up (only SFX that started before it are lost). With none, the device is
               rebuilt from the events in the log and plays on from now. */
            uint16_t lo = E <= AUD_NEV ? 0 : (uint16_t)(E - AUD_NEV), i = E;
            late_load(a, io);
            a->next_t = T;
            a->ev_next = lo;
            while (i != lo) {
                i--;
                uint64_t h = io->rd(io->ctx, AUD_EV_Q + 2 * (i % AUD_NEV));
                if ((h >> 56) != 0xE1 || (uint16_t)(h >> 40) != i) break;   /* reused or not in */
                if (((h >> 36) & 0xF) == 4 && (uint32_t)(T - (uint32_t)h) < AUD_NSLOT / 2) {
                    a->next_t = (uint32_t)h;
                    a->ev_next = i;
                    a->want_sync = 1;
                    break;
                }
            }
        }
        a->started = 1;
    }
    a->last_T = T;
    a->last_E = E;
    if ((uint16_t)(E - a->ev_next) > AUD_NEV) {         /* fell behind the ring: lost events */
        a->bad_events += (uint16_t)(E - a->ev_next) - AUD_NEV;
        a->ev_next = (uint16_t)(E - AUD_NEV);
    }
    while (a->ev_next != E && a->nq < sizeof a->q / sizeof a->q[0]) {
        uint32_t q = AUD_EV_Q + 2 * (a->ev_next % AUD_NEV);
        uint64_t h = io->rd(io->ctx, q);
        if ((h >> 56) != 0xE1 || (uint16_t)(h >> 40) != a->ev_next) break;   /* not there yet */
        uint64_t pay = io->rd(io->ctx, q + 1);
        /* the FPGA writes a payload before its header: if the header is not the same after the
           payload was read, the slot was being reused (this side fell a ring behind) - stop and
           let the overrun check above count it next pass */
        if (io->rd(io->ctx, q) != h) break;
        aud_event *ev = &a->q[a->nq++];
        ev->payload = pay;
        ev->tick = (uint32_t)h;
        ev->kind = (h >> 36) & 0xF;
        ev->flags = (h >> 32) & 0xF;
        a->ev_next++;
    }
    /* ticks before T, or before the first event not read yet; at most AUD_NSLOT per pass, so a
       long stall is caught up over several passes instead of one that blocks for seconds (the
       slots it renders late are skipped by the FPGA's tag check either way) */
    uint32_t limit = T;
    if (a->ev_next != E) limit = a->nq ? a->q[a->nq - 1].tick : a->next_t;
    if ((int32_t)(limit - a->next_t) > (int32_t)AUD_NSLOT) limit = a->next_t + AUD_NSLOT;
    uint32_t n = 0, out[AUD_NV];
    unsigned qi = 0;
    for (;;) {
        /* an event stamped t applies before tick t: apply every one stamped at or before the
           next tick now, rendered or not (so a queue full of events of one tick drains) */
        while (qi < a->nq && (int32_t)(a->q[qi].tick - a->next_t) <= 0) apply(a, io, &a->q[qi++]);
        if ((int32_t)(limit - a->next_t) <= 0) break;
        aud_tick(a, out);
        uint32_t b = AUD_PCM_Q + 8 * (a->next_t % AUD_NSLOT);
        for (int k = 0; k < 4; k++) io->wr(io->ctx, b + 1 + k, (uint64_t)out[2 * k + 1] << 32 | out[2 * k]);
        io->wr(io->ctx, b, 0xD5ull << 56 | a->next_t);   /* the tag last */
        a->next_t++;
        a->ticks++;
        n++;
    }
    /* the events stamped after the next tick wait for it */
    memmove(a->q, a->q + qi, sizeof a->q[0] * (a->nq - qi));
    a->nq -= qi;
    return n;
}
