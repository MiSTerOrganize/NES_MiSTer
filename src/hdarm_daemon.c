/* hdarm_daemon.c - the ARM half of the display-list route on the MiSTer (OPEN_ITEMS item 11).
 *
 * Links with hdarm4.c built as a library (-DHDARM_LIB). Every frame it takes the frame record
 * the FPGA wrote (hd_capture), the watched-address values (hd3_pengine's watch copier), runs
 * Mesen's decisions (hdarm4) and publishes the display list hd3_paint executes. The protocol is
 * documented at the top of rtl/hdpack/hd3_pengine.sv; the addresses here must match it.
 *
 *   hdarm_daemon [--stats N] [--once] [--lag 0|1]
 *     --stats N   print timing every N frames (default 300; 0 = never)
 *     --once      exit after the first published frame (bring-up check)
 *     --lag 1     (default) publish frame F-1's finished list as soon as record F is ready,
 *                 then decide F: the painter never waits on a half-published list and the
 *                 write-back + copy + decisions of one frame get a whole frame period, for one
 *                 more frame of latency (OPEN_ITEMS item 11, lever (c)).
 *     --lag 0     publish each list as soon as it is decided (the earlier behaviour)
 *
 * DDR3 is mapped through /dev/mem with O_SYNC (uncached): the FPGA writes through the F2SDRAM
 * bridge, which the ARM caches do not snoop, so a cached mapping would read stale lines.
 * Uncached reads are slow, which is why the record is copied ONCE per frame into local memory
 * and the pack's decision sections once per pack.
 */
#define _GNU_SOURCE
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <pthread.h>
#include <sched.h>
#include "hdarm_rec.h"                /* fetch(), fetch_record(), REC_LEN, TRAILER */
#include "romfix_arm.h"               /* the ROM patcher's ARM half */
#include "aud_arm.h"                  /* HD audio's ARM half */

int hdarm_init(const uint8_t *pack, size_t len);
uint32_t hdarm_nwatched(void);
int hdarm_frame(const uint8_t *rec, const uint8_t *vals, uint32_t nvals,
                const uint64_t **list, size_t *nq);

#define PACK_PA    0x30000000u        /* the .hdp verbatim */
#define PACK_MAX   0x0C000000u        /* 192 MB */
#define REC_PA     0x3C000000u        /* bank b at + REC_LEN * b */
#define WVAL       0x000F0100u
#define WMRK       0x000F0200u
#define LIST_PA    0x3C200000u        /* buffer k at + 0x80000 * k */
#define LIST_LEN   0x00080000u
#define MBOX_PA    0x3C400000u        /* mailbox; pack-state qword at + 8 */
#define SEC_BLOCKS 4                  /* the block pixels: never read by the ARM */
#define SEC_ROMFIX 14                 /* ROM patch runs: the service thread copies its own */
#define SEC_AUDIO  15                 /* HD audio: the audio thread reads it, past 192 MB too */
/* the sections hdarm4 reads; the rest are only named in the header */
static int decision_section(int i) { return i != SEC_BLOCKS && i != SEC_ROMFIX && i != SEC_AUDIO; }

static volatile sig_atomic_t g_stop = 0;
static void on_signal(int s) { (void)s; g_stop = 1; }

static double now_ms(void) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec * 1e-6;
}

static int g_mem = -1;
static uint8_t *map(uint32_t pa, uint32_t len) {
    void *p = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, g_mem, pa);
    if (p == MAP_FAILED) { perror("mmap"); exit(1); }
    return p;
}

static uint64_t rd64(const volatile uint8_t *p) { return *(const volatile uint64_t *)p; }
static uint32_t le32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

/* one 64-bit store the FPGA sees whole: its mailbox read is a single qword */
static void wr64(volatile uint8_t *p, uint64_t v) {
#ifdef __ARM_NEON
    vst1_u64((uint64_t *)p, vcreate_u64(v));
#else
    *(volatile uint64_t *)p = v;
#endif
}

/* The pack: header + the sections the decisions read, into a buffer reaching the end of the
   last of them (calloc'd, so untouched ranges cost no memory). The block pixels, the ROM patch
   runs and the HD audio are left in DDR3 for the FPGA and the other threads: HD audio is most
   of a large pack and may lie past the 192 MB window (Dragon Quest III: 264 MB of it), which
   the decisions never need. */
static uint8_t *load_pack(const volatile uint8_t *pk, size_t *len) {
    uint8_t hdr[0x200];
    fetch(hdr, pk, sizeof hdr);
    if (memcmp(hdr, "NHDP", 4)) { fprintf(stderr, "hdarm: no pack header at 0x30000000\n"); return NULL; }
    size_t end = sizeof hdr;
    for (int i = 0; i < 18; i++) {
        uint32_t o = le32(hdr + 0x80 + 8 * i), s = le32(hdr + 0x84 + 8 * i);
        if (s && decision_section(i) && (size_t)o + s > end) end = (size_t)o + s;
    }
    if (end > PACK_MAX) { fprintf(stderr, "hdarm: decision sections reach past 192 MB\n"); return NULL; }
    uint8_t *buf = calloc(1, end);
    if (!buf) { fprintf(stderr, "hdarm: cannot allocate %zu bytes for the pack\n", end); return NULL; }
    memcpy(buf, hdr, sizeof hdr);
    for (int i = 0; i < 18; i++) {
        uint32_t o = le32(hdr + 0x80 + 8 * i), s = le32(hdr + 0x84 + 8 * i);
        if (s && decision_section(i)) fetch(buf + o, pk + o, s);
    }
    *len = end;
    return buf;
}

/* ------------------------------------------------------------------ the service thread
 * On core 1 (the display-list loop keeps core 0), every millisecond: the ROM patcher's ARM half
 * (romfix_arm.c; the FPGA half is rtl/hdpack/hd3_romfix.sv, the handshake is documented at its
 * top). When the status names a question not answered yet: copy the staged ROM and the pack's
 * section 14 out of DDR3, decide, write the target, then the command - the target lands before
 * the FPGA can see a command that names it. */
static volatile uint8_t *g_pk, *g_rf, *g_mbox;          /* pack, staged/target ROMs, mailbox */
#define RF_PA 0x3C800000u                                /* window byte RF_SRC_OFF; target + 4 MB */
/* The window is mapped strongly-ordered, where an unaligned load faults, and a section starts at
   any byte: copy whole aligned 128-byte blocks through a bounce buffer (every window here is
   128-byte aligned, and 192 MB is a multiple of 128, so a block never straddles two of them).
   The bounce is on the stack: the service and audio threads both copy. */
static void copy_aligned(volatile uint8_t *base, uint32_t off, uint32_t len, uint8_t *dst) {
    uint8_t bounce[128] __attribute__((aligned(16)));
    while (len) {
        uint32_t at = off & ~127u, k = off - at, n = 128 - k < len ? 128 - k : len;
        fetch(bounce, base + at, 128);
        memcpy(dst, bounce + k, n);
        dst += n; off += n; len -= n;
    }
}
static void romfix_service(rf_state *st, uint8_t *rom, uint8_t *tgt) {
    volatile uint8_t *stq = g_mbox + (RF_ST_OFF - 0x0C400000u), *cmq = g_mbox + (RF_CMD_OFF - 0x0C400000u);
    if (rf_session_check(st, rd64(cmq)))
        fprintf(stderr, "hdarm: ROM patcher: the core was reloaded, new session\n");
    uint64_t s = rd64(stq);
    if (!rf_pending(st, s)) return;
    uint32_t L = RF_LEN(s) > RF_MAXB ? RF_MAXB : RF_LEN(s);
    fetch(rom, g_rf, (L + 127) & ~127u);
    uint8_t hdr[8];
    uint32_t o14 = 0, n14 = 0;
    uint8_t *s14 = NULL;
    if (RF_ACTIVE(s)) {                                  /* (no active pack: 0xF0 is not a header) */
        copy_aligned(g_pk, 0xF0, 8, hdr);                /* section 14: offset, size */
        o14 = le32(hdr); n14 = le32(hdr + 4);
    }
    if (n14 && (uint64_t)o14 + n14 <= PACK_MAX && (s14 = malloc(n14)) != NULL) copy_aligned(g_pk, o14, n14, s14);
    else n14 = 0;
    uint32_t tl; uint64_t cmd;
    double t0 = now_ms();
    int w = rf_decide(st, s, rom, s14, n14, tgt, &tl, &cmd);
    free(s14);
    if (!w) return;
    memcpy((void *)(g_rf + (RF_TGT_OFF - RF_SRC_OFF)), tgt, tl);
    __sync_synchronize();
    wr64(cmq, cmd);
    /* The core reloaded while this was decided (hd3_romfix's power-up clear zeroes the status):
       the command answers the old session - take it back. The new session's rseq starts 0x40
       past the old one, so the FPGA would not take it anyway; this keeps the command qword at
       0 for the session check above. */
    __sync_synchronize();
    if (rd64(stq) == 0) {
        wr64(cmq, 0);
        rf_init(st);
        fprintf(stderr, "hdarm: ROM patcher: the core was reloaded while deciding, command withdrawn\n");
        return;
    }
    fprintf(stderr, "hdarm: ROM %u bytes: action %u result %u (%u bytes), %.1f ms\n", L,
            (unsigned)((cmd >> 30) & 3), (unsigned)((cmd >> 27) & 7), tl, now_ms() - t0);
}

static void *service(void *arg) {
    (void)arg;
    cpu_set_t cs; CPU_ZERO(&cs); CPU_SET(1, &cs);
    pthread_setaffinity_np(pthread_self(), sizeof cs, &cs);
    static rf_state st;                                  /* zero: answers the current status */
    uint8_t *rom = malloc(RF_MAXB + 128), *tgt = malloc(RF_MAXB);
    if (!rom || !tgt) { fprintf(stderr, "hdarm: out of memory (service)\n"); return NULL; }
    wr64(g_mbox + (RF_CMD_OFF - 0x0C400000u), 0);        /* no command from an earlier run */
    while (!g_stop) {
        romfix_service(&st, rom, tgt);
        usleep(1000);
    }
    return NULL;
}

/* ------------------------------------------------------------------ the audio thread
 * HD audio's ARM half (aud_arm.c; the FPGA half is rtl/hdpack/hd3_audio.sv, the handshake is
 * documented in aud_arm.h). Its own thread on core 1, so a long ROM-patch decision on the service
 * thread cannot starve it: the FPGA mixes each slot AUD_D (512) ticks, ~10.7 ms, after its tick,
 * and the thread renders whatever has become due every 2 ms. Window qwords are mapped from three
 * places: the status qword beside the mailbox, the event log + PCM slots at 0x3D000000, and the
 * pack (file bytes below 192 MB at 0x30000000, the rest in the overflow window at 0x24000000). */
#define AUD_PA   0x3D000000u                             /* event log; PCM slots at + 0x10000 */
#define AUD_LEN  0x00050000u
#define OVF_PA   0x24000000u                             /* pack file bytes >= 192 MB */
#define PACK_TOT 0x18000000u                             /* 384 MB: window + overflow */
static volatile uint8_t *g_aud, *g_ovf;
static volatile uint8_t *aud_ptr(uint32_t q) {
    uint32_t b = q * 8;
    if (b >= 0x0C400000u && b < 0x0C401000u) return g_mbox + (b - 0x0C400000u);
    if (b >= 0x0D000000u && b < 0x0D000000u + AUD_LEN) return g_aud + (b - 0x0D000000u);
    return NULL;
}
static uint64_t aud_rd(void *ctx, uint32_t q) {
    (void)ctx;
    volatile uint8_t *p = aud_ptr(q);
    if (!p) return 0;
#ifdef __ARM_NEON
    return vget_lane_u64(vld1_u64((const uint64_t *)p), 0);  /* one 64-bit read, never torn */
#else
    return rd64(p);
#endif
}
static void aud_wr(void *ctx, uint32_t q, uint64_t v) {
    (void)ctx;
    volatile uint8_t *p = aud_ptr(q);
    if (!p) return;
    /* a slot's tag (its first qword) is written after its samples: order them */
    if (q >= AUD_PCM_Q && ((q - AUD_PCM_Q) & 7) == 0) __sync_synchronize();
    wr64(p, v);
}
/* pack file bytes through copy_aligned: below 192 MB from the main window, the rest from the
   overflow window */
static int aud_fetch(void *ctx, uint32_t off, uint32_t len, uint8_t *dst) {
    (void)ctx;
    if ((uint64_t)off + len > PACK_TOT) return 1;
    if (off < PACK_MAX) {
        uint32_t n = len < PACK_MAX - off ? len : PACK_MAX - off;
        copy_aligned(g_pk, off, n, dst);
        dst += n; off += n; len -= n;
    }
    if (len) copy_aligned(g_ovf, off - PACK_MAX, len, dst);
    return 0;
}
static void *audio(void *arg) {
    (void)arg;
    cpu_set_t cs; CPU_ZERO(&cs); CPU_SET(1, &cs);
    pthread_setaffinity_np(pthread_self(), sizeof cs, &cs);
    struct sched_param sp = { .sched_priority = 10 };
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);   /* best effort: fine if refused */
    static aud_arm a;
    aud_init(&a);
    aud_io io = { aud_rd, aud_wr, aud_fetch, NULL };
    uint32_t restarts = 0, resets = 0;
    int started = 0;
    while (!g_stop) {
        aud_service(&a, &io);
        int restart = a.started != started || a.restarts != restarts;
        if (restart) {
            started = a.started; restarts = a.restarts;
            fprintf(stderr, "hdarm: audio log (re)started at tick %u\n", a.next_t);
        }
        if (restart || a.resets != resets) {  /* a pack reset or a late start loaded the section */
            resets = a.resets;
            fprintf(stderr, "hdarm: audio section: %d entries, restore checkpoints %s\n", a.n_ent,
                    !a.n_ent ? "-" : a.ck_loaded ? "from the pack" : "built in the background");
        }
        /* the rest of the pass builds restore checkpoints (a few hundred microseconds) */
        aud_build_checkpoints(&a, AUD_CKB_STEP);
        usleep(2000);
    }
    aud_free(&a);
    return NULL;
}

/* Copy a list into buffer (fid & 3), pad it to a multiple of 8 qwords, then name it in the
   mailbox: the list lands before the mailbox moves. */
static void publish(volatile uint8_t *lists, volatile uint8_t *mbox, uint32_t fid,
                    const uint64_t *dl, size_t nq) {
    size_t np = (nq + 7) & ~(size_t)7;
    volatile uint8_t *lb = lists + (fid & 3) * LIST_LEN;
    memcpy((void *)lb, dl, nq * 8);
    memset((void *)(lb + nq * 8), 0, (np - nq) * 8);
    __sync_synchronize();
    wr64(mbox, 0xA5ull << 56 | (uint64_t)np << 32 | fid);
}

/* ------------------------------------------------------------------ the pacer thread
 * The scan-out has no frame buffer, only an 8-line ring (hd3_scanout.sv): a frame whose list has
 * not been published by the time it is shown is BLACK. A frame whose decisions take longer than a
 * frame period (heavy packs: Mega Man Super spikes to 30+ ms) would flash black. The pacer, on
 * core 1, watches the frame records; when a new record appears while the main thread is still
 * deciding, nobody would publish for that frame, so it publishes the last list again under a new
 * frame id. The FPGA then paints the previous picture once more: a late frame is held for a frame
 * instead of going black. The main thread's list still goes out the moment it is ready.
 * All publishing (main thread and pacer) goes through g_pub_lock, which owns g_fid and the copy
 * of the last published list. */
static pthread_mutex_t g_pub_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile uint8_t *g_lists, *g_recs;
static uint32_t g_fid;                       /* last frame id published (under g_pub_lock) */
static uint64_t *g_last;                     /* the last published list (under g_pub_lock) */
static size_t g_last_nq;
static int g_have_last;                      /* g_last belongs to the current pack */
static volatile int g_busy;                  /* main thread is copying/deciding a record */
static volatile unsigned g_held;             /* pacer re-publishes since the last stats line */

static void publish_main(const uint64_t *dl, size_t nq) {
    pthread_mutex_lock(&g_pub_lock);
    g_fid++; if (!g_fid) g_fid = 1;
    publish(g_lists, g_mbox, g_fid, dl, nq);
    memcpy(g_last, dl, nq * 8); g_last_nq = nq; g_have_last = 1;
    pthread_mutex_unlock(&g_pub_lock);
}

static void forget_last(void) {              /* the pack changed or went away */
    pthread_mutex_lock(&g_pub_lock);
    g_have_last = 0;
    pthread_mutex_unlock(&g_pub_lock);
}

static void *pacer(void *arg) {
    (void)arg;
    cpu_set_t cs; CPU_ZERO(&cs); CPU_SET(1, &cs);
    pthread_setaffinity_np(pthread_self(), sizeof cs, &cs);
    struct sched_param sp = { .sched_priority = 5 };
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);   /* best effort: fine if refused */
    uint32_t seen = 0;
    while (!g_stop) {
        uint32_t f0 = (uint32_t)rd64(g_recs + TRAILER), f1 = (uint32_t)rd64(g_recs + REC_LEN + TRAILER);
        uint32_t N = f0 > f1 ? f0 : f1;
        if (N > seen) {
            seen = N;
            if (g_busy) {
                pthread_mutex_lock(&g_pub_lock);
                if (g_have_last && g_busy) {
                    g_fid++; if (!g_fid) g_fid = 1;
                    publish(g_lists, g_mbox, g_fid, g_last, g_last_nq);
                    g_held++;
                }
                pthread_mutex_unlock(&g_pub_lock);
            }
        } else if (N < seen) {
            seen = N;                        /* the NES was reset: frame numbers restarted */
        }
        usleep(250);
    }
    return NULL;
}

int main(int argc, char **argv) {
    int stats = 300, once = 0, lag = 1;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--stats") && i + 1 < argc) stats = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--once")) once = 1;
        else if (!strcmp(argv[i], "--lag") && i + 1 < argc) lag = atoi(argv[++i]) != 0;
        else { fprintf(stderr, "usage: hdarm_daemon [--stats N] [--once] [--lag 0|1]\n"); return 2; }
    }
    if (once) lag = 0;                                 /* bring-up: publish the first list now */
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    g_mem = open("/dev/mem", O_RDWR | O_SYNC);
    if (g_mem < 0) { perror("/dev/mem"); return 1; }
    volatile uint8_t *pk   = map(PACK_PA, PACK_MAX);
    volatile uint8_t *recs = map(REC_PA, 2 * REC_LEN);
    volatile uint8_t *lists = map(LIST_PA, 4 * LIST_LEN);
    volatile uint8_t *mbox = map(MBOX_PA, 0x1000);
    g_pk = pk; g_mbox = mbox; g_rf = map(RF_PA, 2 * RF_MAXB);
    g_aud = map(AUD_PA, AUD_LEN); g_ovf = map(OVF_PA, PACK_MAX);
    {   /* the display-list loop on core 0, the core with the faster DDR3 reads (its decision
           time is measured there, hdarm4.c); the two threads below put themselves on core 1 */
        cpu_set_t cs; CPU_ZERO(&cs); CPU_SET(0, &cs);
        pthread_setaffinity_np(pthread_self(), sizeof cs, &cs);
    }
    g_lists = lists; g_recs = recs;
    g_last = malloc(LIST_LEN);          /* the pacer's copy of the last published list */
    if (!g_last) { fprintf(stderr, "hdarm: out of memory\n"); return 1; }
    pthread_t svc, aud, pac;
    if (pthread_create(&svc, NULL, service, NULL)) { perror("pthread_create"); return 1; }
    if (pthread_create(&aud, NULL, audio, NULL)) { perror("pthread_create"); return 1; }
    if (!once && pthread_create(&pac, NULL, pacer, NULL)) { perror("pthread_create"); return 1; }

    uint8_t *rec = calloc(1, REC_LEN);  /* fetch_record fills only the parts hdarm4 reads */
    uint64_t *held = malloc(LIST_LEN);  /* --lag 1: the decided list waiting for the next record */
    size_t held_nq = 0;
    int have_held = 0;
    if (!rec || !held) { fprintf(stderr, "hdarm: out of memory\n"); return 1; }
    uint8_t *pack = NULL;
    uint32_t gen = 0, last = 0, nw = 0;
    int refused = 0;                   /* the pack of generation gen was refused */
    wr64(mbox, 0);                                     /* nothing published */
    double t_copy = 0, t_dec = 0, t_pub = 0, t_worst = 0;
    unsigned n = 0, skipped = 0, torn = 0;
    size_t maxq = 0;

    while (!g_stop) {
        uint64_t pw = rd64(mbox + 8);
        int valid = (pw >> 56) == 0x5A && ((pw >> 48) & 1);
        if (!valid) {                                  /* no pack: stay quiet */
            if (pack) { free(pack); pack = NULL; wr64(mbox, 0); fprintf(stderr, "hdarm: pack gone\n"); }
            have_held = 0;                             /* a held list belongs to the old pack */
            forget_last();
            usleep(20000); continue;
        }
        if (refused && (uint32_t)pw == gen) { usleep(20000); continue; }   /* a refused pack is tried once */
        if (!pack || (uint32_t)pw != gen) {
            refused = 0;
            free(pack); pack = NULL; have_held = 0;
            forget_last();
            size_t len;
            double t0 = now_ms();
            pack = load_pack(pk, &len);
            if (!pack || hdarm_init(pack, len)) {
                fprintf(stderr, "hdarm: pack refused\n");
                free(pack); pack = NULL; gen = (uint32_t)pw; refused = 1; continue;
            }
            gen = (uint32_t)pw; nw = hdarm_nwatched(); last = 0;
            fprintf(stderr, "hdarm: pack generation %u loaded, %zu bytes, %u watched, %.1f ms\n",
                    gen, len, nw, now_ms() - t0);
        }
        /* the newest finished record */
        uint32_t f0 = (uint32_t)rd64(recs + TRAILER), f1 = (uint32_t)rd64(recs + REC_LEN + TRAILER);
        uint32_t F = f0 > f1 ? f0 : f1, b = f0 > f1 ? 0 : 1;
        if (F <= last || (F & 1) != b) { usleep(500); continue; }
        volatile uint8_t *bank = recs + b * REC_LEN;
        uint8_t vals[256] = {0};
        if (nw) {
            uint64_t mk = rd64(bank + WMRK);
            if ((uint32_t)mk != F) { usleep(200); continue; }   /* values not copied yet */
            fetch(vals, bank + WVAL, 256);
        }
        double t0 = now_ms();
        if (have_held) {                               /* --lag 1: F-1's list goes out first */
            publish_main(held, held_nq);
            have_held = 0;
        }
        double tp = now_ms();
        g_busy = 1;                                    /* from here a new record finds us busy */
        fetch_record(rec, bank);
        if ((uint32_t)rd64(bank + TRAILER) != F) { g_busy = 0; torn++; continue; }  /* overwritten mid-copy */
        double t1 = now_ms();
        const uint64_t *dl; size_t nq;
        int refused = hdarm_frame(rec, vals, nw, &dl, &nq);
        g_busy = 0;
        if (refused) { fprintf(stderr, "hdarm: frame %u refused\n", F); last = F; continue; }
        double t2 = now_ms();
        size_t np = (nq + 7) & ~(size_t)7;
        if (np * 8 > LIST_LEN) { fprintf(stderr, "hdarm: list of %zu qwords too long\n", nq); last = F; continue; }
        if (last && F > last + 1) skipped += F - last - 1;
        last = F;
        if (lag) {                                     /* hdarm_frame's list is its own buffer */
            memcpy(held, dl, nq * 8); held_nq = nq; have_held = 1;
        } else {
            publish_main(dl, nq);
        }
        double t3 = now_ms();
        t_copy += t1 - tp; t_dec += t2 - t1; t_pub += (tp - t0) + (t3 - t2);
        if (t3 - t0 > t_worst) t_worst = t3 - t0;
        if (np > maxq) maxq = np;
        n++;
        if (once) break;
        if (stats && n % stats == 0) {
            unsigned held_now = g_held; g_held = 0;
            fprintf(stderr, "hdarm: %u frames  copy %.2f  decide %.2f  publish %.2f ms avg  worst %.2f  "
                    "skipped %u  held %u  torn %u  max list %zu\n",
                    n, t_copy / stats, t_dec / stats, t_pub / stats, t_worst, skipped, held_now, torn, maxq);
            t_copy = t_dec = t_pub = t_worst = 0; skipped = torn = 0;
        }
    }
    wr64(mbox, 0);
    g_stop = 1;
    pthread_join(svc, NULL);
    pthread_join(aud, NULL);
    if (!once) pthread_join(pac, NULL);
    fprintf(stderr, "hdarm: stopped after %u frames\n", n);
    return 0;
}
