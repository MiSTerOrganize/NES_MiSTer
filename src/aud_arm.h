/* aud_arm.h - the ARM half of stage-6 HD audio (OPEN_ITEMS item 11, 2026-10-28: audio moved off
 * the FPGA). The FPGA half is rtl/hdpack/hd3_audio.sv; the handshake is documented at its top.
 *
 * The FPGA keeps the registers, the status the CPU reads, a voice tracker for both, and the
 * final mix. It logs every write the device takes, every savestate restore and every pack reset
 * to DDR3, stamped with the 48 kHz tick it applies before, and publishes how many ticks it has
 * mixed. This side runs hdaudio.Device (tools/audio/hdaudio.py) on that log - the ADPCM decode
 * and the volumes - and writes, for every tick, the volume-scaled sample of each voice in mix
 * order (BGM, then the SFX in start order) to a PCM slot the FPGA reads AUD_D ticks later.
 *
 * DDR3 (window qword indices; the window is 0x30000000):
 *   AUD_ST_Q   status  {8'hA7, 8'd0, events logged 16, ticks mixed 32}
 *   AUD_EV_Q   event i at qwords 2*(i % AUD_NEV) (header), +1 (payload)
 *              header {8'hE1, i 16, kind 4, flags 4, tick 32}
 *              kind 1: CPU write, payload {reg 3, value 8}
 *              kind 2: savestate restore, payload = word A (hdaudio.Device.save)
 *              kind 3: pack reset, payload {section length 32, 3'd0, section offset 29}
 *              kind 4: state, every 2^SYNC_LOG2 ticks: payload = word A of now, flags bit 0 =
 *                      BGM paused. Ignored by a side that started with the session; one that
 *                      started late begins at the latest (the BGM resumes at its frame).
 *   AUD_PCM_Q  slot t % AUD_NSLOT at qwords 8*(t % AUD_NSLOT): {8'hD5, 24'd0, t},
 *              then 4 qwords of 2 x {R 16, L 16}: mix positions 0..7, unused = 0
 *
 * Every event stamped t applies before tick t, and every event stamped < T is in the log
 * before a status reading T, so ticks < T can be rendered.
 */
#ifndef AUD_ARM_H
#define AUD_ARM_H
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

#define AUD_ST_Q   0x188000Au     /* byte 0xC400050 */
#define AUD_EV_Q   0x1A00000u     /* byte 0xD000000 */
#define AUD_PCM_Q  0x1A02000u     /* byte 0xD010000 */
#define AUD_NEV    2048u
#define AUD_NSLOT  4096u
#define AUD_NV     8              /* mix positions: BGM + 7 SFX */
#define AUD_SFX    7
#define AUD_MAXENT 512
#define AUD_CK     16384u         /* ADPCM state checkpoint spacing, frames: a restore decodes at most
                                     AUD_CK - 1 frames in one pass (2.2 ms on the A9 at 135 ns a frame,
                                     measured 2026-10-29), which must fit inside AUD_D with room */
#ifndef AUD_QN
#define AUD_QN     256            /* events read ahead of the render (the tests build it at 2) */
#endif

typedef struct {
    uint64_t (*rd)(void *ctx, uint32_t q);              /* one window qword */
    void (*wr)(void *ctx, uint32_t q, uint64_t v);
    /* pack file bytes [off, off + len) into dst; 0 on success */
    int (*fetch)(void *ctx, uint32_t off, uint32_t len, uint8_t *dst);
    void *ctx;
} aud_io;

typedef struct {
    uint32_t key, start, frames, loop;
    uint64_t s0, sl;
    uint64_t *ck;                 /* state at frame k * AUD_CK (first pass), when ckv[k] */
    uint8_t *ckv;
    uint32_t nck;
} aud_ent;

/* Stream bytes are read from the pack in DDR3 a block at a time (the section can be over
   100 MB, so it is never copied whole): each voice keeps the block it is playing. */
#define AUD_BLK    256u

typedef struct {
    int on, e, loop;
    uint32_t pos;
    int pl, il, pr, ir;
    int bad;                      /* a block of this stream could not be fetched: its state is
                                     not the stream's, so no checkpoint is kept from it */
    uint32_t blk_at;              /* section offset of blk, or UINT32_MAX */
    uint8_t blk[AUD_BLK];
} aud_voice;

/* frames the checkpoint builder decodes per aud_service pass (a few hundred microseconds) */
#ifndef AUD_CKB_STEP
#define AUD_CKB_STEP 8192u
#endif

typedef struct {
    uint64_t payload;
    uint32_t tick;
    uint8_t kind, flags;
} aud_event;

typedef struct {
    int started;                  /* next_t / ev_next set from the first status */
    int want_sync;                /* started late at a state event: apply it (then 0) */
    uint32_t next_t;              /* next tick to render */
    uint16_t ev_next;             /* next event index to read */
    uint32_t last_T;              /* the last status read, to see a new session */
    uint16_t last_E;
    aud_event q[AUD_QN];          /* read, not yet applied */
    unsigned nq;
    /* the section (where it is in the pack file) and the device (hdaudio.Device) */
    const aud_io *io;             /* set by aud_service; stream blocks are fetched through it */
    uint32_t sec_off, sec_len;
    int n_ent;
    aud_ent ent[AUD_MAXENT];
    int album, options, paused, bvol, svol;
    aud_voice bgm;
    aud_voice sfx[AUD_SFX];
    int n_sfx;
    /* the checkpoint builder: decodes every stream once, in the background, so a restore far
       into a stream starts at most AUD_CK frames before its frame (ckb_e = the entry it is on;
       n_ent when done) */
    int ckb_e;
    aud_voice ckb;
    int ck_loaded;                /* the section's own checkpoint table was taken (load_checkpoints) */
    /* counters */
    uint32_t ticks, events, resets, bad_events, fetch_fail, restarts;
} aud_arm;

void aud_init(aud_arm *a);
void aud_free(aud_arm *a);
/* one pass: read the status, take the new events, render every tick that can be rendered.
 * Returns the ticks rendered. */
uint32_t aud_service(aud_arm *a, const aud_io *io);

/* the device on its own: load the section at pack file bytes [off, off + len) through io (its
   header and entry table are read now, its streams as they play), a write, a restore, a tick */
int  aud_load_section(aud_arm *a, const aud_io *io, uint32_t off, uint32_t len);
/* decode up to budget frames of the checkpoint builder; 1 while there is more to do */
int  aud_build_checkpoints(aud_arm *a, uint32_t budget);
void aud_write(aud_arm *a, int reg, int v);
void aud_restore(aud_arm *a, uint64_t word_a);
/* one tick: out[k] = {R << 16 | L} for mix position k (0 = none) */
void aud_tick(aud_arm *a, uint32_t out[AUD_NV]);
#ifdef __cplusplus
}
#endif
#endif
