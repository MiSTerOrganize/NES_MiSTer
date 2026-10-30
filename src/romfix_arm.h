/* romfix_arm.h - the ARM half of the pack's <patch> (stage 5; HDPACK_DESIGN.md "Stage 5 - ROM
 * patches as built"). rtl/hdpack/hd3_romfix.sv stages the ROM and replays what this decides.
 *
 * Shared by hdarm_daemon.c (on the MiSTer) and the testbenches (tools/sim3/tb_engine.cpp,
 * tools/fullsim/fullsim.cpp), so the tests run the code the device runs. No I/O: the caller
 * reads the status qword and the staged ROM out of DDR3, and writes the target and then the
 * command back.
 */
#ifndef ROMFIX_ARM_H
#define ROMFIX_ARM_H
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

#define RF_MAXB    0x400000u          /* staged and target areas, 4 MB each (hd3_romfix MAXB) */
#define RF_SRC_OFF 0x0C800000u        /* window byte offsets (+ 0x30000000 physical) */
#define RF_TGT_OFF 0x0CC00000u
#define RF_ST_OFF  0x0C400040u        /* status qword; command at + 8 */
#define RF_CMD_OFF 0x0C400048u

typedef struct {
    int     have;                     /* a status has been decided on */
    uint8_t rseq, aseq;               /* the status last decided on */
    uint8_t id;                       /* the last command id issued */
    int     hashed;                   /* sha_raw / sha_n belong to ROM hrseq */
    uint8_t hrseq;
    uint8_t sha_raw[20], sha_n[20];
    int     n_cand;                   /* an iNES file with something in bytes 7-15 */
} rf_state;

/* status fields (hd3_romfix.sv) */
#define RF_MAGIC(s)   ((unsigned)((s) >> 56) & 0xFF)
#define RF_RSEQ(s)    ((unsigned)((s) >> 48) & 0xFF)
#define RF_ASEQ(s)    ((unsigned)((s) >> 40) & 0xFF)
#define RF_LASTID(s)  ((unsigned)((s) >> 32) & 0xFF)
#define RF_ACTIVE(s)  ((unsigned)((s) >> 31) & 1)
#define RF_CONTENT(s) ((unsigned)((s) >> 28) & 3)
#define RF_STAGED(s)  ((unsigned)((s) >> 27) & 1)
#define RF_LEN(s)     ((uint32_t)((s) & 0xFFFFFF))

void rf_init(rf_state *st);

/* A new FPGA session: hd3_romfix writes 0 to the command qword at power-up, before any status
   (and starts its rseq 0x40 past the old status's, so no old answer can match a new question).
   Call with the command qword before rf_pending: when this side has answered something and
   now reads 0 there, the core was reloaded - forget the old session, so nothing of it is
   carried into the new one. Returns 1 when it reset. */
int rf_session_check(rf_state *st, uint64_t cmd_now);

/* 1 when status s asks a question not answered yet (the caller then fetches the staged ROM,
   RF_LEN(s) bytes, and calls rf_decide). */
int rf_pending(const rf_state *st, uint64_t s);

/* Decide for status s. rom: the staged file (RF_LEN(s) bytes). s14 / s14_len: section 14 of
   the pack the FPGA has loaded (NULL / 0 when none). tgt: RF_MAXB bytes of scratch. On return
   *tgt_len is the number of target bytes to copy to the target area BEFORE the command is
   written (0 = none), and *cmd the command qword. Returns 1 when a command must be written. */
int rf_decide(rf_state *st, uint64_t s, const uint8_t *rom, const uint8_t *s14, uint32_t s14_len,
              uint8_t *tgt, uint32_t *tgt_len, uint64_t *cmd);

/* The build alone, for the host tests: entry e (64 bytes inside section 14), the source form
   (p2 = with header bytes 7-15 read as 0). Returns the target length, or 0 with *why set to
   3 (refused) or 4 (CRC). */
uint32_t rf_build(const uint8_t *s14, uint32_t s14_len, const uint8_t *e, const uint8_t *src,
                  uint32_t L, int p2, uint8_t *tgt, int *why);

void rf_sha1(const uint8_t *d, size_t n, uint8_t out[20]);
#ifdef __cplusplus
}
#endif
#endif
