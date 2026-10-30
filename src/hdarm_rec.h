/* hdarm_rec.h - what the ARM copies out of a frame record (shared by hdarm_daemon.c and its
 * host test rec_subset_test.c, so the test checks the code that runs on the device). */
#ifndef HDARM_REC_H
#define HDARM_REC_H
#include <stdint.h>
#include <string.h>
#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

#define REC_LEN    0x00100000u        /* one record bank */
#define TRAILER    0x000F0000u        /* frame number of the record in the bank */

/* copy out of uncached memory. The DDR3 window is outside System RAM, so /dev/mem maps it
   strongly-ordered and every load instruction is its own bus transaction: what sets the speed
   is bytes per instruction. Measured on the A9 (2026-10-22, dmem_bench, core 0): one VLDM of
   16 D registers (128 bytes) 340-365 MB/s; four 16-byte VLD1s 82 MB/s; memcpy 55 MB/s. Whole
   128-byte blocks go through VLDM, the rest through the 64-byte loop. Read-back of a written
   pattern compared equal both ways. */
static void fetch(void *dst, const volatile uint8_t *src, size_t n) {
#ifdef __ARM_NEON
    uint8_t *d = dst; size_t i = 0;
#ifdef __arm__
    for (; i + 128 <= n; i += 128)
        __asm__ volatile("vldmia %0, {d0-d15}\n\tvstmia %1, {d0-d15}"
                         : : "r"((const uint8_t *)src + i), "r"(d + i)
                         : "memory", "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
                           "d8", "d9", "d10", "d11", "d12", "d13", "d14", "d15");
#endif
    for (; i + 64 <= n; i += 64) {
        const uint8_t *s = (const uint8_t *)src + i;
        uint8x16_t a = vld1q_u8(s), b = vld1q_u8(s + 16), c = vld1q_u8(s + 32), e = vld1q_u8(s + 48);
        vst1q_u8(d + i, a); vst1q_u8(d + i + 16, b); vst1q_u8(d + i + 32, c); vst1q_u8(d + i + 48, e);
    }
    for (; i < n; i++) d[i] = src[i];
#else
    memcpy(dst, (const void *)src, n);
#endif
}

/* The parts of a frame record hdarm4 reads (hdarm4.c build_records / tile_rec), copied to the
   same offsets: the per-pixel tile entries (240 lines x 1 KB) and, per line, the 2 KB header's
   first 64 bytes (line word + palette), its spans (16 B each from +64) and its sprites (16 B
   each from +640) - only as many as the line word says - and the trailer. About a third of
   the 1 MB bank. The clamps are build_records' own. The line word is read first (one 8-byte
   access); the header and its spans are then one block from +0, and the sprites one block
   from +640, each rounded up to 128 bytes so they go through VLDM. The rounding stays inside
   the line's 2 KB block (at most +640 and +1664) and only over-reads, so the bytes hdarm4
   reads are the same. */
static size_t up128(size_t n) { return (n + 127) & ~(size_t)127; }
static void fetch_record(uint8_t *dst, const volatile uint8_t *bank) {
    fetch(dst, bank, 240 * 1024);
    for (int y = 0; y < 240; y++) {
        uint32_t o = 0x40000 + y * 2048;
        uint64_t q0 = *(const volatile uint64_t *)(bank + o);
        int nspan = (int)((q0 >> 32) & 0xFF); if (nspan > 34) nspan = 34;
        int nspr  = (int)((q0 >> 48) & 0xFF); if (nspr > 64) nspr = 64;
        fetch(dst + o, bank + o, up128(64 + (size_t)nspan * 16));
        if (nspr) fetch(dst + o + 640, bank + o + 640, up128((size_t)nspr * 16));
    }
    fetch(dst + TRAILER, bank + TRAILER, 8);
}
#endif
