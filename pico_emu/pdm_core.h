#pragma once
/*
 * Platform-independent core of the PDM emulator: sigma-delta modulator and bit packing for the PIO program.
 * Header-only so the firmware (main.c) and the PC test (test/host_test.c) run exactly the same code.
 */
#include <stdbool.h>
#include <stdint.h>

#ifndef PDM_RAMFUNC
#define PDM_RAMFUNC(name) name
#endif

#define PDM_NCH         6
#define PDM_OSR         64           /* PDM clock cycles per PCM sample (daq-detector: PDM clock = 64 fs) */
#define PDM_WORDS_PER_FRAME (PDM_OSR / 4)   /* 4 PDM cycles = 8 halves x 3 bits = 24 bits per FIFO word */
#define SD_FS           32768        /* feedback level, Q15 full scale */
#define SD_LIMIT        24576        /* input clamp at 0.75 FS: a 2nd-order modulator overloads near 1 FS */

typedef struct {
    int32_t i1, i2, y, prev;
} sd_state_t;

static inline void sd_reset(sd_state_t *s)
{
    s->i1 = s->i2 = s->prev = 0;
    s->y = -SD_FS;
}

/*
 * One channel, n frames of interleaved PCM (stride PDM_NCH): gain (Q12), x64 linear interpolation and a
 * 2nd-order modulator (two integrators, 1-bit quantizer).
 * out[2f] = PDM cycles 0..31 of frame f, out[2f+1] = cycles 32..63, bit k = cycle k, 1 = +FS.
 */
static void PDM_RAMFUNC(sd_block)(sd_state_t *s, const int16_t *x, int n, uint32_t *out, int32_t gain_q12)
{
    int32_t i1 = s->i1, i2 = s->i2, y = s->y, prev = s->prev;
    for (int f = 0; f < n; f++) {
        int32_t cur = (x[f * PDM_NCH] * gain_q12) >> 12;
        cur = cur > SD_LIMIT ? SD_LIMIT : (cur < -SD_LIMIT ? -SD_LIMIT : cur);
        int32_t d = cur - prev;
        int32_t acc = prev * PDM_OSR;
        for (int h = 0; h < 2; h++) {
            uint32_t m = 0;
#pragma GCC unroll 32
            for (int j = 0; j < 32; j++) {
                acc += d;
                i1 += (acc >> 6) - y;
                i2 += i1 - y;
                int32_t sg = i2 >> 31;               /* 0 -> bit 1 (+FS), -1 -> bit 0 (-FS) */
                y = (SD_FS ^ sg) - sg;
                m = (m >> 1) | ((uint32_t)~sg & 0x80000000u);
            }
            out[2 * f + h] = m;
        }
        prev = cur;
    }
    s->i1 = i1;
    s->i2 = i2;
    s->y = y;
    s->prev = prev;
}

/*
 * Mic m's nibble (4 PDM cycles) -> its bits in a FIFO word. Word bit 3k + line = clock half k
 * (even k: after the rising edge). Line j = m % 3 carries M(j+1) in the first half of every cycle and
 * M(j+4) in the second (swap exchanges them).
 */
static inline void pdm_build_lut(uint32_t lut[PDM_NCH][16], bool swap)
{
    for (int m = 0; m < PDM_NCH; m++) {
        int line = m % 3, second = (m >= 3) ^ (swap ? 1 : 0);
        for (int n = 0; n < 16; n++) {
            uint32_t v = 0;
            for (int k = 0; k < 4; k++) {
                if (n >> k & 1) {
                    v |= 1u << (3 * (2 * k + second) + line);
                }
            }
            lut[m][n] = v;
        }
    }
}

/* Packs n frames: masks[m][2f + half] -> words ring[(w0 + i) & ring_mask]; returns the next word index */
static uint32_t PDM_RAMFUNC(pdm_pack)(uint32_t *ring, uint32_t ring_mask, uint32_t w, const uint32_t (*lut)[16],
                                      uint32_t *const masks[PDM_NCH], int n)
{
    for (int f = 0; f < n; f++) {
        for (int half = 0; half < 2; half++) {
            int i = 2 * f + half;
            uint32_t m0 = masks[0][i], m1 = masks[1][i], m2 = masks[2][i];
            uint32_t m3 = masks[3][i], m4 = masks[4][i], m5 = masks[5][i];
            for (int sh = 0; sh < 32; sh += 4) {
                ring[w & ring_mask] = lut[0][(m0 >> sh) & 15] | lut[1][(m1 >> sh) & 15] | lut[2][(m2 >> sh) & 15] |
                                      lut[3][(m3 >> sh) & 15] | lut[4][(m4 >> sh) & 15] | lut[5][(m5 >> sh) & 15];
                w++;
            }
        }
    }
    return w;
}
